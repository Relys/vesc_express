/*
    Copyright 2022‑2024 Benjamin Vedder     <benjamin@vedder.se>
    Copyright 2022      Joel Svensson      <svenssonjoel@yahoo.se>

    This file is part of the VESC firmware and is released under the
    terms of the GNU General Public License, version 3 (or any later).

    -----------------------------------------------------------------
    FreeRTOS / ESP‑IDF PORT
    -----------------------------------------------------------------
    All ChibiOS threading primitives have been replaced with their
    FreeRTOS counterparts so that `(spawn …)` and related Lisp helpers
    work unmodified on ESP32 targets.  No other functionality has been
    changed.
*/

#pragma GCC optimize("Os")

#include "ahrs.h"
#include "commands.h"
#include "comm_can.h"
#include "extensions.h"
#include "packet.h"
#include "lbm_flat_value.h"
#include "flash_helper.h"
#include "lispif.h"
#include "lispbm.h"
#include "mempools.h"
#include "terminal.h"
#include "utils.h"
#include "imu.h"
#include "main.h"
#include "extensions/vesc_c_if.h"
#include "conf_custom.h"
#include "freertos/task.h"
#include "lispif_rgbled_extensions.h"
#include "lbm_color_extensions.h"

typedef struct {
	const char *name;
	void *arg;
	void (*func)(void *);
	volatile bool should_terminate;
	TaskHandle_t handle;
} lib_thd_info;

#define LIB_MAX_THREADS 20
static lib_thd_info *lib_thread_infos[LIB_MAX_THREADS] = {0};
static size_t lib_thread_infos_cnt                     = 0;

#define LIB_NUM_MAX 10

static lib_info loaded_libs[LIB_NUM_MAX] = {0};

__attribute__((section(".libif"))) static volatile union {
	vesc_c_if cif;
	char pad[2048];
} cif;
static bool lib_init_done = false;

static TaskHandle_t lib_running_threads[20];
static size_t lib_running_threads_cnt = 0;

static void lib_sleep_ms(uint32_t ms) {
	vTaskDelay(pdMS_TO_TICKS(ms));
}

static void lib_sleep_us(uint32_t us) {
	esp_rom_delay_us(us);
}

static float lib_system_time(void) {
	return UTILS_AGE_S(0);
}

static float lib_ts_to_age_s(TickType_t ts) {
	return UTILS_AGE_S(ts);
}

static void lib_thd(void *arg) {
	lib_thd_info *t = (lib_thd_info *)arg;

	// Set thread-local storage for should_terminate check
	vTaskSetThreadLocalStoragePointer(NULL, 0, t);

	t->func(t->arg);

	// Task finished, remove from global tracking
	for (size_t i = 0; i < lib_thread_infos_cnt; i++) {
		if (lib_thread_infos[i] == t) {
			// Shift down remaining entries
			for (size_t j = i; j < lib_thread_infos_cnt - 1; j++) {
				lib_thread_infos[j] = lib_thread_infos[j + 1];
			}
			lib_thread_infos[--lib_thread_infos_cnt] = NULL;
			break;
		}
	}

	lbm_free(t);
	vTaskDelete(NULL); // clean self-termination
}

static bool lib_should_terminate(void) {
	lib_thd_info *info =
		(lib_thd_info *)pvTaskGetThreadLocalStoragePointer(NULL, 0);
	return info && info->should_terminate;
}

lib_thread lispif_spawn(
	void (*func)(void *), size_t stack_size, char *name, void *arg
) {
	commands_printf_lisp(
		"Spawn: func=%p, stack=%u, name=%s", func, stack_size,
		name ? name : "NULL"
	);

	if (!utils_is_func_valid(func)) {
		commands_printf_lisp(
			"Invalid function address. Make sure that the function is static."
		);
		return 0;
	}

	lib_thd_info *info = lbm_malloc_reserve(sizeof(lib_thd_info));
	commands_printf_lisp("Thread info: %p", info);

	if (info) {
		info->arg              = arg;
		info->func             = func;
		info->name             = name;
		info->should_terminate = false;

		TaskHandle_t thd = NULL;
		commands_printf_lisp("Creating task...");

		// FreeRTOS allocates its own stack - use words not bytes
		BaseType_t result = xTaskCreatePinnedToCore(
			lib_thd, name ? name : "lib-thd",
			stack_size, // Convert bytes to words
			info, 6, &thd, tskNO_AFFINITY
		);

		commands_printf_lisp("xTaskCreate result: %d, handle: %p", result, thd);

		if (result == pdPASS && thd != NULL) {
			if (lib_running_threads_cnt
				< (sizeof(lib_running_threads)
				   / sizeof(lib_running_threads[0]))) {
				lib_running_threads[lib_running_threads_cnt++] = thd;
			}
			commands_printf_lisp("Task created successfully!");
			return (lib_thread)thd;
		} else {
			commands_printf_lisp("Task creation FAILED! Result: %d", result);
			lbm_free(info);
		}
	} else {
		commands_printf_lisp("Thread info allocation failed!");
	}

	return 0;
}

static void lib_request_terminate(lib_thread thd) {
	lib_thd_info *info =
		(lib_thd_info *)pvTaskGetThreadLocalStoragePointer(thd, 0);
	if (info) {
		info->should_terminate = true;
	}

	// Wait for task to self-terminate
	int timeout = 2000;
	while (eTaskGetState(thd) != eDeleted && timeout-- > 0) {
		vTaskDelay(pdMS_TO_TICKS(1));
	}

	if (timeout <= 0) {
		commands_printf_lisp("Not terminating, crashing...");
		vTaskDelay(pdMS_TO_TICKS(20));
		taskENTER_CRITICAL(NULL);
		for (;;) {
			__NOP();
		}
	}
}

static void **lib_get_arg(uint32_t prog_addr) {
    // Normalize in case the caller passed a DROM address
    uint32_t p = (uint32_t)utils_drom_to_irom((void*)prog_addr);

    for (int i = 0; i < LIB_NUM_MAX; i++) {
        uint32_t base = loaded_libs[i].base_addr;
        if (!base) continue;

        // Accept either header+4 (prog field) or the header base itself
        if (p == base + 4u) {
            return &loaded_libs[i].arg;
        }
    }
    return NULL;
}

static bool lib_create_byte_array(lbm_value *value, lbm_uint num_elt) {
	return lbm_heap_allocate_array(value, num_elt);
}

static bool lib_eval_is_paused(void) {
	return lbm_get_eval_state() == EVAL_CPS_STATE_PAUSED;
}

static lib_mutex lib_mutex_create(void) {
	SemaphoreHandle_t *m = lbm_malloc_reserve(sizeof(SemaphoreHandle_t));
	*(m)                 = xSemaphoreCreateMutex();
	return (lib_mutex)m;
}

static void lib_mutex_lock(lib_mutex m) {
	xSemaphoreTake(*((SemaphoreHandle_t *)m), portMAX_DELAY);
}

static void lib_mutex_unlock(lib_mutex m) {
	xSemaphoreGive(*((SemaphoreHandle_t *)m));
}

static lib_semaphore lib_sem_create(void) {
	SemaphoreHandle_t *s = lbm_malloc_reserve(sizeof(SemaphoreHandle_t));
	*(s)                 = xSemaphoreCreateCounting(0xffff, 0);
	return (lib_semaphore)s;
}

static void lib_sem_wait(lib_semaphore s) {
	xSemaphoreTake(*((SemaphoreHandle_t *)s), portMAX_DELAY);
}

static void lib_sem_signal(lib_semaphore s) {
	xSemaphoreGive(*((SemaphoreHandle_t *)s));
}

static bool lib_sem_wait_to(lib_semaphore s, TickType_t timeout_ticks) {
	return xSemaphoreTake(*((SemaphoreHandle_t *)s), timeout_ticks) == pdPASS;
}

static void lib_sem_reset(lib_semaphore s) {
	do {
		xSemaphoreTake(*((SemaphoreHandle_t *)s), 0);
		vSemaphoreDelete(*((SemaphoreHandle_t *)s));
		*((SemaphoreHandle_t *)s) = xSemaphoreCreateCounting(0xffff, 0);
	} while (0);
}

static bool lib_add_extension(char *sym_str, extension_fptr ext) {
	if (sym_str[0] != 'e' || sym_str[1] != 'x' || sym_str[2] != 't'
		|| sym_str[3] != '-') {
		commands_printf_lisp("Error: Extensions must start with ext-");
		return false;
	}

	return lbm_add_extension(sym_str, ext);
}

static int lib_lbm_set_error_reason(char *str) {
	lbm_set_error_reason(str);
	return 1;
}

static void comm_can_transmit_sid_wrapper(
	uint32_t id, const uint8_t *data, uint8_t len
) {
	comm_can_transmit_sid(id, data, len);
}

static void comm_can_transmit_eid_wrapper(
	uint32_t id, const uint8_t *data, uint8_t len
) {
	comm_can_transmit_eid(id, data, len);
}

// High resolution timer for short busy-wait sleeps and time measurement
uint32_t lib_timer_time_now() {
	return (uint32_t)(esp_timer_get_time()); // microseconds
}

float lib_timer_seconds_elapsed_since(uint32_t time_us) {
	uint32_t now_us = lib_timer_time_now();
	return (now_us - time_us) / 1000000.0f; // wraparound-safe subtraction
}

void lib_timer_sleep(float seconds) {
	uint32_t start_us = lib_timer_time_now();
	uint32_t wait_us  = (uint32_t)(seconds * 1000000.0f);
	while ((lib_timer_time_now() - start_us) < wait_us) {
		// Busy wait
	}
}

// App data handler wrapper
bool commands_set_app_data_handler_wrapper(
	void (*func)(unsigned char *data, unsigned int len)
) {
	void (*func_irom)(unsigned char *, unsigned int) = utils_drom_to_irom(func);
	return commands_set_app_data_handler(func_irom);
}
// CAN standard ID callback wrapper
//void can_set_sid_cb_wrapper(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len)) {
//    bool (*p_func_irom)(uint32_t, uint8_t*, uint8_t) = utils_drom_to_irom(p_func);
//    can_set_sid_cb(p_func_irom);
//}

// CAN extended ID callback wrapper
//void can_set_eid_cb_wrapper(bool (*p_func)(uint32_t id, uint8_t *data, uint8_t len)) {
//    bool (*p_func_irom)(uint32_t, uint8_t*, uint8_t) = utils_drom_to_irom(p_func);
//    can_set_eid_cb(p_func_irom);
//}

// Terminal callback removal wrapper
void terminal_unregister_callback_wrapper(
	void (*cbf)(int argc, const char **argv)
) {
	void (*cbf_irom)(int, const char **) = utils_drom_to_irom(cbf);
	terminal_unregister_callback(cbf_irom);
}

// IMU read callback wrapper
void imu_set_read_callback_wrapper(
	void (*func)(float *acc, float *gyro, float *mag, float dt)
) {
	void (*func_irom)(float *, float *, float *, float) =
		utils_drom_to_irom(func);
	imu_set_read_callback(func_irom);
}

// Reply function removal wrapper
void commands_unregister_reply_func_wrapper(
	void (*reply_func)(unsigned char *data, unsigned int len)
) {
	void (*reply_func_irom)(unsigned char *, unsigned int) =
		utils_drom_to_irom(reply_func);
	commands_unregister_reply_func(reply_func_irom);
}

// Terminal command registration wrapper (1 function pointer)
void terminal_register_command_callback_wrapper(
	const char *command, const char *help, const char *arg_names,
	void (*cbf)(int argc, const char **argv)
) {
	void (*cbf_irom)(int, const char **) = utils_drom_to_irom(cbf);
	terminal_register_command_callback(command, help, arg_names, cbf_irom);
}

// Packet init wrapper (2 function pointers)
//void packet_init_wrapper(void (*s_func)(unsigned char *, unsigned int),
//         void (*p_func)(unsigned char *, unsigned int),
//         PACKET_STATE_t *state) {
//    void (*s_func_irom)(unsigned char*, unsigned int) = utils_drom_to_irom(s_func);
//    void (*p_func_irom)(unsigned char*, unsigned int) = utils_drom_to_irom(p_func);
//    packet_init(s_func_irom, p_func_irom, state);
//}

// Custom config wrapper (3 function pointers)
void conf_custom_add_config_wrapper(
	int (*get_cfg)(uint8_t *data, bool is_default),
	bool (*set_cfg)(uint8_t *data), int (*get_cfg_xml)(uint8_t **data)
) {
	int (*get_cfg_irom)(uint8_t *, bool) = utils_drom_to_irom(get_cfg);
	bool (*set_cfg_irom)(uint8_t *)      = utils_drom_to_irom(set_cfg);
	int (*get_cfg_xml_irom)(uint8_t **)  = utils_drom_to_irom(get_cfg_xml);
	conf_custom_add_config(get_cfg_irom, set_cfg_irom, get_cfg_xml_irom);
}

lbm_value ext_load_native_lib(lbm_value *args, lbm_uint argn) {
    lbm_value res = lbm_enc_sym(SYM_EERROR);

    // Expect a single numeric argument containing the IROM base address
    if (argn != 1 || !lbm_is_number(args[0])) {
        return res;
    }

	if (!lib_init_done) {
		memset((char *)cif.pad, 0, 2048);
		// LBM
		cif.cif.lbm_add_extension            = lib_add_extension;
		cif.cif.lbm_block_ctx_from_extension = lbm_block_ctx_from_extension;
		cif.cif.lbm_unblock_ctx              = lbm_unblock_ctx;
		cif.cif.lbm_get_current_cid          = lbm_get_current_cid;
		cif.cif.lbm_set_error_reason         = lib_lbm_set_error_reason;
		cif.cif.lbm_pause_eval_with_gc       = lbm_pause_eval_with_gc;
		cif.cif.lbm_continue_eval            = lbm_continue_eval;
		cif.cif.lbm_send_message             = lbm_send_message;
		cif.cif.lbm_eval_is_paused           = lib_eval_is_paused;

		cif.cif.lbm_cons                     = lbm_cons;
		cif.cif.lbm_car                      = lbm_car;
		cif.cif.lbm_cdr                      = lbm_cdr;
		cif.cif.lbm_list_destructive_reverse = lbm_list_destructive_reverse;
		cif.cif.lbm_create_byte_array        = lib_create_byte_array;

		cif.cif.lbm_add_symbol_const   = lbm_add_symbol_const;
		cif.cif.lbm_get_symbol_by_name = lbm_get_symbol_by_name;

		cif.cif.lbm_enc_i     = lbm_enc_i;
		cif.cif.lbm_enc_u     = lbm_enc_u;
		cif.cif.lbm_enc_char  = lbm_enc_char;
		cif.cif.lbm_enc_float = lbm_enc_float;
		cif.cif.lbm_enc_u32   = lbm_enc_u32;
		cif.cif.lbm_enc_i32   = lbm_enc_i32;
		cif.cif.lbm_enc_sym   = lbm_enc_sym;

		cif.cif.lbm_dec_as_float = lbm_dec_as_float;
		cif.cif.lbm_dec_as_u32   = lbm_dec_as_u32;
		cif.cif.lbm_dec_as_i32   = lbm_dec_as_i32;
		cif.cif.lbm_dec_char     = lbm_dec_char;
		cif.cif.lbm_dec_str      = lbm_dec_str;
		cif.cif.lbm_dec_sym      = lbm_dec_sym;

		cif.cif.lbm_is_byte_array = lbm_is_array_r;
		cif.cif.lbm_is_cons       = lbm_is_cons;
		cif.cif.lbm_is_number     = lbm_is_number;
		cif.cif.lbm_is_char       = lbm_is_char;
		cif.cif.lbm_is_symbol     = lbm_is_symbol;

		cif.cif.lbm_enc_sym_nil    = ENC_SYM_NIL;
		cif.cif.lbm_enc_sym_true   = ENC_SYM_TRUE;
		cif.cif.lbm_enc_sym_terror = ENC_SYM_TERROR;
		cif.cif.lbm_enc_sym_eerror = ENC_SYM_EERROR;
		cif.cif.lbm_enc_sym_merror = ENC_SYM_MERROR;

		cif.cif.lbm_is_symbol_nil  = lbm_is_symbol_nil;
		cif.cif.lbm_is_symbol_true = lbm_is_symbol_true;

		// Os
		cif.cif.sleep_ms          = lib_sleep_ms;
		cif.cif.sleep_us          = lib_sleep_us;
		cif.cif.system_time       = lib_system_time;
		cif.cif.ts_to_age_s       = lib_ts_to_age_s;
		cif.cif.printf            = commands_printf_lisp;
		cif.cif.malloc            = lbm_malloc_reserve;
		cif.cif.free              = lbm_free;
		cif.cif.spawn             = lispif_spawn;
		cif.cif.request_terminate = lib_request_terminate;
		cif.cif.should_terminate  = lib_should_terminate;
		cif.cif.get_arg           = lib_get_arg;

		// ST IO
		//cif.cif.set_pad_mode = lib_set_pad_mode;
		//cif.cif.set_pad = lib_set_pad;
		//cif.cif.clear_pad = lib_clear_pad;

		// Abstract IO
		//cif.cif.io_set_mode = lib_io_set_mode;
		//cif.cif.io_write = lib_io_write;
		//cif.cif.io_read = lib_io_read;
		//cif.cif.io_read_analog = lib_io_read_analog;
		//cif.cif.io_get_st_pin = lib_io_get_st_pin;

		// CAN
		//cif.cif.can_set_sid_cb = comm_can_set_sid_rx_callback;
		//cif.cif.can_set_eid_cb = comm_can_set_eid_rx_callback;
		cif.cif.can_transmit_sid          = comm_can_transmit_sid_wrapper;
		cif.cif.can_transmit_eid          = comm_can_transmit_eid_wrapper;
		cif.cif.can_send_buffer           = comm_can_send_buffer;
		cif.cif.can_set_duty              = comm_can_set_duty;
		cif.cif.can_set_current           = comm_can_set_current;
		cif.cif.can_set_current_off_delay = comm_can_set_current_off_delay;
		cif.cif.can_set_current_brake     = comm_can_set_current_brake;
		cif.cif.can_set_rpm               = comm_can_set_rpm;
		cif.cif.can_set_pos               = comm_can_set_pos;
		cif.cif.can_set_current_rel       = comm_can_set_current_rel;
		cif.cif.can_set_current_rel_off_delay =
			comm_can_set_current_rel_off_delay;
		cif.cif.can_set_current_brake_rel  = comm_can_set_current_brake_rel;
		cif.cif.can_ping                   = comm_can_ping;
		cif.cif.can_get_status_msg_index   = comm_can_get_status_msg_index;
		cif.cif.can_get_status_msg_id      = comm_can_get_status_msg_id;
		cif.cif.can_get_status_msg_2_index = comm_can_get_status_msg_2_index;
		cif.cif.can_get_status_msg_2_id    = comm_can_get_status_msg_2_id;
		cif.cif.can_get_status_msg_3_index = comm_can_get_status_msg_3_index;
		cif.cif.can_get_status_msg_3_id    = comm_can_get_status_msg_3_id;
		cif.cif.can_get_status_msg_4_index = comm_can_get_status_msg_4_index;
		cif.cif.can_get_status_msg_4_id    = comm_can_get_status_msg_4_id;
		cif.cif.can_get_status_msg_5_index = comm_can_get_status_msg_5_index;
		cif.cif.can_get_status_msg_5_id    = comm_can_get_status_msg_5_id;
		cif.cif.can_get_status_msg_6_index = comm_can_get_status_msg_6_index;
		cif.cif.can_get_status_msg_6_id    = comm_can_get_status_msg_6_id;

		// Comm
		cif.cif.commands_process_packet = commands_process_packet;
		cif.cif.send_app_data           = commands_send_app_data;
		cif.cif.set_app_data_handler    = commands_set_app_data_handler_wrapper;

		// UART will need to implement this
		//cif.cif.uart_start = lib_uart_start;
		//cif.cif.uart_write = lib_uart_write;
		//cif.cif.uart_read = lib_uart_write;

		// Packets
		cif.cif.packet_init         = packet_init;
		cif.cif.packet_reset        = packet_reset;
		cif.cif.packet_process_byte = packet_process_byte;
		cif.cif.packet_send_packet  = packet_send_packet;

		// IMU
		cif.cif.imu_startup_done        = imu_startup_done;
		cif.cif.imu_get_roll            = imu_get_roll;
		cif.cif.imu_get_pitch           = imu_get_pitch;
		cif.cif.imu_get_yaw             = imu_get_yaw;
		cif.cif.imu_get_rpy             = imu_get_rpy;
		cif.cif.imu_get_accel           = imu_get_accel;
		cif.cif.imu_get_gyro            = imu_get_gyro;
		cif.cif.imu_get_mag             = imu_get_mag;
		cif.cif.imu_derotate            = imu_derotate;
		cif.cif.imu_get_accel_derotated = imu_get_accel_derotated;
		cif.cif.imu_get_gyro_derotated  = imu_get_gyro_derotated;
		cif.cif.imu_get_quaternions     = imu_get_quaternions;
		cif.cif.imu_get_calibration     = imu_get_calibration;

		// EEPROM
		cif.cif.read_eeprom_var  = read_eeprom_var;
		cif.cif.store_eeprom_var = store_eeprom_var;

		// Terminal
		cif.cif.terminal_register_command_callback =
			terminal_register_command_callback_wrapper;
		cif.cif.terminal_unregister_callback =
			terminal_unregister_callback_wrapper;

		// Plot
		cif.cif.plot_init        = commands_init_plot;
		cif.cif.plot_add_graph   = commands_plot_add_graph;
		cif.cif.plot_set_graph   = commands_plot_set_graph;
		cif.cif.plot_send_points = commands_send_plot_points;

		// Custom config
		cif.cif.conf_custom_add_config    = conf_custom_add_config_wrapper;
		cif.cif.conf_custom_clear_configs = conf_custom_clear_configs;

		// Settings (TODO: Add more types)
		//cif.cif.get_cfg_float = get_cfg_float;
		//cif.cif.get_cfg_int = get_cfg_int;
		//cif.cif.set_cfg_float = set_cfg_float;
		//cif.cif.set_cfg_int = set_cfg_int;
		//cif.cif.store_cfg = store_cfg;
		//cif.cif.get_cfg = get_cfg;

		// Mutex
		cif.cif.mutex_create = lib_mutex_create;
		cif.cif.mutex_lock   = lib_mutex_lock;
		cif.cif.mutex_unlock = lib_mutex_unlock;

		// Get io-pin from lbm symbol (this is only safe from extensions)
		//cif.cif.lbm_symbol_to_io = lbm_symbol_to_io;

		// High resolution timer for short busy-wait sleeps and time measurement TODO
		cif.cif.timer_time_now              = lib_timer_time_now;
		cif.cif.timer_seconds_elapsed_since = lib_timer_seconds_elapsed_since;
		cif.cif.timer_sleep                 = lib_timer_sleep;

		// System lock (with counting)
		cif.cif.sys_lock   = utils_sys_lock_cnt;
		cif.cif.sys_unlock = utils_sys_unlock_cnt;

		cif.cif.commands_unregister_reply_func =
			commands_unregister_reply_func_wrapper;

		// IMU AHRS functions and read callback
		cif.cif.imu_set_read_callback   = imu_set_read_callback_wrapper;
		cif.cif.ahrs_init_attitude_info = ahrs_init_attitude_info;
		cif.cif.ahrs_update_initial_orientation =
			ahrs_update_initial_orientation;
		cif.cif.ahrs_update_mahony_imu   = ahrs_update_mahony_imu;
		cif.cif.ahrs_update_madgwick_imu = ahrs_update_madgwick_imu;
		cif.cif.ahrs_get_roll            = ahrs_get_roll;
		cif.cif.ahrs_get_pitch           = ahrs_get_pitch;
		cif.cif.ahrs_get_yaw             = ahrs_get_yaw;

		// Store backup data
		cif.cif.store_backup_data = main_store_backup_data;

		// Flat values
		cif.cif.lbm_start_flatten  = lbm_start_flatten;
		cif.cif.lbm_finish_flatten = lbm_finish_flatten;
		cif.cif.f_b                = f_b;
		cif.cif.f_cons             = f_cons;
		cif.cif.f_float            = f_float;
		cif.cif.f_i                = f_i;
		cif.cif.f_i32              = f_i32;
		cif.cif.f_i64              = f_i64;
		cif.cif.f_lbm_array        = f_lbm_array;
		cif.cif.f_sym              = f_sym;
		cif.cif.f_u32              = f_u32;
		cif.cif.f_u64              = f_u64;

		// Unblock unboxed
		cif.cif.lbm_unblock_ctx_unboxed = lbm_unblock_ctx_unboxed;

		// System time
		cif.cif.system_time_ticks = xTaskGetTickCount;
		cif.cif.sleep_ticks       = vTaskDelay;

		// Semaphores
		cif.cif.sem_create  = lib_sem_create;
		cif.cif.sem_wait    = lib_sem_wait;
		cif.cif.sem_signal  = lib_sem_signal;
		cif.cif.sem_wait_to = lib_sem_wait_to;
		cif.cif.sem_reset   = lib_sem_reset;

		cif.cif.rgbled_init   = rgbled_init;
		cif.cif.rgbled_deinit = rgbled_deinit;
		cif.cif.rgbled_update = rgbled_update;
		cif.cif.color_add_sub = color_add_sub;
		cif.cif.color_scale   = color_scale;
		cif.cif.color_mix     = color_mix;

		lib_init_done = true;
	}

    // Read IROM header base directly
    uint32_t irom_base = lbm_dec_as_u32(args[0]);
    commands_printf_lisp("=== LOAD ATTEMPT: irom=0x%08X ===", irom_base);

    // Basic pointer/alignment sanity
    if (irom_base == 0 || (irom_base & 0x3) != 0) {
        lbm_set_error_reason("Invalid IROM base pointer");
        return res;
    }

    // Validate native header magic
    uint32_t magic_be = 0;
    memcpy(&magic_be, (const void *)irom_base, sizeof(magic_be));
    if (magic_be != __builtin_bswap32(NATIVE_LIB_MAGIC)) {
        lbm_set_error_reason("Magic number not found at IROM address");
        return res;
    }

    // Duplicate check by IROM base
    for (int i = 0; i < LIB_NUM_MAX; i++) {
        if (loaded_libs[i].stop_fun != NULL && loaded_libs[i].base_addr == irom_base) {
            lbm_set_error_reason("Library already loaded");
            return res;
        }
    }

    bool ok = false;
    for (int i = 0; i < LIB_NUM_MAX; i++) {
        if (loaded_libs[i].stop_fun == NULL) {
            loaded_libs[i].base_addr = irom_base;

            // Entry is after header: magic(4) + prog_addr(4) = 8 bytes
            uint32_t func_addr = irom_base + 8;
            if (func_addr & 0x3) {
                lbm_set_error_reason("IROM function address not aligned");
                return res;
            }

            commands_printf_lisp("Calling init function at 0x%08X", func_addr);
            ok = ((bool (*)(lib_info *))func_addr)(&loaded_libs[i]);

            if (loaded_libs[i].stop_fun != NULL) {
                void *stop_fun_irom = utils_drom_to_irom(loaded_libs[i].stop_fun);
                if (!utils_is_func_valid(stop_fun_irom)) {
                    loaded_libs[i].stop_fun = NULL;
                    lbm_set_error_reason("Invalid stop function. Must be static.");
                    return res;
                }
                loaded_libs[i].stop_fun = stop_fun_irom;
                commands_printf_lisp("Library init successful");
            } else {
                commands_printf_lisp("Library init failed - no stop function set");
            }
            break;
        }
    }

    if (ok) {
        commands_printf_lisp("=== LOAD SUCCESS ===");
        res = lbm_enc_sym(SYM_TRUE);
    } else {
        lbm_set_error_reason("Library init failed");
    }

    return res;
}

lbm_value ext_unload_native_lib(lbm_value *args, lbm_uint argn) {
    lbm_value res = lbm_enc_sym(SYM_EERROR);

    if (argn != 1 || !lbm_is_number(args[0])) {
        return res;
    }

    uint32_t irom_base = lbm_dec_as_u32(args[0]);

    for (int i = 0; i < LIB_NUM_MAX; i++) {
        if (loaded_libs[i].stop_fun != NULL && loaded_libs[i].base_addr == irom_base) {
            if (utils_is_func_valid(loaded_libs[i].stop_fun)) {
                loaded_libs[i].stop_fun(loaded_libs[i].arg);
            }
            loaded_libs[i].stop_fun  = NULL;
            loaded_libs[i].base_addr = 0;
            loaded_libs[i].arg       = NULL;

            commands_printf_lisp("Library at 0x%08X unloaded", irom_base);
            return lbm_enc_sym(SYM_TRUE);
        }
    }

    lbm_set_error_reason("Library not loaded");
    return res;
}