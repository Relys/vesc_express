#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "soc/gpio_struct.h"

#include "disp_gc9a01a.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

static int display_width = 240;
static int display_height = 240;

// Private variables
static int m_pin_reset = -1;
static int m_pin_dc = -1;

#define DISP_REG_SET        GPIO.out_w1ts.val
#define DISP_REG_CLR        GPIO.out_w1tc.val

#define COMMAND()           (DISP_REG_CLR = 1 << m_pin_dc)
#define DATA()              (DISP_REG_SET = 1 << m_pin_dc)

static void command_start(uint8_t cmd) {
    COMMAND();
    hwspi_send_data(&cmd, 1);
    DATA();
}

static uint16_t to_disp_color(uint32_t rgb) {
    uint8_t b = (uint8_t)rgb;
    uint8_t g = (uint8_t)(rgb >> 8);
    uint8_t r = (uint8_t)(rgb >> 16);
    r >>= 3;
    g >>= 2;
    b >>= 3;

    uint8_t color_high = (r << 3) | (g >> 3);
    uint8_t color_low = (g << 5) | b;

    uint16_t color = (color_high << 8) | color_low;
    return color;
}

// Add support for indexed2 format
static void blast_indexed2(image_buffer_t *img, color_t *colors) {
    command_start(0x2C);
    hwspi_data_stream_start();

    uint8_t *data = img->data;
    int num_pix = img->width * img->height;

    for (int i = 0; i < num_pix; i++) {
        int byte = i >> 3;
        int bit = 7 - (i & 0x7);
        int color_ind = (data[byte] & (1 << bit)) >> bit;

        uint16_t c = to_disp_color(COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width));
        hwspi_data_stream_write((uint8_t)c);
        hwspi_data_stream_write((uint8_t)(c >> 8));
    }

    hwspi_data_stream_finish();
}

// Add support for indexed4 format
static void blast_indexed4(image_buffer_t *img, color_t *colors) {
    command_start(0x2C);
    hwspi_data_stream_start();

    uint8_t *data = img->data;
    int num_pix = img->width * img->height;

    for (int i = 0; i < num_pix; i++) {
        int byte = i >> 2;
        int bit = (3 - (i & 0x03)) * 2;
        int color_ind = (data[byte] & (0x03 << bit)) >> bit;

        uint16_t c = to_disp_color(COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width));
        hwspi_data_stream_write((uint8_t)c);
        hwspi_data_stream_write((uint8_t)(c >> 8));
    }

    hwspi_data_stream_finish();
}

// Add support for indexed16 format
static void blast_indexed16(image_buffer_t *img, color_t *colors) {
    command_start(0x2C);
    hwspi_data_stream_start();

    uint8_t *data = img->data;
    int num_pix = img->width * img->height;

    for (int i = 0; i < num_pix; i++) {
        int byte = i >> 1;
        int bit = (1 - (i & 0x01)) * 4;
        int color_ind = (data[byte] & (0x0F << bit)) >> bit;

        uint16_t c = to_disp_color(COLOR_TO_RGB888(colors[color_ind], i % img->width, i / img->width));
        hwspi_data_stream_write((uint8_t)c);
        hwspi_data_stream_write((uint8_t)(c >> 8));
    }

    hwspi_data_stream_finish();
}

// Add support for rgb332 format
static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
    command_start(0x2C);
    hwspi_data_stream_start();

    for (int i = 0; i < num_pix; i++) {
        uint8_t pix = data[i];
        uint32_t r = (pix >> 5) & 0x07;
        uint32_t g = (pix >> 2) & 0x07;
        uint32_t b = pix & 0x03;

        uint32_t rgb888 = (r << (16 + 5)) | (g << (8 + 5)) | (b << 6);
        uint16_t disp = to_disp_color(rgb888);

        hwspi_data_stream_write((uint8_t)disp);
        hwspi_data_stream_write((uint8_t)(disp >> 8));
    }

    hwspi_data_stream_finish();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
    command_start(0x2C);  // Memory Write command
    hwspi_data_stream_start();

    for (int i = 0; i < num_pix; i++) {
        uint16_t pix = (((uint16_t)data[2 * i]) << 8) | ((uint16_t)data[2 * i + 1]);

        uint32_t r = (uint32_t)(pix >> 11);
        uint32_t g = (uint32_t)((pix >> 5) & 0x3F);
        uint32_t b = (uint32_t)(pix & 0x1F);
        uint32_t rgb888 = r << (16 + 3) | g << (8 + 2) | b << 3;
        uint16_t disp = to_disp_color(rgb888);

        hwspi_data_stream_write((uint8_t)disp);
        hwspi_data_stream_write((uint8_t)(disp >> 8));
    }

    hwspi_data_stream_finish();
}

static void blast_rgb888(uint8_t *data, uint32_t num_pix) {
    command_start(0x2C);  // Memory Write command
    hwspi_data_stream_start();

    for (int i = 0; i < num_pix; i++) {
        uint32_t r = data[3 * i];
        uint32_t g = data[3 * i + 1];
        uint32_t b = data[3 * i + 2];

        uint32_t rgb888 = (r << 16) | (g << 8) | b;
        uint16_t disp = to_disp_color(rgb888);

        hwspi_data_stream_write((uint8_t)disp);
        hwspi_data_stream_write((uint8_t)(disp >> 8));
    }

    hwspi_data_stream_finish();
}

bool disp_gc9a01a_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
    uint16_t cs = x;
    uint16_t ce = x + img->width - 1;
    uint16_t ps = y;
    uint16_t pe = y + img->height - 1;

    if (ce >= display_width || pe >= display_height) {
        return false;
    }

    uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
    uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

    disp_gc9a01a_command(0x2A, col, 4);  // Column address set
    disp_gc9a01a_command(0x2B, row, 4);  // Page address set

    uint32_t num_pix = img->width * img->height;

    hwspi_begin();
    switch (img->fmt) {
        case indexed2:
            if (!colors) return false;
            blast_indexed2(img, colors);
            break;
        case indexed4:
            if (!colors) return false;
            blast_indexed4(img, colors);
            break;
        case indexed16:
            if (!colors) return false;
            blast_indexed16(img, colors);
            break;
        case rgb332:
            blast_rgb332(img->data, num_pix);
            break;
        case rgb565:
            blast_rgb565(img->data, num_pix);
            break;
        case rgb888:
            blast_rgb888(img->data, num_pix);
            break;
        default:
            return false;
    }
    hwspi_end();

    return true;
}

void disp_gc9a01a_clear(uint32_t color) {
    uint16_t clear_color_disp = to_disp_color(color);

    uint16_t cs = 0;
    uint16_t ce = display_width - 1;
    uint16_t ps = 0;
    uint16_t pe = display_height - 1;

    uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
    uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

    disp_gc9a01a_command(0x2A, col, 4);  // Column address set
    disp_gc9a01a_command(0x2B, row, 4);  // Page address set

    hwspi_begin();
    command_start(0x2C);  // Memory Write command
    hwspi_data_stream_start();
    for (int i = 0; i < (display_width * display_height); i++) {
        hwspi_data_stream_write((uint8_t)(clear_color_disp));
        hwspi_data_stream_write((uint8_t)(clear_color_disp >> 8));
    }
    hwspi_data_stream_finish();
    hwspi_end();
}

static lbm_value ext_disp_cmd(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_NUMBER_ALL();

    lbm_value res = ENC_SYM_TERROR;

    if (argn > 1) {
        uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
        uint8_t paras[12];
        for (int i = 0; i < argn - 1; i++) {
            paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
        }

        disp_gc9a01a_command(cmd, paras, argn - 1);
        res = ENC_SYM_TRUE;
    } else if (argn == 1) {
        uint8_t cmd = (uint8_t)lbm_dec_as_u32(args[0]);
        disp_gc9a01a_command(cmd, 0, 0);
        res = ENC_SYM_TRUE;
    }

    return res;
}

static lbm_value ext_disp_orientation(lbm_value *args, lbm_uint argn) {
    LBM_CHECK_ARGN_NUMBER(1);

    uint32_t orientation = lbm_dec_as_u32(args[0]);
    uint8_t arg = 0;
    lbm_value res = ENC_SYM_TRUE;
    switch (orientation) {
        case 0:
            arg = 0x08;
            disp_gc9a01a_command(0x36, &arg, 1);
            display_width = 240;
            display_height = 240;
            break;
        case 1:
            arg = 0x60;
            disp_gc9a01a_command(0x36, &arg, 1);
            display_width = 240;
            display_height = 240;
            break;
        case 2:
            arg = 0xC0;
            disp_gc9a01a_command(0x36, &arg, 1);
            display_width = 240;
            display_height = 240;
            break;
        case 3:
            arg = 0xA0;
            disp_gc9a01a_command(0x36, &arg, 1);
            display_width = 240;
            display_height = 240;
            break;
        default:
            res = ENC_SYM_EERROR;
            break;
    }
    return res;
}

void disp_gc9a01a_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz) {
    hwspi_init(clock_mhz, 0, -1, pin_sd0, pin_clk, pin_cs);
    m_pin_reset = pin_reset;
    m_pin_dc = pin_dc;

    gpio_config_t gpconf = {0};
    gpconf.pin_bit_mask = BIT(m_pin_reset) | BIT(m_pin_dc);
    gpconf.mode = GPIO_MODE_OUTPUT;
    gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpconf.intr_type = GPIO_INTR_DISABLE;

    gpio_config(&gpconf);

    gpio_set_level(m_pin_reset, 1);
    gpio_set_level(m_pin_dc, 0);

    // Reset display
    gpio_set_level(m_pin_reset, 0);
    vTaskDelay(5);
    gpio_set_level(m_pin_reset, 1);
    vTaskDelay(120);

    // Initialize GC9A01A
    for (int i = 0; i < sizeof(gc9a01a_init_sequence) / sizeof(gc9a01a_init_sequence[0]); i++) {
        int argn = gc9a01a_init_sequence[i][0] - 1;
        const uint8_t *args = &gc9a01a_init_sequence[i][2];
        uint8_t cmd = gc9a01a_init_sequence[i][1];
        disp_gc9a01a_command(cmd, args, argn);
    }

    // Exit sleep mode and turn on the display
    disp_gc9a01a_command(0x11, NULL, 0);  // Sleep OUT
    vTaskDelay(100);
    disp_gc9a01a_command(0x29, NULL, 0);  // Display ON
    vTaskDelay(100);

    // Set default display parameters
    display_width = 240;
    display_height = 240;

    // Clear the display
    disp_gc9a01a_clear(0);

    // Register LispBM extensions for display commands and orientation
    lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
    lbm_add_extension("ext-disp-orientation", ext_disp_orientation);
}

void disp_gc9a01a_command(uint8_t command, const uint8_t *args, int argn) {
    hwspi_begin();
    command_start(command);
    if (args != NULL && argn > 0) {
        hwspi_send_data(args, argn);
    }
    hwspi_end();
}