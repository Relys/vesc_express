/*
	Copyright 2023 Benjamin Vedder	benjamin@vedder.se
	Copyright 2023 Joel Svensson    svenssonjoel@yahoo.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "disp_icna3306.h"
#include "hwspi.h"
#include "lispif.h"
#include "lispbm.h"

#define DISPLAY_WIDTH		194
#define DISPLAY_HEIGHT		368

// Private variables
static int m_pin_reset = -1;

static void command_start(uint8_t command) {
	uint8_t cmd[4];
	cmd[0] = 0x02;
	cmd[1] = 0;
	cmd[2] = command;
	cmd[3] = 0;
	hwspi_send_data(cmd, 4);
}

static inline uint16_t to_disp_color(uint32_t rgb) {
	uint8_t b = (uint8_t)rgb;
	uint8_t g = (uint8_t)(rgb >> 8);
	uint8_t r = (uint8_t)(rgb >> 16);
	r >>= 3;
	g >>= 2;
	b >>= 3;

	uint8_t color_high = 0;
	color_high = r << 3;
	color_high |= (g >> 3);

	uint8_t color_low = 0;
	color_low = g << 5;
	color_low |= b;

	// the order of output is bit 7 - 0 : 15 - 8
	uint16_t color = color_high;
	color |= (((uint16_t)color_low) << 8);
	return color;
}

static void blast_indexed2(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 3;
		int bit  = 7 - (i & 0x7);
		int color_ind = (data[byte] & (1 << bit)) >> bit;

		uint16_t c = to_disp_color(
						COLOR_TO_RGB888(colors[color_ind],
								i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed4(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 2;
		int bit = (3 - (i & 0x03)) * 2;
		int color_ind = (data[byte] & (0x03 << bit)) >> bit;

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_indexed16(image_buffer_t *img, color_t *colors) {
	command_start(0x2C);
	hwspi_data_stream_start();

	uint8_t *data = img->data;
	int num_pix = img->width * img->height;

	for (int i = 0; i < num_pix; i ++) {
		int byte = i >> 1;    // byte to access is pix / 2
		int bit = (1 - (i & 0x01)) * 4; // bit position to access within byte
		int color_ind = (data[byte] & (0x0F << bit)) >> bit; // extract 4 bit value.

		uint16_t c = to_disp_color(
				COLOR_TO_RGB888(colors[color_ind],
						i % img->width, i / img->width));
		hwspi_data_stream_write((uint8_t)c);
		hwspi_data_stream_write((uint8_t)(c >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb332(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint8_t pix = data[i];
		uint32_t r = (uint32_t)((pix >> 5) & 0x7);
		uint32_t g = (uint32_t)((pix >> 2) & 0x7);
		uint32_t b = (uint32_t)(pix & 0x3);
		uint32_t rgb888 = r << (16 + 5) | g << (8 + 5) | b << 6;
		uint16_t disp = to_disp_color(rgb888);
		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

static void blast_rgb565(uint8_t *data, uint32_t num_pix) {
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
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
	command_start(0x2C);
	hwspi_data_stream_start();

	for (int i = 0; i < num_pix; i ++) {
		uint32_t r = data[3 * i];
		uint32_t g = data[3 * i + 1];
		uint32_t b = data[3 * i + 2];

		uint32_t rgb888 = r << 16 | g << 8 | b;
		uint16_t disp = to_disp_color(rgb888);

		hwspi_data_stream_write((uint8_t)disp);
		hwspi_data_stream_write((uint8_t)(disp >> 8));
	}

	hwspi_data_stream_finish();
}

bool disp_icna3306_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors) {
	uint16_t cs = x;
	uint16_t ce = x + img->width - 1;
	uint16_t ps = y;
	uint16_t pe = y + img->height - 1;

	if (ce >= DISPLAY_WIDTH || pe >= DISPLAY_HEIGHT) {
		return false;
	}

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_icna3306_command(0x2A, col, 4);
	disp_icna3306_command(0x2B, row, 4);

	uint32_t num_pix = img->width * img->height;

	hwspi_begin();
	switch(img->fmt) {
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
		blast_rgb332(img->data + img->data_offset, num_pix);
		break;
	case rgb565:
		blast_rgb565(img->data + img->data_offset, num_pix);
		break;
	case rgb888:
		blast_rgb888(img->data + img->data_offset, num_pix);
		break;
	default:
		break;
	}
	hwspi_end();

	return true;
}

void disp_icna3306_clear(uint32_t color) {
	uint16_t clear_color_disp = to_disp_color(color);

	uint16_t cs = 0;
	uint16_t ce = DISPLAY_WIDTH - 1;
	uint16_t ps = 0;
	uint16_t pe = DISPLAY_HEIGHT - 1;

	uint8_t col[4] = {cs >> 8, cs, ce >> 8, ce};
	uint8_t row[4] = {ps >> 8, ps, pe >> 8, pe};

	disp_icna3306_command(0x2A, col, 4);
	disp_icna3306_command(0x2B, row, 4);

	hwspi_begin();
	command_start(0x2C);
	hwspi_data_stream_start();
	for (int i = 0; i < (DISPLAY_WIDTH * DISPLAY_HEIGHT); i ++) {
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
		for (int i = 0; i < argn - 1; i ++) {
			paras[i] = (uint8_t)lbm_dec_as_u32(args[i + 1]);
		}

		disp_icna3306_command(cmd, paras, argn - 1);

		res = ENC_SYM_TRUE;
	} else if (argn == 1) {
		uint8_t cmd = (uint8_t) lbm_dec_as_u32(args[0]);
		disp_icna3306_command(cmd, 0, 0);
		res = ENC_SYM_TRUE;
	}

	return res;
}

static lbm_value ext_disp_set_spi_mode(lbm_value *args, lbm_uint argn) {
	(void)args; (void)argn;
	uint8_t cmd[1] = {0xFF};
	hwspi_begin();
	hwspi_send_data(cmd, 1);
	hwspi_end();
	return ENC_SYM_TRUE;
}

void disp_icna3306_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int clock_mhz) {
	hwspi_init(clock_mhz, 0, -1, pin_sd0, pin_clk, pin_cs);
	m_pin_reset = pin_reset;

	gpio_config_t gpconf = {0};
	gpconf.pin_bit_mask = BIT(m_pin_reset);
	gpconf.mode = GPIO_MODE_OUTPUT;
	gpconf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	gpconf.pull_up_en = GPIO_PULLUP_DISABLE;
	gpconf.intr_type =  GPIO_INTR_DISABLE;

	gpio_config(&gpconf);

	lbm_add_extension("ext-disp-cmd", ext_disp_cmd);
	lbm_add_extension("ext-disp-set-spi-mode", ext_disp_set_spi_mode);
}

void disp_icna3306_command(uint8_t command, uint8_t *args, int argn) {
	hwspi_begin();
	command_start(command);
	if (args != NULL && argn > 0) {
		hwspi_send_data(args, argn);
	}
	hwspi_end();
}

static const uint8_t init_sequence[][2] = {
		// ===  CMD2 password  ===
		{0xFE, 0x20},
		{0xF4, 0x5A},
		{0xF5, 0x59},

		// ===  ID code  ===
		{0xFE, 0x40},
		{0xD8, 0x33},
		{0xD9, 0x06},
		{0xDA, 0x00},

		// ===  QSPI setting  ===
		{0xFE, 0x20},
		{0x1A, 0x15},
		{0x19, 0x10},
		{0x1C, 0xA0},

		// ===  Timing Gen  ===
		{0xFE, 0x40},
		{0x01, 0x90},
		{0x02, 0x5C},
		{0x59, 0x01},
		{0x5A, 0x58},
		{0x5B, 0x08},
		{0x5C, 0x08},
		{0x70, 0x01},
		{0x71, 0x58},
		{0x72, 0x08},
		{0x73, 0x08},

		// ===  AOD setting===
		{0xFE, 0x40},
		{0x5D, 0x24},
		{0x60, 0x08},
		{0x61, 0x04},
		{0x62, 0x7F},
		{0x69, 0x06},
		{0x0C, 0xD7},
		{0x0D, 0xFC},
		{0x39, 0x24},
		{0x3D, 0x08},
		{0x47, 0x06},
		{0x6D, 0x04},
		{0x10, 0x11},
		{0x11, 0x09},

		// ===  Power Settings  ===
		{0xFE, 0xE0},
		{0x00, 0x14},
		{0x01, 0x01},
		{0x02, 0x00},
		{0x04, 0x04},
		{0x06, 0x0F},
		{0x08, 0x00},
		{0x09, 0x14},
		{0x0A, 0x01},
		{0x0B, 0x00},
		{0x0C, 0x04},
		{0x0E, 0x0F},
		{0x0F, 0x00},
		{0x10, 0x14},
		{0x11, 0x10},
		{0x24, 0x00},
		{0x21, 0x99},
		{0x2D, 0x99},
		{0x32, 0x99},
		{0x26, 0x41},
		{0x22, 0x1A},
		{0x23, 0x13},
		{0x30, 0x01},

		{0xFE, 0x40},
		{0x57, 0x43},
		{0x58, 0x33},
		{0x6E, 0x43},
		{0x6F, 0x33},
		{0x74, 0x43},
		{0x75, 0x33},

		// === GOA mapping ===
		{0xFE, 0x70},
		{0x9B, 0x02},
		{0x9C, 0x03},
		{0x9D, 0x08},
		{0x9E, 0x19},
		{0x9F, 0x19},
		{0xA0, 0x19},
		{0xA2, 0x19},
		{0xA3, 0x19},
		{0xA4, 0x19},
		{0xA5, 0x19},
		{0xA6, 0x11},
		{0xA7, 0x10},
		{0xA9, 0x0F},
		{0xAA, 0x19},
		{0xAB, 0x19},
		{0xAC, 0x19},
		{0xAD, 0x19},
		{0xAE, 0x19},
		{0xAF, 0x19},
		{0xB0, 0x19},
		{0xB1, 0x19},
		{0xB2, 0x19},
		{0xB3, 0x19},
		{0xB4, 0x19},
		{0xB5, 0x19},
		{0xB6, 0x19},
		{0xB7, 0x19},
		{0xB8, 0x00},
		{0xB9, 0x01},
		{0xBA, 0x09},
		{0xBB, 0x19},
		{0xBC, 0x19},
		{0xBD, 0xF9},
		{0xBE, 0x19},
		{0xBF, 0x19},
		{0xC0, 0x0E},
		{0xC1, 0x0D},
		{0xC2, 0x0C},
		{0xC3, 0x19},
		{0xC4, 0x19},
		{0xC5, 0x19},
		{0xC6, 0x19},
		{0xC7, 0x19},
		{0xC8, 0x19},

		// ===  source/mux sequence ===
		{0xFE, 0x40},
		{0x4C, 0x22},
		{0x53, 0xA0},
		{0x08, 0x0A},

		// ===  SD/SW_Toggle_Sequence_Control ===
		{0xFE, 0xF0},
		{0x72, 0x33},
		{0x73, 0x66},
		{0x74, 0x22},
		{0x75, 0x55},
		{0x76, 0x11},
		{0x77, 0x44},
		{0x78, 0x33},
		{0x79, 0x66},
		{0x7A, 0x22},
		{0x7B, 0x55},
		{0x7C, 0x11},
		{0x7D, 0x44},
		{0x7E, 0x66},
		{0x7F, 0x33},
		{0x80, 0x55},
		{0x81, 0x22},
		{0x82, 0x44},
		{0x83, 0x11},
		{0x84, 0x66},
		{0x85, 0x33},
		{0x86, 0x55},
		{0x87, 0x22},
		{0x88, 0x44},
		{0x89, 0x11},

		// === GIP Setting  ===
		{0xFE, 0x70},
		{0x00, 0xC0},
		{0x01, 0x08},
		{0x02, 0x02},
		{0x03, 0x00},
		{0x04, 0x00},
		{0x05, 0x01},
		{0x06, 0x28},
		{0x07, 0x28},
		{0x09, 0xC0},
		{0x0A, 0x08},
		{0x0B, 0x02},
		{0x0C, 0x00},
		{0x0D, 0x00},
		{0x0E, 0x00},
		{0x0F, 0x28},
		{0x10, 0x28},
		{0x12, 0xC0},
		{0x13, 0x08},
		{0x14, 0x02},
		{0x15, 0x00},
		{0x16, 0x00},
		{0x17, 0x01},
		{0x18, 0xD8},
		{0x19, 0x18},
		{0x1B, 0xC0},
		{0x1C, 0x08},
		{0x1D, 0x02},
		{0x1E, 0x00},
		{0x1F, 0x00},
		{0x20, 0x00},
		{0x21, 0xD8},
		{0x22, 0x18},
		{0x4C, 0x80},
		{0x4D, 0x00},
		{0x4E, 0x01},
		{0x4F, 0x00},
		{0x50, 0x01},
		{0x51, 0x01},
		{0x52, 0x01},
		{0x53, 0xC6},
		{0x54, 0x00},
		{0x55, 0x03},
		{0x56, 0x28},
		{0x58, 0x28},
		{0x65, 0x80},
		{0x66, 0x05},
		{0x67, 0x10},

		// === MUX Sequence Control ===
		{0xFE, 0xF0},
		{0xA3, 0x00},

		{0xFE, 0x70},
		{0x76, 0x00},
		{0x77, 0x00},
		{0x78, 0x05},
		{0x68, 0x08},
		{0x69, 0x08},
		{0x6A, 0x10},
		{0x6B, 0x08},
		{0x6C, 0x08},
		{0x6D, 0x08},

		{0xFE, 0xF0},
		{0xA9, 0x18},
		{0xAA, 0x18},
		{0xAB, 0x18},
		{0xAC, 0x18},
		{0xAD, 0x18},
		{0xAE, 0x18},

		{0xFE, 0x70},
		{0x93, 0x00},
		{0x94, 0x00},
		{0x96, 0x05},
		{0xDB, 0x08},
		{0xDC, 0x08},
		{0xDD, 0x10},
		{0xDE, 0x08},
		{0xDF, 0x08},
		{0xE0, 0x08},
		{0xE7, 0x18},
		{0xE8, 0x18},
		{0xE9, 0x18},
		{0xEA, 0x18},
		{0xEB, 0x18},
		{0xEC, 0x18},

		// ===  Power on/off sequence Blank period control  ===
		{0xFE, 0x70},
		{0xD1, 0xF0},
		{0xD2, 0xFF},
		{0xD3, 0xF0},
		{0xD4, 0xFF},
		{0xD5, 0xA0},
		{0xD6, 0xAA},
		{0xD7, 0xF0},
		{0xD8, 0xFF},

		// ===  Source  ===
		{0xFE, 0x40},
		{0x4D, 0xAA},
		{0x4E, 0x00},
		{0x4F, 0xA0},
		{0x50, 0x00},
		{0x51, 0xF3},
		{0x52, 0x23},
		{0x6B, 0xF3},
		{0x6C, 0x13},
		{0x8F, 0xFF},
		{0x90, 0xFF},
		{0x91, 0x3F},
		{0xA2, 0x10},
		{0x07, 0x21},
		{0x35, 0x81},

		// === gamma setting  ===
		{0xFE, 0x40},
		{0x33, 0x10},

		{0xFE, 0x50},
		{0xA9, 0x00},
		{0xAA, 0x00},
		{0xAB, 0x01},

		{0xFE, 0x60},
		{0xA9, 0x30},
		{0xAA, 0x90},
		{0xAB, 0x01},

		//=== SCC ===
		{0xFE, 0x90},
		{0x51, 0x00},
		{0x52, 0x08},
		{0x53, 0x00},
		{0x54, 0x18},
		{0x55, 0x00},
		{0x56, 0x00},
		{0x57, 0x00},
		{0x58, 0x00},
		{0x59, 0x08},
		{0x5A, 0x00},
		{0x5B, 0x18},
		{0x5C, 0x00},
		{0x5D, 0x00},
		{0x5E, 0x80},
		{0x5F, 0x00},
		{0x60, 0x00},
		{0x61, 0x00},
		{0x62, 0x18},
		{0x63, 0x00},
		{0x64, 0x00},
		{0x65, 0x00},
		{0x66, 0x08},
		{0x67, 0x80},
		{0x68, 0x40},
		{0x69, 0x00},
		{0x6A, 0x00},
		{0x6B, 0x00},
		{0x6C, 0x00},
		{0x6D, 0x00},
		{0x6E, 0x00},
		{0x6F, 0x18},
		{0x70, 0x80},
		{0x71, 0x00},
		{0x72, 0x00},
		{0x73, 0x00},
		{0x74, 0x00},
		{0x75, 0x00},
		{0x76, 0x18},
		{0x77, 0x00},
		{0x78, 0x08},
		{0x79, 0x00},
		{0x7A, 0x00},
		{0x7B, 0x00},
		{0x7C, 0x00},
		{0x7D, 0x18},
		{0x7E, 0x00},
		{0x7F, 0x08},
		{0x80, 0x00},
		{0x81, 0x00},
		{0x82, 0x80},
		{0x83, 0x40},
		{0x84, 0x00},
		{0x85, 0x00},
		{0x86, 0x08},
		{0x87, 0x00},
		{0x88, 0x04},
		{0x89, 0x00},
		{0x8A, 0x18},
		{0x8B, 0x40},
		{0x8C, 0x00},
		{0x8D, 0x00},
		{0x8E, 0x00},
		{0x8F, 0x04},
		{0x90, 0x00},
		{0x91, 0x18},
		{0x92, 0x00},
		{0x93, 0x04},
		{0x94, 0x40},
		{0x95, 0x00},
		{0x96, 0x00},
		{0x97, 0x00},
		{0x98, 0x18},
		{0x99, 0x00},
		{0x9A, 0x04},
		{0x9B, 0x00},
		{0x9C, 0x04},
		{0x9D, 0x80},
		{0x9E, 0x40},
		{0x9F, 0x00},
		{0xA0, 0x00},
		{0xA2, 0x04},

		// === Power saving ===
		{0xFE, 0x70},
		{0x98, 0x74},
		{0xC9, 0x05},
		{0xCA, 0x05},
		{0xCB, 0x05},
		{0xCC, 0x05},
		{0xCD, 0x05},
		{0xCE, 0x85},
		{0xCF, 0x05},
		{0xD0, 0x45},

		{0xFE, 0xE0},
		{0x19, 0x42},
		{0x1E, 0x42},
		{0x1C, 0x41},
		{0x18, 0x00},
		{0x1B, 0x0C},
		{0x1A, 0x9A},
		{0x1D, 0xDA},
		{0x28, 0x5F},

		{0xFE, 0x40},
		{0x54, 0xAC},
		{0x55, 0xA0},
		{0x48, 0xAA},

		//======================== 194*368 setting ============================
		{0xFE, 0x40},
		{0x76, 0x96},
		{0x77, 0xC2},
		{0x78, 0x8E},
		{0x79, 0xB3},
		{0x7A, 0x8D},
		{0x7B, 0x11},

		//======================== EDGE SETTING ============================
		{0xFE, 0x20},
		{0x27, 0xC2},

		//=== CMD1 setting ===
		{0xFE, 0x00},
		{0xC4, 0x80},
		{0x3A, 0x55},
		{0x35, 0x00},
		{0x53, 0x20},
		{0x51, 0xFF},
		{0x63, 0xFF},
};

static void disp_command_1arg(uint8_t command, uint8_t arg) {
	hwspi_begin();
	command_start(command);
	hwspi_send_data(&arg, 1);
	hwspi_end();
}

void disp_icna3306_reset(void) {
	gpio_set_level(m_pin_reset, 0);
	vTaskDelay(1);
	gpio_set_level(m_pin_reset, 1);
	vTaskDelay(200);

	for (int i = 0; i < 378; i ++) {
		disp_command_1arg(init_sequence[i][0], init_sequence[i][1]);
	}

	// CMD1
	disp_command_1arg(0xFE, 0x00);

	// SPI write SRAM enable, single SPI mode
	disp_command_1arg(0xC4, 0x80);

	// RGB565
	disp_command_1arg(0x3A, 0x55);

	// Tearing effect
	disp_command_1arg(0x35, 0x00);

	// Brightness
	disp_command_1arg(0x53, 0x20);
	disp_command_1arg(0x51, 0xFF);
	disp_command_1arg(0x63, 0xFF);

	// Sleep out
	disp_icna3306_command(0x11, 0, 0);
	vTaskDelay(200);

	// Display on
	disp_icna3306_command(0x29, 0, 0);
	vTaskDelay(200);

	disp_icna3306_clear(0);
}
