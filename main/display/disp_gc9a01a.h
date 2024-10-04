/*
    Copyright 2023 Benjamin Vedder    benjamin@vedder.se
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

#ifndef MAIN_DISPLAY_DISP_GC9A01A_H_
#define MAIN_DISPLAY_DISP_GC9A01A_H_

#include <stdint.h>
#include <stdbool.h>
#include "lispif_disp_extensions.h"
// Example initialization sequence for the GC9A01A
static const uint8_t gc9a01a_init_sequence[][7] = {
    {3, 0xEF, 0x03, 0x80},
    {4, 0xFE, 0x00, 0x01, 0x02},  // Adjust based on datasheet
    {2, 0x36, 0x48},  // Memory Access Control, 48h means RGB format
    {2, 0x3A, 0x55},  // Pixel Format Set, 16-bit color (RGB565)
    {2, 0xB1, 0x00},  // Frame Rate Control
    {5, 0xF2, 0x03, 0x12, 0xA4, 0xFF},  // Adjust for power and timing control
    {2, 0x11, 0x00},  // Exit Sleep mode
    {2, 0x29, 0x00},  // Display ON
    // Add other necessary commands as per the datasheet...
};

void disp_gc9a01a_init(int pin_sd0, int pin_clk, int pin_cs, int pin_reset, int pin_dc, int clock_mhz);
void disp_gc9a01a_command(uint8_t command, const uint8_t *args, int argn);
bool disp_gc9a01a_render_image(image_buffer_t *img, uint16_t x, uint16_t y, color_t *colors);
void disp_gc9a01a_clear(uint32_t color);
void disp_gc9a01a_reset(void);

#endif /* MAIN_DISPLAY_DISP_GC9A01A_H_ */