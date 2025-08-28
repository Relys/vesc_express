/*
    Copyright 2024 Benjamin Vedder

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAIN_LBM_COLOR_EXTENSIONS_H_
#define MAIN_LBM_COLOR_EXTENSIONS_H_

#include <stdbool.h>

bool lbm_color_extensions_init(void);
uint32_t color_scale(uint32_t color, float scale);
uint32_t color_add_sub(uint32_t color1, uint32_t color2, bool sub);
uint32_t color_mix(uint32_t color1, uint32_t color2, float ratio);

#endif /* MAIN_LBM_COLOR_EXTENSIONS_H_ */
