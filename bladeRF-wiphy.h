/*
 * This file is part of the bladeRF project
 *
 * Copyright (C) 2020 Nuand LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

struct bladeRF_wiphy_header_tx {
   uint16_t rsvd;
   uint16_t flags;
   uint16_t modulation;
   uint16_t bandwidth;
   uint16_t len;
   uint16_t rsvd2;
   uint32_t cookie;
};

struct bladeRF_wiphy_header_rx {
   uint16_t type;
   uint8_t  bandwidth;
   uint8_t  modulation;
   union {
      struct {
         uint16_t len;
         uint16_t rsvd2;
      };
      uint32_t cookie;
   };
   uint32_t rsvd3;
};
