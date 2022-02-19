/*
 * Copyright (c) 2017, Stanislav Lakhtin. All rights reserved.
 * Copyright (c) 2022, Vladislav Tsendrovskii
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. No personal names or organizations' names associated with the
 *    Atomthreads project may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#define _swap(a, b) { uint8_t t = a; a = b; b = t; }
#define _bitSet(x) (1 << (x))
#define _bitClear(x) (~(1 << x))
#define _bitCheck(number, x) (number >> x) & 1

// address of device is // 011110+SA0+RW - 0x3C or 0x3D

#define DEFAULT_7bit_OLED_SLAVE_ADDRESS 0x3C

// Addressing mode

#ifndef DEFAULTBUFFERLENGTH
#define DEFAULTBUFFERLENGTH 1024
#endif

enum SSD1306_AddressingMode {
  Horizontal  = 0b00,
  Vertical    = 0b01,
  Page        = 0b10, // RESET
  INVALID     = 0b11  // You MUST NOT USE IT
};

struct SSD1306_State_s {
    uint32_t I2C_OLED;
    uint8_t OLED_ADDRESS;
    uint8_t WIDTH;
    uint8_t HEIGHT;
    uint16_t screenBufferLength;
    enum SSD1306_AddressingMode AddressingMode;
    uint8_t screenRAM[DEFAULTBUFFERLENGTH];

    void (*i2c_send_bytes)(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t len, bool *ok);
    void (*i2c_read_bytes)(uint32_t i2c, uint8_t addr, uint8_t *data, size_t len, bool *ok);
};

typedef enum SSD1306_COLOR { white = 0, black = 1} Color;
typedef enum SSD1306_WRAP {nowrap, wrapDisplay, wrapCoord} WrapType;

void ssd1306_init(struct SSD1306_State_s *state,
                  uint32_t i2c, uint8_t address,
                  uint8_t width, uint8_t height,
                  void (*i2c_send_bytes)(uint32_t i2c, uint8_t addr, const uint8_t *data, size_t len, bool *ok),
                  void (*i2c_read_bytes)(uint32_t i2c, uint8_t addr, uint8_t *data, size_t len, bool *ok));

bool ssd1306_send_data(struct SSD1306_State_s *state, uint8_t spec, uint8_t data);

// hardware commands
void ssd1306_setMemoryAddressingMode(struct SSD1306_State_s *state, enum SSD1306_AddressingMode mode);
void ssd1306_setColumnAddressScope(struct SSD1306_State_s *state, uint8_t lower, uint8_t upper);
void ssd1306_setPageAddressScope(struct SSD1306_State_s *state, uint8_t lower, uint8_t upper);
void ssd1306_setPageStartAddressForPageAddressingMode(struct SSD1306_State_s *state, uint8_t pageNum);
void ssd1306_setDisplayStartLine(struct SSD1306_State_s *state, uint8_t startLine);
void ssd1306_setContrast(struct SSD1306_State_s *state, uint8_t value);
void ssd1306_setPrecharge(struct SSD1306_State_s *state, uint8_t value);
void ssd1306_setDisplayOn(struct SSD1306_State_s *state, bool resume); // switch ON/OFF MCU of display
void ssd1306_setInverse(struct SSD1306_State_s *state, bool inverse);
void ssd1306_chargePump(struct SSD1306_State_s *state, bool chargePump);
void ssd1306_switchOLEDOn(struct SSD1306_State_s *state, bool goOn); //switch ON/OFF power switch of the OLED panel
void ssd1306_setDisplayOffset(struct SSD1306_State_s *state, uint8_t verticalShift);
void ssd1306_adjustVcomDeselectLevel(struct SSD1306_State_s *state, uint8_t value);
void ssd1306_setOscillatorFrequency(struct SSD1306_State_s *state, uint8_t value); // you SHOULD use default value (0x80)
void ssd1306_setMultiplexRatio(struct SSD1306_State_s *state, uint8_t ratio);
void ssd1306_setCOMPinsHardwareConfiguration(struct SSD1306_State_s *state, uint8_t);
void ssd1306_setPage(struct SSD1306_State_s *state, uint8_t);
void ssd1306_setColumn(struct SSD1306_State_s *state, uint8_t);

// graphics methods
void ssd1306_draw_block(struct SSD1306_State_s *state, uint8_t row, uint8_t col, uint8_t data);
