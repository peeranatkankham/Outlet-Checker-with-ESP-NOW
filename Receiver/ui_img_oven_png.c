// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.4.0
// LVGL version: 8.3.6
// Project name: SquareLine_Project

#include "ui.h"

#ifndef LV_ATTRIBUTE_MEM_ALIGN
    #define LV_ATTRIBUTE_MEM_ALIGN
#endif

// IMAGE DATA: assets/oven.png
const LV_ATTRIBUTE_MEM_ALIGN uint8_t ui_img_oven_png_data[] = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x03,0x7F,0xF7,0x2B,0x7F,0xF7,0x93,0x7F,0xF7,0xEB,0x7F,0xF7,0x9F,0x7F,0xF7,0x30,0xFF,0xFF,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9F,0xF7,0x22,0x7F,0xF7,0x86,0x7F,0xF7,0xD9,0x7F,0xF7,0xFD,0x7F,0xF7,0xFF,0x7F,0xF7,0xFE,0x7F,0xF7,0xE1,0x7F,0xF7,0x96,0x7F,0xF7,0x37,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x07,0x9E,0xF7,0x69,0x7F,0xF7,0xD8,
    0x7F,0xF7,0xF9,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFB,0x7F,0xF7,0xE3,0x7F,0xF7,0x78,0xFF,0xFF,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x07,0x9F,0xF7,0x23,0x7F,0xF7,0xA7,0x7F,0xF7,0xFA,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFC,0x7F,0xF7,0xC2,0x9F,0xF7,0x37,0x3C,0xE7,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x03,0x5F,0xF7,0x35,0x9F,0xF7,0x8F,0x7F,0xF7,0xED,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xF6,0x7F,0xF7,0xA0,0x9F,0xF7,0x3B,0xFF,0xFF,0x06,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5E,0xEF,0x17,0x7F,0xF7,0x83,0x7F,0xF7,0xDC,0x7F,0xF7,0xFC,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFD,0x7F,0xF7,0xE0,0x7F,0xF7,0x95,0x9E,0xF7,0x22,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x03,0x7F,0xEF,0x4F,
    0x7F,0xF7,0xD8,0x7F,0xF7,0xF9,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFB,0x7F,0xF7,0xE7,0x7F,0xF7,0x64,0xFF,0xFF,0x05,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xDE,0x08,0x9F,0xF7,0x27,0x9F,0xF7,0x9B,0x7F,0xF7,0xF8,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFB,0x7F,0xF7,0xAF,0x9F,0xF7,0x35,0x3C,0xE7,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x02,0x9F,0xF7,0x34,0x7F,0xF7,0x94,0x7F,0xF7,0xE3,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xEC,0x7F,0xF7,0xA0,0x7F,0xF7,0x3E,
    0xFF,0xFF,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xDF,0xFE,0x07,0x7F,0xF7,0x77,0x7F,0xF7,0xDE,0x7F,0xF7,0xFB,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFD,0x7F,0xF7,0xE2,0x7F,0xF7,0x91,0x7F,0xEF,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x06,
    0x7F,0xF7,0x3E,0x7F,0xF7,0xCC,0x7F,0xF7,0xFB,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFD,0x7F,0xF7,0x9B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x05,0x9F,0xF7,0x34,0x7F,0xF7,0x96,0x7F,0xF7,0xF2,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xEF,0xFF,0x5F,0xEF,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xD6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x01,0x7F,0xF7,0x2B,0x7F,0xF7,0x98,0x7F,0xF7,0xDC,0x7F,0xF7,0xFE,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0x5F,0xEF,0xFF,0x5F,0xEF,0xFF,0x5F,0xEF,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xDA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x7F,0xF7,0x56,0x7F,0xF7,0xDB,0x7F,0xF7,0xFB,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x3E,0xEF,0xFF,0x3E,0xEF,0xFF,0x3F,0xEF,0xFF,0x5F,0xEF,0xFF,0x1E,0xEF,0xFF,0xFE,0xE6,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x3F,0xE7,0x0A,0x7F,0xF7,0x39,0x7F,0xF7,0xB5,0x7F,0xF7,0xFE,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0x1E,0xE7,0xFF,0x1E,0xEF,0xFF,0x3E,0xEF,0xFF,0x1E,0xE7,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x01,0x9F,0xF7,0x35,0x7F,0xF7,0x99,0x7F,0xF7,0xEC,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xEF,0xFF,0x3F,0xEF,0xFF,0x1E,0xE7,0xFF,0x1E,0xE7,0xFF,0x1E,0xE7,0xFF,0x1E,0xE7,0xFF,0xDE,0xDE,0xFF,0x7D,0xD6,0xFF,0x1D,0xCE,0xFF,0x3D,0xCE,0xFF,0xBD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x00,0x00,0x00,0x7F,0xEF,0x1E,0x7F,0xF7,0x94,0x7F,0xF7,0xDF,0x7F,0xF7,0xFC,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x3F,0xEF,0xFF,0xFE,0xE6,0xFF,0xFE,0xE6,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0xFE,0xE6,0xFF,0xBD,0xDE,0xFF,0x7D,0xD6,0xFF,0xBD,0xC5,0xFF,0xFD,0xCD,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x5E,0xEF,0x18,0x3E,0xEF,0xBD,0x5E,0xEF,0xFC,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0xFE,0xE6,0xFF,0xDE,0xE6,0xFF,0xDE,0xE6,0xFF,0xDE,0xE6,0xFF,0x7D,0xD6,0xFF,0x7D,0xD6,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x1D,0xCE,0xFF,0x9D,0xBD,0xFF,0xFE,0xE6,0xFF,
    0x3E,0xEF,0xFF,0x1E,0xE7,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0xFE,0xE6,0x6E,0x1E,0xE7,0xFD,0x3E,0xEF,0xFF,0x5E,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0x1E,0xE7,0xFF,0xDE,0xDE,0xFF,0xDE,0xDE,0xFF,0xBE,0xDE,0xFF,0x3C,0xCE,0xFF,0x77,0x94,0xFF,0xD3,0x6A,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0xBD,0xC5,0xFF,0x1D,0xCE,0xFF,0x1E,0xE7,0xFF,0x1E,0xE7,0xFF,0xFE,0xE6,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0xBD,0xDE,0xC5,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x3E,0xEF,0xFF,0x5E,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0x1E,0xE7,0xFF,0xDE,0xDE,0xFF,0xBE,0xDE,0xFF,0xBE,0xDE,0xFF,0x5D,0xD6,0xFF,0x7A,0xB5,0xFF,0xD5,0x83,0xFF,0x72,0x5A,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x9D,0xC5,0xFF,0x3D,0xCE,0xFF,0xFE,0xE6,0xFF,0xDE,0xE6,0xFF,0xFE,0xE6,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xDF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDE,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x3E,0xEF,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x3E,0xEF,0xFF,0xBE,0xDE,0xFF,0x9E,0xDE,0xFF,0x9E,0xDE,0xFF,0x9E,0xDE,0xFF,0x7A,0xB5,0xFF,0xD8,0xA4,0xFF,0x97,0x9C,0xFF,0xF3,0x6A,0xFF,0x51,0x5A,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x3D,0xCE,0xFF,0x1D,0xCE,0xFF,0xBD,0xDE,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDE,0xE6,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x5E,0xEF,0xFF,0x7F,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0xDE,0xE6,0xFF,0x7E,0xD6,0xFF,0x9E,0xDE,0xFF,0x7D,0xD6,0xFF,0x1C,0xCE,0xFF,0xD8,0xA4,0xFF,0x77,0x9C,0xFF,0xB7,0x9C,0xFF,0xF5,0x8B,0xFF,0x51,0x52,0xFF,
    0x51,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x7D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x3E,0xEF,0xFF,0x5E,0xEF,0xFF,0x7F,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0xDE,0xE6,0xFF,0x7E,0xD6,0xFF,0x7E,0xD6,0xFF,0x7D,0xD6,0xFF,0xDB,0xC5,0xFF,0xF9,0xAC,0xFF,0x36,0x94,0xFF,0x56,0x94,0xFF,0x97,0x9C,0xFF,0x97,0x9C,0xFF,0x90,0x5A,0xFF,0xCD,0x41,0xFF,0x30,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x5D,0xD6,0xFF,0xBE,0xDE,0xFF,0xDE,0xDE,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,
    0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x3E,0xEF,0xFF,0x5E,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0xFE,0xE6,0xFF,0x7E,0xD6,0xFF,0x5D,0xD6,0xFF,0x5E,0xD6,0xFF,0x3D,0xCE,0xFF,0xD9,0xA4,0xFF,0x16,0x8C,0xFF,0x16,0x8C,0xFF,0x36,0x94,0xFF,0x76,0x94,0xFF,0x96,0x9C,0xFF,0x92,0x73,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0x30,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x7D,0xD6,0xFF,0xBE,0xDE,0xFF,0xDF,0xFF,0xFF,0x9F,0xF7,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x3E,0xEF,0xFF,0x5E,0xEF,0xFF,0x7F,0xF7,0xFF,
    0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x3F,0xEF,0xFF,0x7E,0xD6,0xFF,0x3D,0xD6,0xFF,0x3D,0xD6,0xFF,0x5D,0xD6,0xFF,0x7A,0xB5,0xFF,0xD5,0x83,0xFF,0xB5,0x83,0xFF,0xF5,0x83,0xFF,0x16,0x8C,0xFF,0x56,0x94,0xFF,0x55,0x94,0xFF,0x55,0x94,0xFF,0x0E,0x4A,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0x30,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x5D,0xD6,0xFF,0x5F,0xEF,0xFF,0xFF,0xFF,0xFF,0x9F,0xF7,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x1E,0xE7,0xFF,0x5E,0xEF,0xFF,0x7F,0xEF,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xFF,0x5F,0xEF,0xFF,0xDE,0xDE,0xFF,0x3D,0xCE,0xFF,0x3D,0xCE,0xFF,0x1D,0xCE,0xFF,0x9B,0xBD,0xFF,0x37,0x94,0xFF,0x54,0x7B,0xFF,0x94,0x7B,0xFF,0xD5,0x83,0xFF,
    0xF5,0x8B,0xFF,0x14,0x8C,0xFF,0x34,0x8C,0xFF,0x75,0x94,0xFF,0x31,0x6B,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xEE,0x49,0xFF,0x31,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x5D,0xD6,0xFF,0x7F,0xF7,0xFF,0xFF,0xFF,0xFF,0x1E,0xE7,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x3E,0xEF,0xFF,0x5E,0xEF,0xFF,0x3E,0xEF,0xFF,0xBE,0xDE,0xFF,0x3D,0xD6,0xFF,0x1D,0xCE,0xFF,0x1D,0xCE,0xFF,0x7B,0xB5,0xFF,0x37,0x94,0xFF,0x54,0x73,0xFF,0x33,0x73,0xFF,0x74,0x7B,0xFF,0x94,0x7B,0xFF,0xB3,0x7B,0xFF,0xD3,0x83,0xFF,0x14,0x8C,0xFF,0x54,0x94,0xFF,0xF3,0x83,0xFF,0x0E,0x4A,0xFF,0xCE,0x41,0xFF,0x0F,0x4A,0xFF,0x10,0x52,0xFF,0x31,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x7D,0xD6,0xFF,
    0xDE,0xDE,0xFF,0xFE,0xE6,0xFF,0x9D,0xDE,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFF,0x5D,0xD6,0xFF,0xFD,0xCD,0xFF,0x1D,0xCE,0xFF,0xFD,0xC5,0xFF,0x58,0x94,0xFF,0x13,0x6B,0xFF,0xD3,0x6A,0xFF,0x13,0x6B,0xFF,0x54,0x73,0xFF,0x73,0x73,0xFF,0x72,0x73,0xFF,0xB3,0x7B,0xFF,0xF3,0x83,0xFF,0x34,0x8C,0xFF,0x75,0x94,0xFF,0x11,0x6B,0xFF,0x0F,0x4A,0xFF,0x30,0x52,0xFF,0xEF,0x49,0xFF,0xCD,0x41,0xFF,0x30,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x5D,0xD6,0xFF,0x5D,0xD6,0xFF,0x9D,0xD6,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,
    0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0x7D,0xD6,0xFF,0xFD,0xC5,0xFF,0xDD,0xC5,0xFF,0x3B,0xB5,0xFF,0x34,0x73,0xFF,0x92,0x5A,0xFF,0xB2,0x62,0xFF,0xF3,0x6A,0xFF,0x13,0x6B,0xFF,0x12,0x6B,0xFF,0x51,0x73,0xFF,0x92,0x7B,0xFF,0xD3,0x83,0xFF,0x14,0x8C,0xFF,0x55,0x94,0xFF,0x55,0x8C,0xFF,0x91,0x62,0xFF,0x10,0x52,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0x30,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x7D,0xD6,0xFF,0xFD,0xC5,0xFF,0xDE,0xDE,0xFF,0xDE,0xE6,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xDA,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x1D,0xCE,0xFF,0xBC,0xC5,0xFF,0xF6,0x8B,0xFF,0x72,0x5A,0xFF,0x71,0x5A,0xFF,
    0x92,0x62,0xFF,0xB2,0x62,0xFF,0xD1,0x62,0xFF,0xF1,0x62,0xFF,0x31,0x6B,0xFF,0x72,0x73,0xFF,0xB3,0x7B,0xFF,0xF4,0x83,0xFF,0x55,0x94,0xFF,0x96,0x9C,0xFF,0x73,0x7B,0xFF,0xEE,0x49,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0x30,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x5D,0xD6,0xFF,0x3D,0xCE,0xFF,0xFE,0xE6,0xFF,0xBD,0xDE,0xFF,0xDD,0xDE,0xFF,0x5F,0xEF,0xFF,0x7F,0xF7,0xD4,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x14,0x6B,0xFF,0x51,0x5A,0xFF,0x71,0x5A,0xFF,0x70,0x5A,0xFF,0x90,0x5A,0xFF,0xD0,0x62,0xFF,0x11,0x6B,0xFF,0x51,0x73,0xFF,0x92,0x7B,0xFF,0xD3,0x83,0xFF,0x55,0x94,0xFF,0x96,0x9C,0xFF,0x55,0x94,0xFF,0x0E,0x4A,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,
    0xCD,0x41,0xFF,0xEE,0x49,0xFF,0x51,0x52,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x7D,0xD6,0xFF,0x3D,0xD6,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0x3E,0xE7,0xFF,0x7F,0xF7,0xFF,0x7F,0xF7,0xA5,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x14,0x6B,0xFF,0x71,0x5A,0xFF,0x2E,0x52,0xFF,0x6F,0x52,0xFF,0xB0,0x5A,0xFF,0xF0,0x62,0xFF,0x31,0x6B,0xFF,0x72,0x73,0xFF,0xD4,0x83,0xFF,0x56,0x94,0xFF,0x75,0x94,0xFF,0x95,0x94,0xFF,0xD0,0x62,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0x0F,0x4A,0xFF,0x51,0x52,0xFF,0x51,0x5A,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xDE,0xFF,0xDE,0xE6,0xFF,0x3E,0xEF,0xFF,0x5F,0xEF,0xFF,0x7F,0xEF,0xFB,0x5E,0xF7,0x3D,
    0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x14,0x73,0xFF,0x72,0x5A,0xFF,0x2E,0x52,0xFF,0x8F,0x5A,0xFF,0xD0,0x62,0xFF,0x11,0x6B,0xFF,0x72,0x73,0xFF,0xD4,0x83,0xFF,0x15,0x8C,0xFF,0x34,0x8C,0xFF,0x75,0x94,0xFF,0xF3,0x83,0xFF,0xED,0x49,0xFF,0xCD,0x41,0xFF,0xCD,0x41,0xFF,0xEE,0x49,0xFF,0x31,0x52,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xDE,0xDE,0xFF,0x9D,0xDE,0xFF,0xBD,0xDE,0xFF,0x1E,0xE7,0xFF,0x5F,0xEF,0xFF,0x5E,0xEF,0xFE,0x5F,0xEF,0xE9,0x7F,0xEF,0x75,0xFF,0xFF,0x05,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,
    0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x34,0x73,0xFF,0xB2,0x62,0xFF,0x6F,0x52,0xFF,0xB0,0x5A,0xFF,0x11,0x6B,0xFF,0x73,0x73,0xFF,0xB4,0x83,0xFF,0xD4,0x83,0xFF,0x14,0x84,0xFF,0x54,0x8C,0xFF,0x55,0x94,0xFF,0xB0,0x5A,0xFF,0xCD,0x41,0xFF,0xEE,0x49,0xFF,0x30,0x52,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0xB2,0x62,0xFF,0xDB,0xC5,0xFF,0xFE,0xE6,0xFF,0xFE,0xE6,0xFF,0x3E,0xEF,0xFF,0x3F,0xEF,0xFF,0x5E,0xEF,0xF6,0x5E,0xEF,0xA7,0x5F,0xEF,0x42,0xFF,0xFF,0x07,0x00,0x00,0x00,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x54,0x73,0xFF,0xD2,0x62,0xFF,0x90,0x5A,0xFF,0x12,0x6B,0xFF,0x73,0x73,0xFF,0x73,0x7B,0xFF,0xB3,0x7B,0xFF,0xF3,0x83,0xFF,0x34,0x8C,0xFF,
    0x75,0x94,0xFF,0xD3,0x83,0xFF,0x2F,0x4A,0xFF,0x30,0x52,0xFF,0x51,0x52,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x13,0x6B,0xFF,0x3C,0xCE,0xFF,0x1E,0xE7,0xFF,0x1E,0xEF,0xFF,0x3E,0xEF,0xFE,0x3F,0xEF,0xC8,0x3F,0xEF,0x49,0x1F,0xF7,0x12,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x54,0x7B,0xFF,0xF3,0x6A,0xFF,0x13,0x6B,0xFF,0x32,0x73,0xFF,0x52,0x73,0xFF,0x72,0x73,0xFF,0xD3,0x7B,0xFF,0x14,0x8C,0xFF,0x54,0x8C,0xFF,0x75,0x94,0xFF,0x32,0x6B,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x92,0x5A,0xFF,0xF6,0x8B,0xFF,0x9D,0xD6,0xFF,0xFE,0xE6,0xFF,0x1E,0xE7,0xFD,
    0x1E,0xE7,0xF3,0x3E,0xEF,0x8E,0x3F,0xF7,0x14,0xFF,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x75,0x7B,0xFF,0x13,0x6B,0xFF,0xF1,0x62,0xFF,0x11,0x6B,0xFF,0x52,0x73,0xFF,0xB2,0x7B,0xFF,0xF3,0x83,0xFF,0x34,0x8C,0xFF,0x96,0x9C,0xFF,0x56,0x94,0xFF,0x92,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x92,0x5A,0xFF,0xB5,0x83,0xFF,0xBB,0xBD,0xFF,0xDE,0xE6,0xFF,0xFE,0xE6,0xFE,0x1E,0xE7,0xE7,0x1E,0xE7,0xA0,0x1E,0xE7,0x3B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9D,0xD6,0xE1,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,
    0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x7C,0xBD,0xFF,0x95,0x7B,0xFF,0x33,0x73,0xFF,0x11,0x6B,0xFF,0x51,0x73,0xFF,0x92,0x7B,0xFF,0xD3,0x83,0xFF,0x35,0x8C,0xFF,0x97,0x9C,0xFF,0xD8,0xA4,0xFF,0x13,0x6B,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x71,0x5A,0xFF,0x75,0x7B,0xFF,0x5A,0xB5,0xFF,0x7D,0xD6,0xFF,0xDE,0xE6,0xFF,0xFE,0xE6,0xFE,0xFE,0xE6,0xB1,0x1E,0xE7,0x43,0x3F,0xE7,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9D,0xD6,0xDC,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0xB5,0x83,0xFF,
    0x54,0x73,0xFF,0x31,0x6B,0xFF,0x72,0x73,0xFF,0xB3,0x7B,0xFF,0x35,0x8C,0xFF,0x97,0x9C,0xFF,0xD7,0xA4,0xFF,0x16,0x8C,0xFF,0x71,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x92,0x5A,0xFF,0xB8,0xA4,0xFF,0x5D,0xD6,0xFF,0xBE,0xDE,0xFF,0xDE,0xDE,0xFF,0xFE,0xE6,0xDD,0xFE,0xE6,0x59,0xFD,0xDE,0x0F,0xFF,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9D,0xD6,0x96,0x9D,0xD6,0xF6,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0xD5,0x83,0xFF,0x74,0x7B,0xFF,0x52,0x73,0xFF,0xB3,0x7B,0xFF,0x36,0x8C,0xFF,0x77,0x94,0xFF,0xB7,0x9C,0xFF,0xB7,0x9C,0xFF,0xF3,0x6A,0xFF,0x51,0x5A,0xFF,0x51,0x5A,0xFF,0x92,0x62,0xFF,0x95,0x83,0xFF,0xFC,0xC5,0xFF,0xBE,0xDE,0xFF,
    0xBE,0xDE,0xFE,0xBE,0xDE,0xEA,0xDE,0xDE,0x9B,0xDE,0xE6,0x1B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x3C,0xD6,0x12,0x9D,0xD6,0x6C,0x9D,0xD6,0xC6,0x9D,0xD6,0xFE,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0xD6,0x83,0xFF,0x94,0x7B,0xFF,0xB4,0x83,0xFF,0x16,0x8C,0xFF,0x56,0x94,0xFF,0x97,0x9C,0xFF,0xD7,0xA4,0xFF,0xD5,0x83,0xFF,0x72,0x5A,0xFF,0x72,0x5A,0xFF,0xB5,0x83,0xFF,0x5A,0xB5,0xFF,0x7E,0xD6,0xFF,0x9E,0xDE,0xFF,0xBE,0xDE,0xEA,0xBE,0xDE,0x99,0xDE,0xE6,0x42,0xFF,0xBD,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF7,0xBD,0x04,0xBD,0xD6,0x1E,0x7D,0xD6,0x6B,0x9D,0xD6,0xE5,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0xF6,0x8B,0xFF,0xB5,0x83,0xFF,0xF5,0x8B,0xFF,0x36,0x94,0xFF,0x77,0x9C,0xFF,0xB7,0x9C,0xFF,0xB7,0x9C,0xFF,0xF3,0x6A,0xFF,0x14,0x6B,0xFF,0x1A,0xAD,0xFF,0x3D,0xCE,0xFF,0x7E,0xD6,0xFF,0x9E,0xDE,0xFF,0x9E,0xDE,0xC2,0xBE,0xDE,0x42,0xBF,0xEE,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x03,0x9D,0xD6,0x2D,0x9D,0xDE,0xBE,0x9D,0xD6,0xF9,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,
    0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0x16,0x8C,0xFF,0xD5,0x83,0xFF,0x16,0x8C,0xFF,0x56,0x94,0xFF,0x97,0x9C,0xFF,0xD8,0xA4,0xFF,0x77,0x94,0xFF,0x57,0x94,0xFF,0x1D,0xCE,0xFF,0x5D,0xD6,0xFF,0x7E,0xD6,0xFE,0x9E,0xD6,0xEB,0x9E,0xD6,0x7B,0x7F,0xE6,0x0A,0xFF,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xBF,0xD6,0x06,0x9D,0xDE,0x55,0x9D,0xD6,0xC5,0x9D,0xD6,0xF4,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0x36,0x8C,0xFF,0xF5,0x8B,0xFF,0x36,0x94,0xFF,0x77,0x9C,0xFF,0xB7,0xA4,0xFF,0x39,0xAD,0xFF,0x9B,0xBD,0xFF,
    0x5D,0xD6,0xFF,0x5E,0xD6,0xFF,0x7E,0xD6,0xE8,0x7E,0xD6,0x9D,0x7D,0xD6,0x25,0xFF,0xFF,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x9E,0xDE,0x16,0x9D,0xD6,0x66,0x9D,0xD6,0xD2,0x9D,0xD6,0xFE,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0x57,0x94,0xFF,0x36,0x8C,0xFF,0x76,0x94,0xFF,0xF8,0xA4,0xFF,0x9B,0xBD,0xFF,0x1D,0xCE,0xFF,0x3D,0xD6,0xFF,0x5D,0xD6,0xF0,0x5D,0xD6,0x99,0x5D,0xD6,0x40,0x3F,0xC6,0x09,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x79,0xCE,0x05,0x7D,0xD6,0x1A,0x9D,0xD6,0x7E,0x9D,0xD6,0xF3,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0x9C,0xBD,0xFF,0x57,0x94,0xFF,0x56,0x94,0xFF,0x3A,0xB5,0xFF,0xFD,0xCD,0xFF,0x3D,0xCE,0xFF,0x3D,0xCE,0xFF,0x3D,0xCE,0xD8,0x5E,0xD6,0x46,0xBF,0xD6,0x0C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFF,0x02,0xBD,0xDE,0x37,0x9D,0xD6,0xC2,0x9D,0xD6,0xF4,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,0xBC,0xBD,0xFF,0x19,0xAD,0xFF,0xDC,0xC5,0xFF,0x1D,0xCE,0xFF,0x1D,0xCE,0xFB,0x3D,0xCE,0xE7,0x3D,0xCE,0x8D,0x1E,0xC6,0x11,0x1F,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFD,0xDE,0x0F,0x9D,0xDE,0x5D,0x9D,0xD6,0xB9,0x9D,0xD6,0xF7,0x9D,0xD6,0xFF,0xFD,0xCD,0xFF,
    0xFD,0xC5,0xFF,0xFD,0xCD,0xFF,0x1D,0xCE,0xFF,0x1D,0xCE,0xE8,0x1D,0xCE,0x91,0x3D,0xD6,0x2C,0xEF,0x7B,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x5D,0xD6,0x19,0x9D,0xD6,0x66,0x9D,0xD6,0xE0,0xFD,0xCD,0xFF,0xFD,0xC5,0xFF,0xFD,0xCD,0xF9,0x1D,0xCE,0xA3,0x1E,0xCE,0x37,0x9F,0xD6,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF7,0xBD,0x04,0x9D,0xDE,0x21,0x3D,0xCE,0xC2,0xFD,0xC5,0xD8,0xFD,0xC5,0x52,0xDC,0xD5,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};
const lv_img_dsc_t ui_img_oven_png = {
    .header.always_zero = 0,
    .header.w = 50,
    .header.h = 50,
    .data_size = sizeof(ui_img_oven_png_data),
    .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
    .data = ui_img_oven_png_data
};
