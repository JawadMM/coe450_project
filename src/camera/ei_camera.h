#pragma once

#include <stdint.h>
#include <stddef.h>

/* Function declarations */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

/* External variable declarations */
extern uint8_t *snapshot_buf;
extern bool debug_nn;