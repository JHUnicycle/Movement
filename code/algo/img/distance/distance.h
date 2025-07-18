#ifndef _IMG_DISTANCE_H
#define _IMG_DISTANCE_H

#include "zf_common_headfile.h"

// 摄像头高度为170.47mm
#define CAMERA_HEIGHT 170.47f
#define CAMERA_ANGLE 70.0f

int16 get_image_horizon();
float distance_reckon(int16 x, int16 y, float c);
float distance_reckon_horizontal(int16 x, int16 y, int16 c);

#endif