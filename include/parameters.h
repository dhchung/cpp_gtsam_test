#pragma once

#include <math.h>

#define base_line 120.0 //[mm]
#define img_center_x 236.0 //[px]
#define img_center_y 172.0 //[px]
#define focal_length 526.0 //
#define depth_err 300.0 //[mm]

#define odom_noise_translation 0.01 //[m]
#define odom_noise_angle 0.5*M_PI/180.0
#define init_noise_translation 0.0001//[m]
#define init_noise_angle 0.0005*M_PI/180.0f
#define measure_noise_normal 0.001
#define measure_noise_distance 0.0001//[m]

#define absolute_noise_translation 0.05 //[m]
#define absolute_noise_angle 0.5*M_PI/180.0
