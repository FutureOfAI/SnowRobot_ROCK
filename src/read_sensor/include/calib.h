#ifndef _CALIB_H
#define _CALIB_H

#include "core.h"
#include <string>

using namespace std;

typedef void (*goto_type)(_YAT_POINTF_THETA);

bool init_calib(string filename);

void goto_calib_pose(_YAT_POINTF_THETA &calib_pose, goto_type gotofunc);

bool goto_calib(_YAT_POINTF_THETA &calib_pose, _YAT_POINTF_THETA &pos_theta, goto_type gotofunc);

bool calib_mode(_YAT_POINTF_THETA &pos_theta, _YAT_POINTF_THETA calibration_start, goto_type gotofunc);

void move_forward();

void stop_running();

void turning(float angle);

bool find_chessboard(_YAT_POINTF_THETA &pos_theta);

#endif