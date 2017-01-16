/*
 * mat.h
 *
 *  Created on: 14 May 2016
 *      Author: Kristjan
 */

#ifndef MAT_H_
#define MAT_H_


float mapf(float x, float in_min, float in_max, float out_min, float out_max);
float constrain(float value, float max, float min);
float LowPassFilter(float RawData);

#endif /* MAT_H_ */
