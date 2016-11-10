/*
 * BUZZER.c
 *
 *  Created on: 11 Mar 2016
 *      Author: Kristjan
 */

#ifndef BUZZER_C_
#define BUZZER_C_

void initBuzzer();
void BuzzerShort();
void BuzzerLong();
void GreenLed(bool enable);
void RedLed(bool enable);
void BlueLed(bool enable);
void MixLed(int R, int G, int B);

#endif /* BUZZER_C_ */
