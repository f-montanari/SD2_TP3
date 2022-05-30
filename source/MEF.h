/*
 * MEF.h
 *
 *  Created on: 25 may. 2022
 *      Author: Franco
 */

#ifndef MEF_H_
#define MEF_H_

#include "globals.h"

#define STX 0x3A // ":"
#define ETX 0x0A // LF

void MEF_RX_init();
void MEF_RX_tick();

#endif /* MEF_H_ */
