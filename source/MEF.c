/*
 * MEF.c
 *
 *  Created on: 30 may. 2022
 *      Author: Franco
 */

#include "MEF.h"

typedef enum {
	LED = 0,
	SW,
	ACC
}TipoID;

typedef enum {
	LED_ID_ROJO = 0,
	LED_ID_VERDE,
	SW_ID_1,
	SW_ID_3,
	ACC
}ID;

typedef enum {
	ENCENDER = 0,
	APAGAR,
	TOGGLE,
	LECTURA
}TipoComando;

typedef struct {
	ID periferico;
	TipoComando pedido;
}Comando;
