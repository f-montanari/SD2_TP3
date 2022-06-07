/*
 * MEF.c
 *
 *  Created on: 30 may. 2022
 *      Author: Franco
 */

#include "MEF.h"
#include "string.h"
#include "ringBuffer.h"
#include "mma8451.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* ================== [Definiciones] ================== */

/* ==================[Enumeracion para estados MEF] ================== */
// Mef Rx
typedef enum{
	MefRx_EsperandoSTX = 0,
	MefRx_LeyendoID,
	MefRx_LeyendoPerifericoMSB,
	MefRx_LeyendoPerifericoLSB,
	MefRx_LeyendoAccion,
	MefRx_EsperandoETX
}MEF_Rx_e;

// Mef procesamiento
typedef enum{
	MefProcesamiento_LeyendoComandoSiguiente = 0,
	MefProcesamiento_EjecutandoComandoSiguiente,
	MefProcesamiento_TransmitiendoRespuesta
}MEF_Procesamiento_e;

/* ==================[variables internas] ================== */
// Mef Procesamiento


/* ================== [funciones internas] ================== */
// Mef Rx
/*
 * determina sobre que periferico se quiere
 * llevar a cabo la accion, dado por la
 * trama recibida por UART.
 */
IdPeriferico definePeriferico(TipoPeriferico tipoPeriferico, uint8_t tmpRxByte){
	IdPeriferico retVal;

	switch(tipoPeriferico){
		case LED_T:
			if(tmpRxByte == 0x31){ // ID recibido 01
				retVal = LED_ID_ROJO;
			}else if(tmpRxByte == 0x32){ // Id recibido 02
				retVal = LED_ID_VERDE;
			}else{
				retVal = ERROR_T;
			}
			break;
		case SW_T:
			if(tmpRxByte == 0x31){ // ID recibido 11
				retVal = SW_ID_1;
			}else if(tmpRxByte == 0x33){ // ID recibido 13
				retVal = SW_ID_3;
			}else{
				retVal = ERROR_T;
			}
			break;
		case ACC_T:
			if(tmpRxByte == 0x30){ // ID recibido 20
				retVal = ACC;
			}else{
				retVal = ERROR_T;
			}
			break;
		case ERROR_T:
			retVal = ERROR_T;
			break;
	}
	return retVal;
}

bool MEF_readByte(uint8_t *retPtr){
	return (uart0_drv_recDatos(retPtr, 1) == 1);
}

// Mef Procesamiento
uint16_t getAcc(){
	return(sqrt( pow(mma8451_getAcX(), 2) +
				 pow(mma8451_getAcY(), 2) +
				 pow(mma8451_getAcZ(), 2) ));
}

void separarEnDigitos(uint8_t *d0, uint8_t *d1, uint8_t *d2, uint16_t value){
	*d2 = value / 100;
	*d1 = (value - *d2 * 100) / 10;
	*d0 = value - *d2 * 100 - *d1*10;
}

uint8_t *ejecutarComando(uint8_t *resultado, Comando *comandoSiguiente){
	//uint8_t *resultado;

	switch(comandoSiguiente->pedido){
		case ENCENDER:
			if(comandoSiguiente->periferico == LED_ID_ROJO){
				board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_ON);
				//resultado = malloc(sizeof(uint8_t)*8);
				sprintf(resultado, ":1001E\n");
			}else{
				board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_ON);
				//resultado = malloc(sizeof(uint8_t)*8);
				sprintf(resultado, ":1002E\n");
			}
			break;
		case APAGAR:
			if(comandoSiguiente->periferico == LED_ID_ROJO){
				board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_OFF);
				//resultado = malloc(sizeof(uint8_t)*8);
				sprintf(resultado, ":1001A\n");
			}else{
				board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_OFF);
				//resultado = malloc(sizeof(uint8_t)*8);
				sprintf(resultado, ":1002A\n");
			}
			break;
		case TOGGLE:
			if(comandoSiguiente->periferico == LED_ID_ROJO){
				board_setLed(BOARD_LED_ID_ROJO, BOARD_LED_MSG_TOGGLE);
				//resultado = malloc(sizeof(uint8_t)*8);
				sprintf(resultado, ":1001T\n");
			}else{
				board_setLed(BOARD_LED_ID_VERDE, BOARD_LED_MSG_TOGGLE);
				//resultado = malloc(sizeof(uint8_t)*8);
				sprintf(resultado, ":1002T\n");
			}
			break;
		case LECTURA:
			if(comandoSiguiente->periferico == SW_ID_1){
				bool isOn;
				isOn = board_getSw(BOARD_SW_ID_1);
				//resultado = malloc(sizeof(uint8_t)*8);
				if(isOn){
					sprintf(resultado, ":1011P\n");
				}else{
					sprintf(resultado, ":1011N\n");
				}
			}else if(comandoSiguiente->periferico == SW_ID_3){
				bool isOn;
				isOn = board_getSw(BOARD_SW_ID_3);
				//resultado = malloc(sizeof(uint8_t)*8);
				if(isOn){
					sprintf(resultado, ":1013P\n");
				}else{
					sprintf(resultado, ":1013N\n");
				}
			}else{
				uint8_t D0, D1, D2;
				uint16_t acc;
				acc = getAcc();
				separarEnDigitos(&D0, &D1, &D2, acc);
				//resultado = malloc(sizeof(uint8_t)*9);
				sprintf(resultado, ":1020%d%d%d\n", D2, D1, D0);
			}
			break;
		case TIPO_COMANDO_ERROR:
			// para no tener warning
			break;
	}

	return resultado;
}

/* ================== [funciones publicas] ================== */
// Mef Rx
void MefRxInit(){

}

void MefRxTick(void *ringBufferComandos){
	static MEF_Rx_e estado = MefRx_EsperandoSTX;
	static bool start, byteAvailable;
	static TipoPeriferico tmpPeriferico = ERROR_T;
	static TipoComando tmpComando = TIPO_COMANDO_ERROR;
	static Comando comando;
	uint8_t rxChar;

	byteAvailable = uart0_drv_recDatos(&rxChar, 1);

	switch(estado){
		case MefRx_EsperandoSTX:
			if(byteAvailable){
				if(rxChar == STX){ // ':'
					estado = MefRx_LeyendoID;
					start = true;
				}
			}
			break;
		case MefRx_LeyendoID:
			if(byteAvailable){
				if(rxChar == STX){
					start = true;
					break;
				}
				if(start){
					(rxChar == 0x31) ? (start = false) : (estado = MefRx_EsperandoSTX);
				}else{
					if(rxChar == 0x30){
						estado = MefRx_LeyendoPerifericoMSB;
					}else{
						estado = MefRx_EsperandoSTX;
					}
				}
			}
			break;
		case MefRx_LeyendoPerifericoMSB:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				if(rxChar == 0x30){
					tmpPeriferico = LED_T;
				}else if(rxChar == 0x31){
					tmpPeriferico = SW_T;
				}else if(rxChar == 0x32){
					tmpPeriferico = ACC_T;
				}else{
					tmpPeriferico = ERROR_T;
				}
				if(tmpPeriferico == ERROR_T){
					estado = MefRx_EsperandoSTX;
				}else{
					estado = MefRx_LeyendoPerifericoLSB;
				}
			}
			break;
		case MefRx_LeyendoPerifericoLSB:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				comando.periferico = definePeriferico(tmpPeriferico, rxChar);
				if(comando.periferico == ID_ERROR){
					estado = MefRx_EsperandoSTX;
				}else if(comando.periferico == LED_ID_ROJO || comando.periferico == LED_ID_VERDE){
					estado = MefRx_LeyendoAccion;
				}else{
					estado = MefRx_EsperandoETX;
					comando.pedido = LECTURA;
				}
			}
			break;
		case MefRx_LeyendoAccion:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				if(rxChar == 0x45){ // accion recibida E
					tmpComando = ENCENDER;
				}else if(rxChar == 0x41){ // accion recibida A
					tmpComando = APAGAR;
				}else if(rxChar == 0x54){ // accion recibida T
					tmpComando = TOGGLE;
				}else{
					tmpComando = TIPO_COMANDO_ERROR;
				}
				if(tmpComando == TIPO_COMANDO_ERROR){
					estado = MefRx_EsperandoSTX;
				}else{
					estado = MefRx_EsperandoETX;
					comando.pedido = tmpComando;
				}
			}
			break;
		case MefRx_EsperandoETX:
			if(byteAvailable){
				if(rxChar == STX){
					estado = MefRx_LeyendoID;
					start = true;
					break;
				}
				if(rxChar == ETX){
					ringBufferComando_putData(ringBufferComandos, comando);
				}
				estado = MefRx_EsperandoSTX;
			}
			break;
	}
}

// Mef Procesamiento
void MefProcesamientoInit(){

}

void MefProcesamientoTick(void *ringBufferComandos){
	static MEF_Procesamiento_e estadoMP = MefProcesamiento_LeyendoComandoSiguiente;
	static Comando comandoPendiente;
	static uint8_t resultado[10];

	switch(estadoMP){
		case MefProcesamiento_LeyendoComandoSiguiente:
			if(!ringBufferComando_isEmpty(ringBufferComandos)){
				ringBufferComando_getData(ringBufferComandos, &comandoPendiente);
				estadoMP = MefProcesamiento_EjecutandoComandoSiguiente;
			}
			break;
		case MefProcesamiento_EjecutandoComandoSiguiente:
			ejecutarComando(resultado, &comandoPendiente);
			estadoMP = MefProcesamiento_TransmitiendoRespuesta;
			break;
		case MefProcesamiento_TransmitiendoRespuesta:
			if( uart0_drv_envDatos(resultado, strlen(resultado)) ){
				estadoMP = MefProcesamiento_LeyendoComandoSiguiente;
			}
			break;
	}
}
