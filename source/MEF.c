/*
 * MEF.c
 *
 *  Created on: 30 may. 2022
 *      Author: Franco
 */

#include "MEF.h"
#include "ringBuffer.h"
#include "uart0_drv.h"

// Enumeraciones y estructuras para determinar el comando recibido
typedef enum {
	LED = 0,
	LED_ID_ROJO,
	LED_ID_VERDE,
	SW,
	SW_ID_1,
	SW_ID_3,
	ACC,
	ID_ERROR
}ID;

typedef enum {
	ENCENDER = 0,
	APAGAR,
	TOGGLE,
	LECTURA,
	TIPO_COMANDO_ERROR
}TipoComando;

typedef struct {
	ID periferico;
	TipoComando pedido;
}Comando;

// Enumeracion para MEF de Recepcion de datos por UART
typedef enum{
	MefRx_EsperandoSTX = 0,
	MefRx_LeyendoID,
	MefRx_LeyendoPerifericoMSB,
	MefRx_LeyendoPerifericoLSB,
	MefRx_LeyendoAccion,
	MefRx_EsperandoETX
}MEF_Rx_estados;

// funciones para MEF RX

/*
 * determina sobre que periferico se quiere
 * llevar a cabo la accion, dado por la
 * trama recibida por UART.
 */
ID definePeriferico(ID tipoPeriferico, uint8_t tmpRxByte){
	ID retVal;

	switch(tipoPeriferico){
		case LED:
			if(tmpRxByte == 0x31){ // ID recibido 01
				retVal = LED_ID_ROJO;
			}else if(tmpRxByte == 0x32){ // Id recibido 02
				retVal = LED_ID_VERDE;
			}else{
				retVal = ID_ERROR;
			}
			break;
		case SW:
			if(tmpRxByte == 0x31){ // ID recibido 11
				retVal = SW_ID_1;
			}else if(tmpRxByte == 0x33){ // ID recibido 13
				retVal = SW_ID_3;
			}else{
				retVal = ID_ERROR;
			}
			break;
		case ACC:
			if(tmpRxByte == 0x30){ // ID recibido 20
				retVal = ACC;
			}else{
				retVal = ID_ERROR;
			}
			break;
	}
	return retVal;
}

TipoComando definePedido(ID tipoPeriferico, uint8_t tmpRxByte){
	TipoComando retVal;

	switch(tipoPeriferico){
		case LED:
			if(tmpRxByte == 0x45){ // accion recibida E
				retVal = ENCENDER;
			}else if(tmpRxByte == 0x41){ // accion recibida A
				retVal = APAGAR;
			}else if(tmpRxByte == 0x54){ // accion recibida T
				retVal = TOGGLE;
			}else{
				retVal = TIPO_COMANDO_ERROR;
			}
			break;
		case SW:
			retVal = LECTURA;
			break;
		case ACC:
			retVal = LECTURA;
			break;
	}
	return retVal;
}

void MEF_RX_init(){
	uart0_drv_init();
	// la variable rxBuffer deberia estar definida en
	// una libreria "globals.h" o algo similar, creo.
	rxBuffer = ringBuffer_init(100); // ver que tamaño le damos al buffer. ¿100?
}

void MEF_RX_tick(){
	static MEF_Rx_estados estado = MefRx_EsperandoSTX;
	bool start, MSBok;
	ID periferico;
	Comando comando;

	switch(estado){
		case MefRx_EsperandoSTX:
			// la variable rxByte deberia estar definida en
			// una libreria "globals.h" o algo similar, creo.
			if(rxByte == STX){ // ':'
				estado = MefRx_LeyendoID;
				start = true;
				MSBok = false;
			}
			break;
		case MefRx_LeyendoID:
			if(rxByte == 0x31 && start){ // '1'
				start = false;
				MSBok = true;
			}
			if(rxByte == 0x30 && MSBok){ // '0'
				estado = MefRx_LeyendoPerifericoMSB;
			}
			if( ( start && (rxByte != 0x31) ) || ( MSBok && (rxByte != 0x30) ) ){ // hubo algun error en la recepcion
				estado = MefRx_EsperandoSTX;
			}
			break;
		case MefRx_LeyendoPerifericoMSB:
			if(rxByte == 0x30){
				periferico = LED;
			}else if(rxByte == 0x31){
				periferico = SW;
			}else if(rxByte == 0x32){
				periferico = ACC;
			}else{
				periferico = ID_ERROR;
			}
			if(periferico == ID_ERROR){
				estado = MefRx_EsperandoSTX;
			}else{
				estado = MefRx_LeyendoPerifericoLSB;
			}
			break;
		case MefRx_LeyendoPerifericoLSB:
			comando.periferico = definePeriferico(periferico, rxByte);
			if(comando.periferico == ID_ERROR){
				estado = MefRx_EsperandoSTX;
			}else{
				estado = MefRx_LeyendoAccion;
			}
			break;
		case MefRx_LeyendoAccion:
			comando.pedido = definePedido(periferico, rxByte);
			if(comando.pedido == TIPO_COMANDO_ERROR){
				estado = MefRx_EsperandoSTX;
			}else{
				estado = MefRx_EsperandoETX;
			}
			break;
		case MefRx_EsperandoETX:
			if(rxByte == ETX){
				estado = MefRx_EsperandoSTX;
				ringBuffer_putData(rxBuffer, comando); // modificar ringBuffer para que admita esta estructura
													   // esta definido solo para uint8_t
			}
			break;
	}
}
