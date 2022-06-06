/*
 * MEF.c
 *
 *  Created on: 30 may. 2022
 *      Author: Franco
 */

#include "MEF.h"
#include "ringBuffer.h"
#include "fsl_debug_console.h"

//#include "uart0_drv.h"
#include "uart1_drv.h"
/* ================== [Enumeraciones y estructuras para determinar el comando recibido] ================== */
typedef enum{
	LED_T = 0,
	SW_T,
	ACC_T,
	ERROR_T
}TipoPeriferico;

typedef enum {
	LED_ID_ROJO = 0,
	LED_ID_VERDE,
	SW_ID_1,
	SW_ID_3,
	ACC,
	ID_ERROR
}IdPeriferico;

typedef enum {
	ENCENDER = 0,
	APAGAR,
	TOGGLE,
	LECTURA,
	TIPO_COMANDO_ERROR
}TipoComando;

typedef struct {
	IdPeriferico periferico;
	TipoComando pedido;
}Comando;

/* ==================[Enumeracion para MEF de Recepcion de datos por UART] ================== */
typedef enum{
	MefRx_EsperandoSTX = 0,
	MefRx_LeyendoID,
	MefRx_LeyendoPerifericoMSB,
	MefRx_LeyendoPerifericoLSB,
	MefRx_LeyendoAccion,
	MefRx_EsperandoETX
}MEF_Rx_estados;


/* ==================[variables internas] ================== */
void *cmdBuffer;
static uint8_t rxBuffer[20];
static int32_t ret;

/* ================== [funciones internas para MEF RX] ================== */
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
	return (uart_drv_recDatos(retPtr, 1) == 1);
}


/* ================== [funciones publicas para MEF RX] ================== */

uint8_t msg[] = "Hola\r\n";

void MEF_RX_init(){
	uart_drv_init();
	cmdBuffer = ringBuffer_init(RB_SIZE);
	uart_drv_envDatos(msg, sizeof(msg)-1);
}

void MEF_RX_tick(){
	static MEF_Rx_estados estado = MefRx_EsperandoSTX;
	static bool start, MSBok;
	TipoPeriferico tmpPeriferico = ERROR_T;
	TipoComando tmpComando = TIPO_COMANDO_ERROR;
	static Comando comando;


	ret = uart_drv_recDatos(rxBuffer, sizeof(rxBuffer));
	PRINTF("%c\n", ret);

	switch(estado){
		case MefRx_EsperandoSTX:
			if(ret){
				if(rxBuffer[0] == STX){ // ':'
					estado = MefRx_LeyendoID;
					start = true;
					MSBok = false;
				}
			}
			break;
		case MefRx_LeyendoID:
			if(ret){
				if(rxBuffer[0] == 0x31 && start){ // '1'
					start = false;
					MSBok = true;
				}
				if(rxBuffer[0] == 0x30 && MSBok){ // '0'
					estado = MefRx_LeyendoPerifericoMSB;
				}
				if( (start && (rxBuffer[0] != 0x31) ) || (MSBok && (rxBuffer[0] != 0x30)) ){ // hubo algun error en la recepcion
					estado = MefRx_EsperandoSTX;
				}
			}
			break;
		case MefRx_LeyendoPerifericoMSB:
			if(ret){
				if(rxBuffer[0] == 0x30){
					tmpPeriferico = LED_T;
				}else if(rxBuffer[0] == 0x31){
					tmpPeriferico = SW_T;
				}else if(rxBuffer[0] == 0x32){
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
			if(ret){
				comando.periferico = definePeriferico(tmpPeriferico, rxBuffer[0]);
				if(comando.periferico == ID_ERROR){
					estado = MefRx_EsperandoSTX;
				}else if(comando.periferico == LED_ID_ROJO || comando.periferico == LED_ID_VERDE){
					estado = MefRx_LeyendoAccion;
				}else{
					estado = MefRx_EsperandoETX;
				}
			}
			break;
		case MefRx_LeyendoAccion:
			if(ret){
				if(rxBuffer[0] == 0x45){ // accion recibida E
					tmpComando = ENCENDER;
				}else if(rxBuffer[0] == 0x41){ // accion recibida A
					tmpComando = APAGAR;
				}else if(rxBuffer[0] == 0x54){ // accion recibida T
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
			if(ret){
				if(rxBuffer[0] == ETX){
					estado = MefRx_EsperandoSTX;
					ringBuffer_putData(cmdBuffer, (uint8_t)&comando); // revisar. Habia pensado en hacer un RB de punteros a Comando
				}
			}
			break;
	}
}
