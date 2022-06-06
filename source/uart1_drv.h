/*
 * uart1_drv.h
 *
 *  Created on: 2 jun. 2022
 *      Author: Franco
 */

#ifndef UART1_DRV_H_
#define UART1_DRV_H_
/*==================[inclusions]=============================================*/
#include "stdint.h"
#include "stdbool.h"

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions definition]==========================*/

void uart_drv_init(void);

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/
int32_t uart_drv_recDatos(uint8_t *pBuf, int32_t size);

/** \brief envía datos por puerto serie vía DMA
 **
 ** \param[inout] pBuf buffer a donde estan los datos a enviar
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes enviados
 **/
int32_t uart_drv_envDatos(uint8_t *pBuf, int32_t size);

/*==================[cplusplus]==============================================*/
#ifdef __cplusplus
}
#endif



#endif /* UART1_DRV_H_ */
