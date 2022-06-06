/* Copyright 2022, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2022, 2019, 2018, Gustavo Muro - Daniel Márquez
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

// Standard C Included Files
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Project Included Files
#include "SD2_board.h"
#include "fsl_lpsci.h"
#include "fsl_port.h"
#include "fsl_lpsci_dma.h"
#include "fsl_dmamux.h"
#include "board.h"

#include "uart0_drv.h"
#include "ringBuffer.h"

/*==================[macros and definitions]=================================*/
#define LPSCI_TX_DMA_CHANNEL 0U
#define BAUD_RATE 115200
#define BUFFER_DMA_SIZE  32

/*==================[internal data declaration]==============================*/
// Tx DMA
uint8_t lpsciTxBuffer_dma[BUFFER_DMA_SIZE];
static lpsci_dma_handle_t lpsciDmaHandle;
static dma_handle_t lpsciTxDmaHandle;
volatile bool lpsciTxOnGoing = false;

// Rx Ring Buffer
static void* lpsciRingBufferRx;

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/* UART user callback */
static void LPSCI_UserCallback(UART0_Type *base, lpsci_dma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_LPSCI_TxIdle == status)
    {
        lpsciTxOnGoing = false;
    }
}

/*==================[external functions definition]==========================*/

void uart0_drv_init(void)
{
    lpsci_config_t config;

    // Crear RingBuffer para Rx
	lpsciRingBufferRx = ringBuffer_init(BUFFER_DMA_SIZE);

    CLOCK_SetLpsci0Clock(0x1U);

    /* PORTA1 (pin 35) is configured as UART0_RX */
    PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);

    /* PORTA2 (pin 36) is configured as UART0_TX */
    PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

    /*
     * config.parityMode = kLPSCI_ParityDisabled;
     * config.stopBitCount = kLPSCI_OneStopBit;
     * config.enableTx = false;
     * config.enableRx = false;
     */
    LPSCI_GetDefaultConfig(&config);
    config.baudRate_Bps = BAUD_RATE;
    config.parityMode = kLPSCI_ParityDisabled;
    config.stopBitCount = kLPSCI_OneStopBit;
    config.enableTx = true;
    config.enableRx = true;

    LPSCI_Init(UART0, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));

    /* CONFIGURACIÓN DMA */
    /* Init DMAMUX */
    DMAMUX_Init(DMAMUX0);

    /* Set channel for LPSCI  */
	DMAMUX_SetSource(DMAMUX0, LPSCI_TX_DMA_CHANNEL, kDmaRequestMux0LPSCI0Tx);
	DMAMUX_EnableChannel(DMAMUX0, LPSCI_TX_DMA_CHANNEL);

	/* Init the DMA module */
	DMA_Init(DMA0);
	DMA_CreateHandle(&lpsciTxDmaHandle, DMA0, LPSCI_TX_DMA_CHANNEL);

    /* Create LPSCI DMA handle. */
    LPSCI_TransferCreateHandleDMA(
            UART0,
            &lpsciDmaHandle,
            LPSCI_UserCallback,
            NULL,
            &lpsciTxDmaHandle,
            NULL);

    /* Habilita interrupciones */
	LPSCI_EnableInterrupts(UART0, kLPSCI_TransmissionCompleteInterruptEnable);
	LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable);
	EnableIRQ(UART0_IRQn);

}


/** \brief envía datos por puerto serie accediendo a memoria RAM
 **
 ** \param[inout] pBuf buffer a donde estan los datos a enviar
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes enviados
 **/
int32_t uart0_drv_envDatos(uint8_t *pBuf, int32_t size)
{
    lpsci_transfer_t xfer;

    if (lpsciTxOnGoing)
    {
        size = 0;
    }
    else
    {
        /* limita size */
        if (size > BUFFER_DMA_SIZE)
            size = BUFFER_DMA_SIZE;

        // Hace copia del buffer a transmitir en otro arreglo
        memcpy(lpsciTxBuffer_dma, pBuf, size);

        xfer.data = lpsciTxBuffer_dma;
        xfer.dataSize = size;

        lpsciTxOnGoing = true;
        LPSCI_TransferSendDMA(UART0, &lpsciDmaHandle, &xfer);

        LPSCI_EnableInterrupts(UART0, kLPSCI_TransmissionCompleteInterruptEnable);


    }

    return size;
}

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/
int32_t uart0_drv_recDatos(uint8_t *pBuf, int32_t size){
	int32_t ret = 0;

	NVIC_DisableIRQ(UART0_IRQn);

	while(!ringBuffer_isEmpty(lpsciRingBufferRx) && ret < size){
		ringBuffer_getData(lpsciRingBufferRx, &pBuf[ret]);
		ret++;
	}

	NVIC_EnableIRQ(UART0_IRQn);
	return ret;

}

void UART0_IRQHandler(void)
{

	// Tx
    if ( (kLPSCI_TransmissionCompleteFlag)            & LPSCI_GetStatusFlags(UART0) &&
         (kLPSCI_TransmissionCompleteInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
    {
        LPSCI_DisableInterrupts(UART0, kLPSCI_TransmissionCompleteInterruptEnable);
        LPSCI_ClearStatusFlags(UART0, kLPSCI_TransmissionCompleteFlag);
    }

    // Rx
    if ( (kLPSCI_RxDataRegFullFlag)            & LPSCI_GetStatusFlags(UART0) &&
		 (kLPSCI_RxDataRegFullInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
	{
    	uint8_t data;
    	data = LPSCI_ReadByte(UART0);
    	ringBuffer_putData(lpsciRingBufferRx, data);
	}
}


/*==================[end of file]============================================*/

