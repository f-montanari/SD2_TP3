/*
 * uart1_drv.c
 *
 *  Created on: 2 jun. 2022
 *      Author: Franco
 */

#include "uart1_drv.h"

#include "fsl_uart_dma.h"
#include "fsl_dmamux.h"
#include "fsl_port.h"

#include "clock_config.h"
#include "pin_mux.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define UART_CHANNEL UART1
#define UART_BAUDRATE 115200
#define UART_TX_DMA_CHANNEL 0U
#define UART_TX_DMA_REQUEST kDmaRequestMux0UART1Tx
#define BUFFER_LENGTH 8


/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_dma_handle_t *handle, status_t status, void *userData);

/*******************************************************************************
 * Variables
 ******************************************************************************/

uart_dma_handle_t g_uartDmaHandle;
dma_handle_t g_uartTxDmaHandle;
dma_handle_t g_uartRxDmaHandle;
uint8_t g_tipString[] = "UART DMA example\r\nSend back received data\r\nEcho every 8 characters\r\n";
uint8_t g_txBuffer[BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty = true;
volatile bool txBufferFull = false;
volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_dma_handle_t *handle, status_t status, void *userData)
{
    userData = userData;

    if (kStatus_UART_TxIdle == status)
    {
        txBufferFull = false;
        txOnGoing = false;
    }

    if (kStatus_UART_RxIdle == status)
    {
        rxBufferEmpty = false;
        rxOnGoing = false;
    }
}

void uart_drv_init(void)
{
	uart_config_t config;
	uart_transfer_t xfer;
	uart_transfer_t sendXfer;
	uart_transfer_t receiveXfer;

	/* Port E Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortE);

	/* PORTE0 (pin 1) is configured as UART1_TX */
	PORT_SetPinMux(PORTE, 0U, kPORT_MuxAlt3);

	/* PORTE1 (pin 2) is configured as UART1_RX */
	PORT_SetPinMux(PORTE, 1U, kPORT_MuxAlt3);

	SIM->SOPT5 = ((SIM->SOPT5 &
				   /* Mask bits to zero which are setting */
				   (~(SIM_SOPT5_UART1TXSRC_MASK | SIM_SOPT5_UART1RXSRC_MASK)))

				  /* UART1 Transmit Data Source Select: UART1_TX pin. */
				  | SIM_SOPT5_UART1TXSRC(SOPT5_UART1TXSRC_UART_TX)

				  /* UART1 Receive Data Source Select: UART1_RX pin. */
				  | SIM_SOPT5_UART1RXSRC(SOPT5_UART1RXSRC_UART_RX));

	/* Initialize the UART. */
	/*
	 * config.baudRate_Bps = 115200U;
	 * config.parityMode = kUART_ParityDisabled;
	 * config.stopBitCount = kUART_OneStopBit;
	 * config.txFifoWatermark = 0;
	 * config.rxFifoWatermark = 1;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = UART_BAUDRATE;
	config.enableTx = true;
	config.enableRx = true;

	UART_Init(UART_CHANNEL, &config, CLOCK_GetFreq(BUS_CLK));

	/* Init DMAMUX */
	DMAMUX_Init(DMAMUX0);

	/* Set channel for FLEXIO_UART */
	DMAMUX_SetSource(DMAMUX0, UART_TX_DMA_CHANNEL, UART_TX_DMA_REQUEST);
	DMAMUX_EnableChannel(DMAMUX0, UART_TX_DMA_CHANNEL);

	/* Init the DMA module */
	DMA_Init(DMA0);
	DMA_CreateHandle(&g_uartTxDmaHandle, DMA0, UART_TX_DMA_CHANNEL);

	/* Create UART DMA handle. */
	UART_TransferCreateHandleDMA(UART_CHANNEL, &g_uartDmaHandle, UART_UserCallback, NULL, &g_uartTxDmaHandle,
								 NULL);

	/* Send g_tipString out. */
	xfer.data = g_tipString;
	xfer.dataSize = sizeof(g_tipString) - 1;
	txOnGoing = true;
	UART_TransferSendDMA(UART_CHANNEL, &g_uartDmaHandle, &xfer);

	/* Wait send finished */
	while (txOnGoing)
	{
	}
}

/** \brief envía datos por puerto serie vía DMA
 **
 ** \param[inout] pBuf buffer a donde estan los datos a enviar
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes enviados
 **/
int32_t uart_drv_envDatos(uint8_t *pBuf, int32_t size)
{

}

int32_t uart_drv_recDatos(uint8_t *pBuf, int32_t size){
	// TODO: Implementar
}
