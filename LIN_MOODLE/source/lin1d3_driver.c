/*
 * lin1d3_driver.c
 *
 *  Created on: Sep 14, 2018
 *      Author: Nico
 */
#include "lin1d3_driver.h"
#include <string.h>
#include <fsl_debug_console.h>

#define BIT_MASK			(0x01)
#define master_stack_size_d	(256)
#define master_task_priority (configMAX_PRIORITIES - 1)
#define master_queue_size_d	(8)

#define slave_stack_size_d	(256)
#define slave_task_priority (configMAX_PRIORITIES - 1)

#define size_of_uart_buffer	(10)

#define size_of_lin_header_d (3)

/*Static function prototypes */
static void master_task(void *pvParameters);
static void slave_task(void *pvParameters);



/******************************************************************************
 * Public functions
 *
 *****************************************************************************/

static uint8_t lin1d3_calculate_checksum(uint8_t* data, uint8_t message_size)
{
	/** Variable for the bit in 1 count*/
	uint16_t checksum = 0;
	/** Counter for the bytes sent*/
	uint8_t data_count;
	/** Counter for the bits in the message*/
	uint8_t bit_count;
	/** Variable to return the checksum*/
	uint8_t checkS255;
	/** Variable to manage the message byte by byte*/
	uint8_t byte;

	/** For each byte in the message*/
	for(data_count = 0 ; data_count < message_size ; data_count ++)
	{
		/** Gets the value from the pointer*/
		byte = *(data);
		/** For each bit in the byte*/
		for(bit_count = 0 ; bit_count < 8 ; bit_count ++)
		{
			/** Gets the LSB from the byte, and adds it to the
			 	 bit counter*/
			checksum += ((uint16_t)byte) & BIT_MASK;
			/** Shifts the data 1 bit to the right*/
			byte >>= 1;
		}

		/** Moves to the next byte*/
		data ++;
	}

	/** Gets the firt byte of the count*/
	checkS255 = ((uint8_t)(checksum));
	/** NOT to the bits counted*/
	checkS255 = ~checkS255;

	/** Returns the checksum*/
	return checkS255;
}

/*
 * Init a LIN node
 * */
lin1d3_handle_t* lin1d3_InitNode(lin1d3_nodeConfig_t config)
{
	lin1d3_handle_t* handle = NULL;
	static uint8_t node_idx = 0;
	char master_task_name[] = "linMaster0";
	char slave_task_name[] = "linSlave0";
	/* Check init parameters */
	if(config.type >= lin1d3_max_nodeType) {
		return NULL;
	}

	/* Create the handle structure and */
	handle = (lin1d3_handle_t*)pvPortMalloc(sizeof(lin1d3_handle_t));
	if(handle ==  NULL) {
		/* Failed to allocate memory for the node handle */
		return NULL;
	}
	/* Init the handle structure with 0s */
	memset(handle, 0, sizeof(lin1d3_handle_t));
	/* Copy the config */
	memcpy(&(handle->config), &config, sizeof(lin1d3_nodeConfig_t));

	/* Create the Node Task */
	if(lin1d3_master_nodeType == config.type) {
		/* Create a queue for User message requests */
		handle->node_queue = xQueueCreate( master_queue_size_d, sizeof(uint8_t));
		if(handle->node_queue == NULL){
			vPortFree(handle);
			return NULL;
		}
		/* Create a task for the node */
		master_task_name[strlen(master_task_name)-1] += node_idx++;
		if (xTaskCreate(master_task, master_task_name, master_stack_size_d, handle, master_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}
	else if(lin1d3_slave_nodeType == config.type) {
		/* Create a task for the node */
		slave_task_name[strlen(slave_task_name)-1] += node_idx++;
		if (xTaskCreate(slave_task, slave_task_name, slave_stack_size_d, handle, slave_task_priority, &(handle->task_handle)) != pdPASS) {
			vPortFree(handle);
			return NULL;
		}
	}

	return handle;
}

/*
 * Send a message frame from a LIN Master node
 * */
uint32_t lin1d3_masterSendMessage(lin1d3_handle_t* handle, uint8_t ID)
{
	if(handle !=  NULL) {
		/* Put the requested ID on the master queue */
		xQueueSend( handle->node_queue, &ID, ( TickType_t ) 0 );
	}
	return 0;
}

/******************************************************************************
 * Static functions
 *
 *****************************************************************************/
static void master_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[] = {0x00, 0x55, 0x00};
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;
	size_t n;
	uint8_t  msg_idx;
	uint8_t checksum = 0x00;
	uint8_t LC = 0x00;
	uint8_t ID0 = 0x00;
	uint8_t ID1 = 0x00;
	uint8_t ID2 = 0x00;
	uint8_t ID3 = 0x00;
	uint8_t ID4 = 0x00;
	uint8_t ID5 = 0x00;
	uint8_t P0 = 0x00;
	uint8_t P1 = 0x00;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		vTaskSuspend(NULL);
	}

    if (0 > UART_RTOS_Init(&(handle->uart_rtos_handle), &(handle->uart_handle), &(handle->uart_config)))
    {
        vTaskSuspend(NULL);
    }

    while(1) {
    	/* Wait for messages on the Queue */
        if(xQueueReceive(handle->node_queue, &ID, portMAX_DELAY)){

        	msg_idx = 0;
        	/*Look for the ID in the message table */
        	while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
        		if(handle->config.messageTable[msg_idx].ID == ID) {
        			break;
        		}
        		msg_idx++;
        	}

        	/* If the message ID was not found then ignore it */
        	if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;

        	/**Get ID Length Control*/
			LC = (ID&0x03);

			/**Get ID ID bits*/
			ID0 = ((ID&0x08)>>3);
			ID1 = ((ID&0x04)>>2);
			ID2 = ((ID&0x02)>>1);
			ID3 = (ID&0x01);

			/** Gets the length bits*/
			ID4 = (LC&0x01);
			ID5 = ((LC&0x02)>>1);

			/** Calculates the parity bits*/
			P0 = ID0^ID1^ID2^ID4;
			P1 = !(ID1^ID3^ID4^ID5);

			/* Put the ID into the header */
			lin1p3_header[2] = ((ID<<4)|(ID4<<3)|(ID5<<2)|(P0<<1)|(P1));

        	memset(lin1p3_message, 0, size_of_uart_buffer);
        	/* Calc the message size */
        	switch(ID&0x03) {
        		case 0x00: message_size = 2;
        		break;
        		case 0x01: message_size = 2;
        		break;
        		case 0x02: message_size = 4;
        		break;
        		case 0x03: message_size = 8;
        		break;
        	}
        	message_size+=1;
        	/* Send the header */
        	UART_RTOS_Send(&(handle->uart_rtos_handle), (uint8_t *)lin1p3_header, 3);
        	/* Wait for the response */
        	UART_RTOS_Receive(&(handle->uart_rtos_handle), lin1p3_message, message_size, &n);

        	/* Check the checksum */
        	checksum = lin1d3_calculate_checksum(lin1p3_message, (message_size - 1));
        	if(checksum != lin1p3_message[message_size - 1])
        	{
        		/**Error at checksum*/
        		continue;
        	}

        	/* Call the message callback */
        	handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);
        }
    }
}

static void slave_task(void *pvParameters)
{
	lin1d3_handle_t* handle = (lin1d3_handle_t*)pvParameters;
	uint8_t  ID;
	uint8_t  lin1p3_header[size_of_lin_header_d];
	uint8_t  lin1p3_message[size_of_uart_buffer];
	uint8_t  message_size = 0;
	size_t n;
	uint8_t  msg_idx;
	uint8_t checksum = 0x00;
	uint8_t LC = 0x00;
	uint8_t ID0 = 0x00;
	uint8_t ID1 = 0x00;
	uint8_t ID2 = 0x00;
	uint8_t ID3 = 0x00;
	uint8_t ID4 = 0x00;
	uint8_t ID5 = 0x00;
	uint8_t P0 = 0x00;
	uint8_t P1 = 0x00;
	uint8_t P0_rec = 0x00;
	uint8_t P1_rec = 0x00;

	if(handle == NULL) {
		vTaskSuspend(NULL);
	}

	/* Init/Configure the UART */
	handle->uart_config.base = handle->config.uartBase;
	handle->uart_config.srcclk = handle->config.srcclk;
	handle->uart_config.baudrate = handle->config.bitrate;
	handle->uart_config.parity = kUART_ParityDisabled;
	handle->uart_config.stopbits = kUART_OneStopBit;
	handle->uart_config.buffer = pvPortMalloc(size_of_uart_buffer);
	handle->uart_config.buffer_size = size_of_uart_buffer;
	if(handle->uart_config.buffer == NULL){
		vTaskSuspend(NULL);
	}

    if (0 > UART_RTOS_Init(&(handle->uart_rtos_handle), &(handle->uart_handle), &(handle->uart_config)))
    {
        vTaskSuspend(NULL);
    }

    while(1) {
    	/* Init the message header buffer */

    	memset(lin1p3_header, 0, size_of_lin_header_d);
    	/* Wait for header on the UART */
    	UART_RTOS_Receive(&(handle->uart_rtos_handle), lin1p3_header, size_of_lin_header_d, &n);
    	/* Check header */
    	if((lin1p3_header[0] != 0x00) &&
    	   (lin1p3_header[1] != 0x55)) {
    		/* Header is not correct we are ignoring the header */
    		continue;
    	}
    	/* Get the message ID */
		ID = (lin1p3_header[1] & 0xFC)>>4;

		/**Get Parity Bits*/
		P0_rec = (lin1p3_header[1] & 0x02)>>1;
		P1_rec = (lin1p3_header[1] & 0x01);

		/**Get ID Length Control*/
		LC = (ID&0x03);

		/**Get ID ID bits*/
		ID0 = ((ID&0x08)>>3);
		ID1 = ((ID&0x04)>>2);
		ID2 = ((ID&0x02)>>1);
		ID3 = (ID&0x01);

		/** Gets the length bits*/
		ID4 = (LC&0x01);
		ID5 = ((LC&0x02)>>1);

		/** Calculates the parity bits*/
		P0 = ID0^ID1^ID2^ID4;
		P1 = !(ID1^ID3^ID4^ID5);

		/** If the parity calculated is different from the parity
			 received*/
		if((P0 != P0_rec) || (P1 != P1_rec))
		{
			/** Restarts the task*/
			continue;
		}

    	/* If the header is correct, check if the message is in the table */
    	msg_idx = 0;
    	/*Look for the ID in the message table */
    	while(msg_idx < lin1d3_max_supported_messages_per_node_cfg_d) {
    		if(handle->config.messageTable[msg_idx].ID == ID) {
    			break;
    		}
    		msg_idx++;
    	}
    	/* If the message ID was not found then ignore it */
    	if(msg_idx == lin1d3_max_supported_messages_per_node_cfg_d) continue;
    	/* Init the message transmit buffer */
    	memset(lin1p3_message, 0, size_of_uart_buffer);
    	/*If the message is in the table call the message callback */
    	handle->config.messageTable[msg_idx].handler((void*)lin1p3_message);

    	/* Calc the message size */
    	switch(ID&0x03) {
    		case 0x00: message_size = 2;
    		break;
    		case 0x01: message_size = 2;
    		break;
    		case 0x02: message_size = 4;
    		break;
    		case 0x03: message_size = 8;
    		break;
    	}
    	/**Calculates the checksum*/
		checksum = lin1d3_calculate_checksum(lin1p3_message, message_size);
		lin1p3_message[message_size] = checksum;
    	message_size+=1;
    	/* Send the message data */
    	UART_RTOS_Send(&(handle->uart_rtos_handle), (uint8_t *)lin1p3_message, message_size);
    }
}
