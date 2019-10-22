/**
* \file  wsndemo.c
*
* \brief WSNDemo application implementation
*
* Copyright (c) 2018 - 2019 Microchip Technology Inc. and its subsidiaries. 
*
* \asf_license_start
*
* \page License
*
* Subject to your compliance with these terms, you may use Microchip
* software and any derivatives exclusively with Microchip products. 
* It is your responsibility to comply with third party license terms applicable 
* to your use of third party software (including open source software) that 
* may accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, 
* WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, 
* INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, 
* AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE 
* LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL 
* LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE 
* SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT 
* ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY 
* RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
* \asf_license_stop
*
*/
/*
* Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "phy.h"
#include "sysTimer.h"
#include "commands.h"
#include "miwi_api.h"
#include "otau.h"

#if defined(PAN_COORDINATOR)
#include "sio2host.h"
#endif

#if defined(ENDDEVICE)
#include "sleep_mgr.h"
#endif

#if SAMD || SAMR21 || SAML21 || SAMR30
#include "system.h"
#else
#include "sysclk.h"
#if (LED_COUNT > 0)
#include "led.h"
#endif
#endif

#include "asf.h"
#include "wsndemo.h"
#if defined(ENABLE_NETWORK_FREEZER)
#include "pdsMemIds.h"
#include "pdsDataServer.h"
#include "wlPdsTaskManager.h"
#endif

#include "streetlight/structures.h"
/*****************************************************************************
*****************************************************************************/
#define APP_SCAN_DURATION 10
#define APP_CAPTION_SIZE  (sizeof(APP_CAPTION) - 1 + SHORT_ADDRESS_CAPTION_SIZE)
#define APP_SERIAL_CMD_SIZE		4

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static AppState_t state_returned = APP_STATE_NO_DEFINED;

#if defined(COORDINATOR) || defined (ENDDEVICE)
static SYS_Timer_t appNetworkStatusTimer;
static bool appNetworkStatus;
#endif

#if defined(PAN_COORDINATOR)
static uint8_t rx_data[APP_RX_BUF_SIZE];
#endif

static AppPacket_t appMsg;
SYS_Timer_t appDataSendingTimer;
static uint8_t wsnmsghandle;

static uint8_t buffer_cmd[80];

static uint8_t cmd[APP_SERIAL_CMD_SIZE];
static uint8_t index_serial = 0x0;
static uint16_t unique_id = 0x0;
static uint16_t dst_addr = 0xFFFF;
static uint8_t flag_busy = 0x0;

static uint8_t try_again = 0x0;

static void Connection_Confirm(miwi_status_t status);
#ifndef PAN_COORDINATOR
void searchConfim(uint8_t foundScanResults, void* ScanResults);
void appLinkFailureCallback(void);
#endif

#if defined(ENABLE_NETWORK_FREEZER)
static void ReconnectionIndication (miwi_status_t status);
#endif
/*- Implementations --------------------------------------------------------*/

#if defined(PAN_COORDINATOR)

static AppState_t UartBytesReceived(uint16_t bytes, uint8_t *byte)
{
	state_returned = APP_STATE_NO_DEFINED;

	for (uint16_t i = 0; i < bytes; i++)
	{
		cmd[index_serial++] = byte[i];
	}
	
	/* Verify if buffer reaches the CMD size (8) */
	if (index_serial >= APP_SERIAL_CMD_SIZE)
	{
		uint8_t indice = 0;
		/* Looking for delimiter @ */
		for (indice = 0; indice < index_serial; indice++)
		{
			if (cmd[indice] == '@')
			{
				break;
			}
		}
		#ifdef DEBUG
		printf("[cmd] %s\n\r", cmd);
		#endif
		/* Getting unique id */
		unique_id = atoi(&cmd[indice + 1]);
		
		if ((unique_id / 10) == 99)
		{
			memset(&buffer_cmd, (unique_id % 10), sizeof(buffer_cmd));
			dst_addr = 0xFFFF;
			unique_id = 99;
			if (flag_busy == 0x0)
			{
				state_returned = APP_STATE_SEND_CMD;
			}
		}
		else
		{
			buffer_cmd[(unique_id / 10)] = unique_id % 10;
			dst_addr = 0xFFFF;
			unique_id = unique_id / 10;
			if (flag_busy == 0x0)
			{
				state_returned = APP_STATE_SEND_CMD;
			}
		}
		
		#ifdef DEBUG
		printf("Sending CMD to: %d  -  %d\n\r", (unique_id / 10), buffer_cmd[(unique_id / 10)]);
		#endif

		index_serial = 0;
		
		memset(&cmd, 0x0, 12);

	}
	
	return state_returned;
}

static void appUartSendMessage(uint8_t *data, uint8_t size)
{
	uint8_t cs = 0;

	sio2host_putchar(0x10);
	sio2host_putchar(0x02);

	for (uint8_t i = 0; i < size; i++) {
		if (data[i] == 0x10) {
			sio2host_putchar(0x10);
			cs += 0x10;
		}

		sio2host_putchar(data[i]);
		cs += data[i];
	}

	sio2host_putchar(0x10);
	sio2host_putchar(0x03);
	cs += 0x10 + 0x02 + 0x10 + 0x03;

	sio2host_putchar(cs);
}

#endif

static void appDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
	#if (LED_COUNT > 0)
	LED_Off(LED_DATA);
	#endif

	if (SUCCESS == status)
	{
		#ifdef DEBUG
		printf("Enviado com sucesso!\r\n");
		#endif
	}
	else
	{
		try_again++;

		#ifdef DEBUG
		printf("failed to send cmd! [%d]\n\r", try_again);
		#endif
		if (try_again > 2)
		{
			try_again = 0;
			appState = APP_STATE_WAIT_CONF;
			flag_busy = 0x0;
		}
		else
		{
			appState = APP_STATE_SEND_CMD;
		}
	}
	if (APP_STATE_WAIT_CONF == appState)
	{
		appState = APP_STATE_WAIT;
		flag_busy = 0x0;
	}
}

static void app_send_cmd(void)
{
	flag_busy = 0x01;
	appMsg.packet_type = PACKET_COMMAND;
	if (unique_id == 99)
	{
		appMsg.light = buffer_cmd[1];
	}
	else
	{
		appMsg.light = buffer_cmd[unique_id];
	}

	appMsg.unique_id = unique_id;
	appMsg.refresh_rate = 30;

	#ifdef DEBUG
	printf("Sending CMD to: %d [%d] - %d\n\r", dst_addr, unique_id, appMsg.light);
	#endif
	
	if (MiApp_SendData(2, (uint8_t *) &dst_addr, sizeof(appMsg), (uint8_t *) &appMsg, wsnmsghandle, true, appDataConf))
	{
		++wsnmsghandle;
		appState = APP_STATE_WAIT_CONF;
	}
}


static void appDataInd(RECEIVED_MESH_MESSAGE *ind)
{
	AppPacket_t *msg = (AppPacket_t *) ind->payload;
	
	switch (msg->packet_type)
	{
		case PACKET_DATA:
		{
			printf("#%d;%d;%d;%d\n", msg->unique_id, msg->light, msg->temp, msg->hum);
		}
	}

	LED_Toggle(LED_DATA);
}


static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
	if ((APP_STATE_WAIT_SEND_TIMER == appState) || (APP_STATE_PREPARE_TO_SLEEP == appState)) 
	{
		appState = APP_STATE_SEND;
	}
	else
	{
		SYS_TimerStart(&appDataSendingTimer);
	}

	(void)timer;
}


static void appSendData(void)
{
    uint16_t shortAddressLocal = 0xFFFF;
    uint16_t shortAddressPanId = 0xFFFF;
    uint16_t dstAddr = 0; /* PAN Coordinator Address */

	if (MiApp_SendData(2, (uint8_t *)&dstAddr, sizeof(appMsg), (uint8_t *)&appMsg, wsnmsghandle, true, appDataConf))
	{
		++wsnmsghandle;
		appState = APP_STATE_WAIT_CONF;
	}
	else
	{
		appState = APP_STATE_SENDING_DONE;
	}
}


static void appInit(void)
{
	appDataSendingTimer.interval = APP_SENDING_INTERVAL;
	appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appDataSendingTimer.handler = appDataSendingTimerHandler;

	MiApp_SubscribeDataIndicationCallback(appDataInd);
#ifndef PAN_COORDINATOR
	MiApp_SubscribeLinkFailureCallback(appLinkFailureCallback);
#endif

#if defined(ENABLE_NETWORK_FREEZER)
    if (appState == APP_STATE_RECONNECT_SUCCESS)
    {
         appState = APP_STATE_SEND;
    }
    else
#endif
    {
#if defined(PAN_COORDINATOR)
        appState = APP_STATE_START_NETWORK;
#else
        appState = APP_STATE_CONNECT_NETWORK;
#endif 
    }
}

#if defined(ENABLE_NETWORK_FREEZER)
static void ReconnectionIndication (miwi_status_t status)
{
	if(SUCCESS == status)
	{
		appState = APP_STATE_RECONNECT_SUCCESS;
	}
	else
	{
		appState = APP_STATE_RECONNECT_FAILURE;
	}
}
#endif


static void APP_TaskHandler(void)
{
	switch (appState) {
	case APP_STATE_INITIAL:
	case APP_STATE_RECONNECT_SUCCESS:
	case APP_STATE_RECONNECT_FAILURE:
	{
		appInit();
	}
	break;

	case APP_STATE_START_NETWORK:
	{
		MiApp_StartConnection(START_CONN_DIRECT, APP_SCAN_DURATION, CHANNEL_MAP, Connection_Confirm);
		appState = APP_STATE_SEND;
	}
	break;

	case APP_STATE_SEND:
	{
		appSendData();
	}
	break;
	
	case APP_STATE_SEND_CMD:
	{
		app_send_cmd();
	}
	break;

	case APP_STATE_SENDING_DONE:
	{
#if defined(ENABLE_SLEEP_FEATURE) && defined(ENDDEVICE) && (CAPABILITY_INFO == CAPABILITY_INFO_ED)
		SYS_TimerStart(&appDataSendingTimer);
		appState = APP_STATE_PREPARE_TO_SLEEP;
#else
		SYS_TimerStart(&appDataSendingTimer);
		appState = APP_STATE_WAIT_SEND_TIMER;
#endif
	}
	break;

	default:
		break;
	}

#if defined(ENABLE_NETWORK_FREEZER)
	/* Read the button level */
	if (port_pin_get_input_level(BUTTON_0_PIN) == BUTTON_0_ACTIVE)
	{
        MiApp_ResetToFactoryNew();
	}
#endif

	uint16_t bytes;
	if ((bytes = sio2host_rx(rx_data, APP_RX_BUF_SIZE)) > 0) {
		UartBytesReceived(bytes, (uint8_t *)&rx_data);
		
		if (state_returned != APP_STATE_NO_DEFINED)
		{
			appState = state_returned;
		}
	}

}


void wsndemo_init(void)
{
	uint8_t i;
	bool invalidIEEEAddrFlag = false;
	uint64_t invalidIEEEAddr;
	
	memset(&buffer_cmd, 0x02, sizeof(buffer_cmd));

#if defined(ENABLE_NETWORK_FREEZER)
    MiApp_SubscribeReConnectionCallback((ReconnectionCallback_t)ReconnectionIndication );
#endif

	/* Initialize the Protocol */
	if (MiApp_ProtocolInit(&defaultParamsRomOrRam, &defaultParamsRamOnly) == RECONNECTION_IN_PROGRESS)
	{
		appState = APP_STATE_WAIT_FOR_RECONNECT_CALLBACK;
	}

	/* Check if a valid IEEE address is available.
		0x0000000000000000 and 0xFFFFFFFFFFFFFFFF is persumed to be invalid */
	/* Check if IEEE address is 0x0000000000000000 */
	memset((uint8_t *)&invalidIEEEAddr, 0x00, LONG_ADDR_LEN);
	if (0 == memcmp((uint8_t *)&invalidIEEEAddr, (uint8_t *)&myLongAddress, LONG_ADDR_LEN))
	{
		invalidIEEEAddrFlag = true;
	}

	/* Check if IEEE address is 0xFFFFFFFFFFFFFFFF */
	memset((uint8_t *)&invalidIEEEAddr, 0xFF, LONG_ADDR_LEN);
	if (0 == memcmp((uint8_t *)&invalidIEEEAddr, (uint8_t *)&myLongAddress, LONG_ADDR_LEN))
	{
		invalidIEEEAddrFlag = true;
	}
	
	if (invalidIEEEAddrFlag)
	{
		/*
			* In case no valid IEEE address is available, a random
			* IEEE address will be generated to be able to run the
			* applications for demonstration purposes.
			* In production code this can be omitted.
			*/
		uint8_t* peui64 = (uint8_t *)&myLongAddress;
		for(i = 0; i<MY_ADDRESS_LENGTH; i++)
		{
			*peui64++ = (uint8_t)rand();
		}
	}
	PHY_SetIEEEAddr((uint8_t *)&myLongAddress);

#if defined(PAN_COORDINATOR)
	sio2host_init();
#endif
}


static void Connection_Confirm(miwi_status_t status)
{
	if (SUCCESS == status)
	{
        appState = APP_STATE_WAIT_CONF;
		#ifdef DEBUG
			printf("Connected!\r\n");
#endif
	}
	else
	{
#if defined(PAN_COORDINATOR)
		appState = APP_STATE_START_NETWORK;
#else
        appState = APP_STATE_CONNECT_NETWORK;
#endif
	}
}

/**
 * Task of the WSNDemo application
 * This task should be called in a while(1)
 */
void wsndemo_task(void)
{
	MeshTasks();
#if defined(ENABLE_NETWORK_FREEZER)
#if PDS_ENABLE_WEAR_LEVELING
    PDS_TaskHandler();
#endif
#endif
	APP_TaskHandler();
}

#ifndef PAN_COORDINATOR
void appLinkFailureCallback(void)
{
	/* On link failure initiate search to establish connection */
	appState = APP_STATE_CONNECT_NETWORK;
	SYS_TimerStop(&appDataSendingTimer);
}
#endif
