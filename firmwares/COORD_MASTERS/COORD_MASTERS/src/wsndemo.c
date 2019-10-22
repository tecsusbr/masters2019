#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "phy.h"
#include "sysTimer.h"
#include "miwi_api.h"

#include "sio2host.h"

#include "system.h"

#include "asf.h"

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
#define MAX_DEVICE_BUFFER		30

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
AppState_t state_returned = APP_STATE_NO_DEFINED;

static SYS_Timer_t appNetworkStatusTimer;
static bool appNetworkStatus;
static uint8_t rx_data[APP_RX_BUF_SIZE];


static AppPacket_t appMsg;
SYS_Timer_t appDataSendingTimer;
static uint8_t wsnmsghandle;

static uint8_t commands[30];

static void Connection_Confirm(miwi_status_t status);

uint16_t dst_addr = 0xFFFF;
uint16_t unique_id = 0x0;



uint16_t buffer[2][MAX_DEVICE_BUFFER];

uint8_t flag_busy = 0x0;
uint8_t verify_cycle = 0x0;

static char cmd[12];
static uint8_t index_serial = 0;

static uint8_t try_again = 0;

bool reconnectStatus = false;
static void ReconnectionIndication (miwi_status_t status);
/*- Implementations --------------------------------------------------------*/

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
			memset(&commands, (unique_id % 10), 30);
			dst_addr = 0xFFFF;
			unique_id = 99;
			if (flag_busy == 0x0)
			{
				state_returned = APP_STATE_SEND_CMD;
			}
		}
		else
		{
			commands[(unique_id / 10)] = unique_id % 10;
		}
		
#ifdef DEBUG
		printf("Sending CMD to: %d  -  %d\n\r", (unique_id / 10), commands[(unique_id / 10)]);
#endif

		index_serial = 0;
		
		memset(&cmd, 0x0, 12);

	}
	
	return state_returned;
}

/*****************************************************************************
*****************************************************************************/
static void appDataInd(RECEIVED_MESH_MESSAGE *ind)
{
	AppPacket_t *msg = (AppPacket_t *)ind->payload;

	#ifdef DEBUG
		printf("Received data! %d\r\n", ind->sourceAddress);
	#endif

	/* Rec Message */
	switch (msg->packet_type)
	{	
	case PACKET_DATA:
		if (flag_busy == 0x0)
		{
			flag_busy = 0x1;
			if (commands[msg->unique_id] != 2 && commands[msg->unique_id] != msg->light)
			{
				msg->light = commands[msg->unique_id];
			}
			printf("#%d;%d;%d;%d\n", msg->unique_id, msg->light, msg->temp, msg->hum);
		
			/* Received a CMD Request */
			dst_addr = ind->sourceAddress;
			unique_id = msg->unique_id;
		
			appState = APP_STATE_SEND_CMD;
		}
		else
		{
			uint8_t i = 0;
			while (i < MAX_DEVICE_BUFFER && buffer[0][i] != 0x0)
			{
				i++;
			}

			buffer[0][i] = ind->sourceAddress;
			buffer[1][i] = msg->unique_id;
		}
		break;
	
	default:
		break;
	}

}

/*****************************************************************************
*****************************************************************************/
static void appDataSendingTimerHandler(SYS_Timer_t *timer)
{
	if (APP_STATE_WAIT_SEND_TIMER == appState)
	{
		appState = APP_STATE_SEND;
	}
	else
	{
		SYS_TimerStart(&appDataSendingTimer);
	}

	(void)timer;
}
/*****************************************************************************
*****************************************************************************/
static void appNetworkStatusTimerHandler(SYS_Timer_t *timer)
{
	(void)timer;
}


/*****************************************************************************
*****************************************************************************/
static void appDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
	if (SUCCESS == status)
	{
		if (!appNetworkStatus)
		{
			SYS_TimerStop(&appNetworkStatusTimer);
			appNetworkStatus = true;
		}
	}
	else
	{
		if (appNetworkStatus)
		{
			SYS_TimerStart(&appNetworkStatusTimer);
			appNetworkStatus = false;
		}
	}
	if (APP_STATE_WAIT_CONF == appState)
	{
		appState = APP_STATE_SENDING_DONE;
	}
}

static void app_cmd_conf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
	if (status == SUCCESS)
	{
		/* SUCCESS, Do nothing */
		flag_busy = 0x0;
#ifdef DEBUG
		printf("Success to send cmd!\n\r");
#endif
		appState = APP_STATE_VERIFY_NEXT_CMD;
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
}
/*****************************************************************************
*****************************************************************************/
static void appSendData(void)
{
    uint16_t shortAddressLocal = 0xFFFF;
    uint16_t dstAddr = 0; /* PAN Coordinator Address */


	/* Get Short address */
	MiApp_Get(SHORT_ADDRESS, (uint8_t *)&shortAddressLocal);
    appMsg.short_addr = shortAddressLocal;

    /* Get Next Hop Short address to reach PAN Coordinator*/
	appMsg.parent_addr = MiApp_MeshGetNextHopAddr(PAN_COORDINATOR_SHORT_ADDRESS);


	SYS_TimerStart(&appDataSendingTimer);
	appState = APP_STATE_WAIT_SEND_TIMER;

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

static void app_send_cmd(void)
{
	appMsg.packet_type = PACKET_COMMAND;
	if (unique_id == 99)
	{
		appMsg.light = commands[1];
	}
	else
	{
		appMsg.light = commands[unique_id];
	}

	appMsg.unique_id = unique_id;
	appMsg.refresh_rate = 30;

#ifdef DEBUG
	printf("Sending CMD to: %d [%d] - %d\n\r", dst_addr, unique_id, appMsg.light);
#endif
	
	if (MiApp_SendData(2, (uint8_t *) &dst_addr, sizeof(appMsg), (uint8_t *) &appMsg, wsnmsghandle, true, app_cmd_conf))
	{
		++wsnmsghandle;
		appState = APP_STATE_WAIT_CONF;
	}
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
#ifdef DEBUG
	printf("App Init\n\r");
#endif

	MiApp_SubscribeDataIndicationCallback(appDataInd);

	appDataSendingTimer.interval = APP_SENDING_INTERVAL;
	appDataSendingTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appDataSendingTimer.handler = appDataSendingTimerHandler;

	appNetworkStatus = false;
	appNetworkStatusTimer.interval = APP_NWKSTATUS_INTERVAL;
	appNetworkStatusTimer.mode = SYS_TIMER_PERIODIC_MODE;
	appNetworkStatusTimer.handler = appNetworkStatusTimerHandler;
	SYS_TimerStart(&appNetworkStatusTimer);

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

static void ReconnectionIndication (miwi_status_t status)
{
	if(SUCCESS == status)
	{
		reconnectStatus = true;
	}
	else
	{
        reconnectStatus = false;
	}
}



/*************************************************************************//**
*****************************************************************************/
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
		MiApp_StartConnection(START_CONN_DIRECT, 5, CHANNEL_MAP, Connection_Confirm);
		appState = APP_STATE_WAIT_CONF;
	}
	break;
	
	case APP_STATE_VERIFY_NEXT_CMD:
	{
		if (verify_cycle < 250)
		{
			verify_cycle++;
		}
		else
		{
			verify_cycle = 0x0;
			uint8_t i = 0;
			while (i < MAX_DEVICE_BUFFER && buffer[0][i] == 0x0)
			{
				i++;
			}
			
			if (i == 10)
			{
				appState = APP_STATE_WAIT_CONF;
			}
			else
			{
				unique_id = buffer[1][i];
				dst_addr = buffer[0][i];
				appState = APP_STATE_VERIFY_NEXT_CMD;
			}
			
		}
	}

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
		SYS_TimerStart(&appDataSendingTimer);
		appState = APP_STATE_WAIT_SEND_TIMER;
	}
	break;

	default:
		break;
	}

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
	
	memset(&cmd, 0x0, 12);
	memset(&commands, 0x02, sizeof(commands));
	memset(&buffer, 0x00, sizeof(buffer));

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

/**
 * Connection confirmation
 */
static void Connection_Confirm(miwi_status_t status)
{
	if (SUCCESS == status)
	{
		#ifdef DEBUG
		printf("Connected!\n\r");
		#endif
		uint8_t channel;
		MiApp_Get(CHANNEL, &channel);
		printf("Connected to channel: %d\n\r", channel);
        appState = APP_STATE_WAIT_CONF;
	}
	else
	{
		#ifdef DEBUG
		printf("not connected...\n\r");
		#endif
		appState = APP_STATE_START_NETWORK;
	}
}

/**
 * Task of the WSNDemo application
 * This task should be called in a while(1)
 */
void wsndemo_task(void)
{
	MeshTasks();
    PDS_TaskHandler();
	APP_TaskHandler();
}
