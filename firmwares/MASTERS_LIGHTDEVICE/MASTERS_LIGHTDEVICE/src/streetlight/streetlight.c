#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "phy.h"
#include "sysTimer.h"
#include "commands.h"
#include "miwi_api.h"

#include "sio2host.h"

#include "system.h"
#include "asf.h"
#include "streetlight/streetlight.h"
#include "streetlight/structures.h"

#if defined(ENABLE_NETWORK_FREEZER)
#include "pdsMemIds.h"
#include "pdsDataServer.h"
#include "wlPdsTaskManager.h"
#endif

static AppState_t appState = APP_STATE_INITIAL;

static AppPacket_t appMsg;
SYS_Timer_t appDataSendingTimer;
static uint8_t wsnmsghandle;

static void Connection_Confirm(miwi_status_t status);
void searchConfim(uint8_t foundScanResults, void* ScanResults);
void appLinkFailureCallback(void);

#if defined(ENABLE_NETWORK_FREEZER)
static void ReconnectionIndication (miwi_status_t status);
#endif

static void appDataInd(RECEIVED_MESH_MESSAGE *ind)
{
	AppPacket_t *msg = (AppPacket_t *)ind->payload;
	
	// Received data!
	// Your code Here.
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

static void appDataConf(uint8_t msgConfHandle, miwi_status_t status, uint8_t* msgPointer)
{
	// Your code here!
	if (SUCCESS == status)
	{

	}
	else
	{

	}

	if (APP_STATE_WAIT_CONF == appState)
	{
		appState = APP_STATE_SENDING_DONE;
	}
}

static void appSendData(void)
{
	uint16_t dstAddr = 0; /* PAN Coordinator Address */
	// Your code here
	// Mount packet to send
	appMsg.packet_type = PACKET_DATA;

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
	MiApp_SubscribeLinkFailureCallback(appLinkFailureCallback);

	if (appState == APP_STATE_RECONNECT_SUCCESS)
	{
		appState = APP_STATE_SEND;
	}
	else
	{
		appState = APP_STATE_CONNECT_NETWORK;
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

		case APP_STATE_CONNECT_NETWORK:
		{
			MiApp_SearchConnection(5, CHANNEL_MAP, searchConfim);
			appState = APP_STATE_CONNECTING_NETWORK;
		}
		break;

		case APP_STATE_SEND:
		{
			appSendData();
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
}

void streetlight_init(void)
{
	uint8_t i;
	bool invalidIEEEAddrFlag = false;
	uint64_t invalidIEEEAddr;

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

	sio2host_init();
}

void searchConfim(uint8_t foundScanResults, void* ScanResults)
{
	searchConf_t* searchConfRes = (searchConf_t *)ScanResults;
	uint8_t selectedParentIndex = 0xFF;
	if (foundScanResults)
	{
		for (uint8_t loopindex = 0; loopindex < foundScanResults; loopindex++)
		{
			if (searchConfRes->beaconList[loopindex].connectionPermit)
			{
				#if defined(ENDDEVICE)
				/* Select the parent which has the high end device capacity (holding less number of end devices) */
				if (loopindex == 0)
				{
					selectedParentIndex = 0;
				}
				#if (CAPABILITY_INFO == CAPABILITY_INFO_ED)
				else if (searchConfRes->beaconList[loopindex].sleepEnddeviceCapacity > searchConfRes->beaconList[selectedParentIndex].sleepEnddeviceCapacity)
				#elif (CAPABILITY_INFO == CAPABILITY_INFO_ED_RXON)
				else if (searchConfRes->beaconList[loopindex].enddeviceCapacity > searchConfRes->beaconList[selectedParentIndex].enddeviceCapacity)
				#endif
				#endif
				{
					selectedParentIndex = loopindex;
				}
			}
		}
		
		if (selectedParentIndex != 0xFF)
		{
			MiApp_EstablishConnection(searchConfRes->beaconList[selectedParentIndex].logicalChannel,
			SHORT_ADDR_LEN, (uint8_t*)&searchConfRes->beaconList[selectedParentIndex].shortAddress, CAPABILITY_INFO, Connection_Confirm);
			return;
		}
		/* Initiate the search again since no connection permit found to join */
		appState = APP_STATE_CONNECT_NETWORK;
	}
	else
	{
		/* Initiate the search again since no beacon */
		appState = APP_STATE_CONNECT_NETWORK;
	}
}


static void Connection_Confirm(miwi_status_t status)
{
	if (SUCCESS == status)
	{
        appState = APP_STATE_SEND;
	}
	else
	{
        appState = APP_STATE_CONNECT_NETWORK;
	}
}

void streetlight_task(void)
{
	MeshTasks();
#if defined(ENABLE_NETWORK_FREEZER)
#if PDS_ENABLE_WEAR_LEVELING
    PDS_TaskHandler();
#endif
#endif
	APP_TaskHandler();
}

void appLinkFailureCallback(void)
{
	/* On link failure initiate search to establish connection */
	appState = APP_STATE_CONNECT_NETWORK;
	SYS_TimerStop(&appDataSendingTimer);
}
