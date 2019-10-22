#include "asf.h"
#include "sio2host.h"
#include "streetlight/streetlight.h"
#include "miwi_api.h"
#include "phy.h"

#if ((BOARD == SAMR30_XPLAINED_PRO) || (BOARD == SAMR21_XPLAINED_PRO))
#include "edbg-eui.h"
#endif

#if defined(ENABLE_NETWORK_FREEZER)
#include "pdsMemIds.h"
#include "pdsDataServer.h"
#include "wlPdsTaskManager.h"
#endif

#if defined(OTAU_ENABLED)
#include "otau.h"
#endif

#if defined(PAN_COORDINATOR) && defined(OTAU_SERVER)
#error "PAN Coordinator in WSN Demo cannot be OTAU Server, as both needs Serial to communicate with Tool..."
#endif

/************************** DEFINITIONS **********************************/
#if (BOARD == SAMR21ZLL_EK)
#define NVM_UID_ADDRESS   ((volatile uint16_t *)(0x00804008U))
#endif

#if (BOARD == SAMR30_MODULE_XPLAINED_PRO)
#define NVM_UID_ADDRESS   ((volatile uint16_t *)(0x0080400AU))
#endif

void ReadMacAddress(void);

int main ( void )
{
	irq_initialize_vectors();

	system_init();
	delay_init();
	
	cpu_irq_enable();	

	
	/* Read the MAC address from either flash or EDBG */
	ReadMacAddress();
	
    /* Initialize system Timer */
    SYS_TimerInit();

#if defined(ENABLE_NETWORK_FREEZER)
    nvm_init(INT_FLASH);
    PDS_Init();
#endif

	PHY_Init();
	
	PHY_ContinuousTransmission();

    /* Initialize the demo */
	//wsndemo_init();

    while(1)
    {
		//wsndemo_task();
#if defined(OTAU_ENABLED)
		otauTask();
#endif
    }
}

/*********************************************************************
* Function:         void ReadMacAddress()
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    Reads MAC Address from MAC Address EEPROM
*
* Side Effects:	    none
*
* Overview:		    Uses the MAC Address from the EEPROM for addressing
*
* Note:			    
**********************************************************************/
void ReadMacAddress(void)
{
#if ((BOARD == SAMR21ZLL_EK) || (BOARD == SAMR30_MODULE_XPLAINED_PRO))
   uint8_t i = 0, j = 0;
   for (i = 0; i < 8; i += 2, j++)
   {
     myLongAddress[i] = (NVM_UID_ADDRESS[j] & 0xFF);
	 myLongAddress[i + 1] = (NVM_UID_ADDRESS[j] >> 8);
   }
#elif ((BOARD == SAMR30_XPLAINED_PRO) || (BOARD == SAMR21_XPLAINED_PRO))
   uint8_t* peui64 = edbg_eui_read_eui64();
	for(uint8_t k=0; k<MY_ADDRESS_LENGTH; k++)
   {
		myLongAddress[k] = peui64[MY_ADDRESS_LENGTH-k-1];
   }
#endif
}

