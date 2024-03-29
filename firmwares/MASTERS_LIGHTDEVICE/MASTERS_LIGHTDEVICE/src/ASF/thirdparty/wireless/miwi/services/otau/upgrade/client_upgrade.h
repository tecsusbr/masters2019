/**
* \file client_upgrade.c
*
* \brief Client Upgrade Interface
*
* Copyright (c) 2018 Microchip Technology Inc. and its subsidiaries.
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


#ifndef CLIENT_UPGRADE_H
#define CLIENT_UPGRADE_H

typedef enum {
	STATE_IDLE,
	STATE_START_DOWNLOAD,
	STATE_IMAGE_REQUESTED,
	STATE_IMAGE_RESPONDED,
	STATE_WAITING_FOR_SWITCH,
	STATE_SWITCH_SUCCESS
}upgradeState_t;

typedef enum {
	UPGRADE_OTAU_IDLE,
	SERVER_DISCOVERY_REQUEST_SENT,
	QUERY_IMAGE_REQUEST_SENT,
	IMAGE_REQUEST_SENT,
	SWITCH_REQUEST_SENT,
}otauUpgradeState_t;

#endif /* CLIENT_UPGRADE_H */