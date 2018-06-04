/**************************************************************************************/
/*                 Embedded Systems(ES-5101) Project - Client                         */
/*                                                                                    */
/*Program Objective:To introduce the technology at the heart of Internet-of-Things.   */
/*                  Specifically, the integration of sensors, wireless communication, */
/*                  and an UDP server.                                                */
/*                                                                                    */
/*Components used: Tiva C series TM4C123G Board and the add-on CC3100 to communicate  */
/*                 the SENSOR value to send it to the server.                         */
/*                                                                                    */
/*Hardware Connections:1)Tiva Board pin 3.3V connected to SENSOR +Vs                  */
/*                     2)Tiva Board pin GND connected to SENSOR GND                   */
/*                     3)Tiva Board pin PE3 connected to SENSOR OUPUT                 */
/*                     4)Tiva Board connected to CC3100 WIFI BOOST                    */
/*                                                                                    */
/*                                                                                    */
/*Citations:1)http://www.ti.com/lit/ug/swru368a/swru368a.pdf                          */
/*          2)http://processors.wiki.ti.com/index.php/CC31xx_%26_CC32xx               */
/*          3)http://edx-org-utaustinx.s3.amazonaws.com/UT601x/ValvanoWareTM4C123.zip */
/*          4)http://e2e.ti.com/support/wireless connectivity/simplelink wifi cc31xx  */
/*            cc32xx/f/968/p/471980/1700236                                           */
/*          5)udp_socket.c was used as a main.c for the lab and then changes were     */
/*            made accordingly of CC3100 SDK folder for TIVA C.                       */
/*                                                                                    */
/*Names: 1. Rajdeep Manjre (801020258) rmanjre@uncc.edu                               */
/*       2. Nisha Mishra   (800963552) nmishra@uncc.edu                               */
/*                                                                                    */
/*Submission Date: 12/12/2017                                                         */
/*                                                                                    */
/**************************************************************************************/


/*
 * main.c - UDP socket sample application
 *
 * Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Application Name     -   UDP socket
 * Application Overview -   This is a sample application demonstrating how to
 *                          open and use a standard UDP socket with CC3100.
 * Application Details  -   http://processors.wiki.ti.com/index.php/CC31xx_UDP_Socket_Application
 *                          doc\examples\udp_socket.pdf
 */

#include "simplelink.h"
#include "sl_common.h"

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_memmap.h"
#include"inc/hw_ints.h"
#include"inc/hw_types.h"
#include"driverlib/gpio.h"
#include"driverlib/pin_map.h"
#include"driverlib/sysctl.h"
#include"driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/interrupt.h"

#define APPLICATION_VERSION "1.2.0"

#define SL_STOP_TIMEOUT        0xFF

/* IP addressed of server side socket.
 * Should be in long format, E.g: 0xc0a8010a == 192.168.1.10
 */
#define IP_ADDR         0xC0A8022F        /* IP Address of server*/
#define PORT_NUM        5001              /* Port number to be used */

#define BUF_SIZE        100

/* Application specific status/error codes */
typedef enum{
	DEVICE_NOT_IN_STATION_MODE = -0x7D0,        /* Choosing this number to avoid overlap w/ host-driver's error codes */
	BSD_UDP_CLIENT_FAILED = DEVICE_NOT_IN_STATION_MODE - 1,

	STATUS_CODE_MAX = -0xBB8
}e_AppStatusCodes;

/*
 * GLOBAL VARIABLES -- Start
 */
_u8 g_Status = 0;

union
{
	_u8 BsdBuf[BUF_SIZE];
	_u32 demobuf[BUF_SIZE/4];
} uBuf;
/*
 * GLOBAL VARIABLES -- End
 */

/*
 * STATIC FUNCTION DEFINITIONS -- Start
 */
static _i32 configureSimpleLinkToDefaultState();
static _i32 establishConnectionWithAP();
static _i32 initializeAppVariables();
static _i32 BsdUdpClient(_u16 Port);
static void displayBanner();
/*
 * STATIC FUNCTION DEFINITIONS -- End
 */

/*
 * ASYNCHRONOUS EVENT HANDLERS -- Start
 */
/*!
    \brief This function handles WLAN events

    \param[in]      pWlanEvent is the event passed to the handler

    \return         None

    \note

    \warning
 */
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
	if(pWlanEvent == NULL)
	{
		CLI_Write(" [WLAN EVENT] NULL Pointer Error \n\r");
		return;
	}

	switch(pWlanEvent->Event)
	{
	case SL_WLAN_CONNECT_EVENT:
	{
		SET_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);

		/*
		 * Information about the connected AP (like name, MAC etc) will be
		 * available in 'slWlanConnectAsyncResponse_t' - Applications
		 * can use it if required
		 *
		 * slWlanConnectAsyncResponse_t *pEventData = NULL;
		 * pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
		 *
		 */
	}
	break;

	case SL_WLAN_DISCONNECT_EVENT:
	{
		slWlanConnectAsyncResponse_t*  pEventData = NULL;

		CLR_STATUS_BIT(g_Status, STATUS_BIT_CONNECTION);
		CLR_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

		pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

		/* If the user has initiated 'Disconnect' request, 'reason_code' is
		 * SL_USER_INITIATED_DISCONNECTION */
		if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
		{
			CLI_Write(" Device disconnected from the AP on application's request \n\r");
		}
		else
		{
			CLI_Write(" Device disconnected from the AP on an ERROR..!! \n\r");
		}
	}
	break;

	default:
	{
		CLI_Write(" [WLAN EVENT] Unexpected event \n\r");
	}
	break;
	}
}

/*!
    \brief This function handles events for IP address acquisition via DHCP
           indication

    \param[in]      pNetAppEvent is the event passed to the handler

    \return         None

    \note

    \warning
 */
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
	if(pNetAppEvent == NULL)
	{
		CLI_Write(" [NETAPP EVENT] NULL Pointer Error \n\r");
		return;
	}

	switch(pNetAppEvent->Event)
	{
	case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
	{
		SET_STATUS_BIT(g_Status, STATUS_BIT_IP_ACQUIRED);

		/*
		 * Information about the connection (like IP, gateway address etc)
		 * will be available in 'SlIpV4AcquiredAsync_t'
		 * Applications can use it if required
		 *
		 * SlIpV4AcquiredAsync_t *pEventData = NULL;
		 * pEventData = &pNetAppEvent->EventData.ipAcquiredV4;
		 *
		 */
	}
	break;

	default:
	{
		CLI_Write(" [NETAPP EVENT] Unexpected event \n\r");
	}
	break;
	}
}

/*!
    \brief This function handles callback for the HTTP server events

    \param[in]      pHttpEvent - Contains the relevant event information
    \param[in]      pHttpResponse - Should be filled by the user with the
                    relevant response information

    \return         None

    \note

    \warning
 */
void SimpleLinkHttpServerCallback(SlHttpServerEvent_t *pHttpEvent,
		SlHttpServerResponse_t *pHttpResponse)
{
	/* Unused in this application */
	CLI_Write(" [HTTP EVENT] Unexpected event \n\r");
}

/*!
    \brief This function handles general error events indication

    \param[in]      pDevEvent is the event passed to the handler

    \return         None
 */
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
	/*
	 * Most of the general errors are not FATAL are are to be handled
	 * appropriately by the application
	 */
	CLI_Write(" [GENERAL EVENT] \n\r");
}

/*!
    \brief This function handles socket events indication

    \param[in]      pSock is the event passed to the handler

    \return         None
 */
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
	if(pSock == NULL)
	{
		CLI_Write(" [SOCK EVENT] NULL Pointer Error \n\r");
		return;
	}

	switch( pSock->Event )
	{
	case SL_SOCKET_TX_FAILED_EVENT:
		/*
		 * TX Failed
		 *
		 * Information about the socket descriptor and status will be
		 * available in 'SlSockEventData_t' - Applications can use it if
		 * required
		 *
		 * SlSockEventData_u *pEventData = NULL;
		 * pEventData = & pSock->socketAsyncEvent;
		 */
		switch( pSock->socketAsyncEvent.SockTxFailData.status )
		{
		case SL_ECLOSE:
			CLI_Write(" [SOCK EVENT] Close socket operation failed to transmit all queued packets\n\r");
			break;
		default:
			CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
			break;
		}
		break;

		default:
			CLI_Write(" [SOCK EVENT] Unexpected event \n\r");
			break;
	}
}
/*
 * ASYNCHRONOUS EVENT HANDLERS -- End
 */

/*******************************************************************************/
/*Function Name: This function is used to initialize, set and read the         */
/*               12-bit ADC. The analog value from the sensor is extracted and */
/*               converted to digital and packed as an UDP message to send it  */
/*               sever.                                                        */
/*                                                                             */
/*Inputs:  Gets the sensor values using the ADCSequenceDataGet() function and  */
/*         stores it in ui32ADC0Value and then it is stored in temporary       */
/*         variable adc_val.                                                   */
/*                                                                             */
/*Outputs: The value stored in temporary variable adc_val is used for further  */
/*         in sending the message on server.                                   */
/*                                                                             */
/*******************************************************************************/
uint32_t ui32ADC0Value[1], adc_val;
void adc_init()
{
	// ENABLE MODULE ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);

	// ENABLE GPIO PORT E for MODULE ADC0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// ENABLE MODULE AN0 OF ADC0
	GPIOPinTypeADC(GPIO_PORTE_BASE,GPIO_PIN_3);

	// ADC0 MODULE, TRIGGER IS PROCESSOR EVENT, SEQUENCER 0 IS CONFIGURED
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

	// ADC0 MODULE, SEQUENCER 0 , FOR 1 SAMPLING, INPUT IS FROM CHANNEL 0 PE3
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);

	// ENABLE THE SEQUENCE 1 FOR ADC0
	ADCSequenceEnable(ADC0_BASE, 1);
	ADCProcessorTrigger(ADC0_BASE, 1);

	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);

	adc_val=ui32ADC0Value[1];
}


/*******************************************************************************/
/*Function Name: Main Function                                                 */
/*                                                                             */
/*Operations:1. The function for Sending the messages from client is called in */
/*              this main function.                                            */
/*           2. The initialization of signal clock is done.                    */
/*           3. The connection establishment and other initializations are done*/
/*                                                                             */
/*******************************************************************************/
int main(int argc, char** argv)
{
	_i32 retVal = -1;

	retVal = initializeAppVariables();
	ASSERT_ON_ERROR(retVal);

	/* Stop WDT and initialize the system-clock of the MCU
       These functions needs to be implemented in PAL */
	stopWDT();
	initClk();


	/* Configure command line interface */
	CLI_Configure();

	displayBanner();

	/*
	 * Following function configures the device to default state by cleaning
	 * the persistent settings stored in NVMEM (viz. connection profiles &
	 * policies, power policy etc)
	 *
	 * Applications may choose to skip this step if the developer is sure
	 * that the device is in its default state at start of application
	 *
	 * Note that all profiles and persistent settings that were done on the
	 * device will be lost
	 */
	retVal = configureSimpleLinkToDefaultState();
	if(retVal < 0)
	{
		CLI_Write(" Failed to configure the device in its default state \n\r");

		LOOP_FOREVER();
	}

	CLI_Write(" Device is configured in default state \n\r");

	/*
	 * Assumption is that the device is configured in station mode already
	 * and it is in its default state
	 */
	/* Initializing the CC3100 device */
	retVal = sl_Start(0, 0, 0);
	if ((retVal < 0) ||
			(ROLE_STA != retVal) )
	{
		CLI_Write(" Failed to start the device \n\r");
		LOOP_FOREVER();
	}

	CLI_Write(" Device started as STATION \n\r");

	/* Connecting to WLAN AP - Set with static parameters defined at the top
       After this call we will be connected and have IP address */
	retVal = establishConnectionWithAP();
	if(retVal < 0)
	{
		CLI_Write(" Failed to establish connection w/ an AP \n\r");
		LOOP_FOREVER();
	}

	CLI_Write(" Connection established w/ AP and IP is acquired \n\r");

	CLI_Write(" Started sending data to UDP server \n\r");


	retVal = BsdUdpClient(PORT_NUM);
	if(retVal < 0)
		CLI_Write(" Failed to send data to UDP server\n\r");
	else
		CLI_Write(" successfully sent data to UDP server \n\r");
	/*
    CLI_Write(" Waiting for data from UDP client \n\r");

    retVal = BsdUdpServer(PORT_NUM);
    if(retVal < 0)
        CLI_Write(" Failed to read data from the UDP client \n\r");
    else
        CLI_Write(" Successfully received data from UDP client \n\r");
	 */
	/* Stop the CC3100 device */
	retVal = sl_Stop(SL_STOP_TIMEOUT);
	if(retVal < 0)
	{
		LOOP_FOREVER();
	}

	return 0;
}

/*!
    \brief This function configure the SimpleLink device in its default state. It:
           - Sets the mode to STATION
           - Configures connection policy to Auto and AutoSmartConfig
           - Deletes all the stored profiles
           - Enables DHCP
           - Disables Scan policy
           - Sets Tx power to maximum
           - Sets power policy to normal
           - Unregisters mDNS services
           - Remove all filters

    \param[in]      none

    \return         On success, zero is returned. On error, negative is returned
 */
static _i32 configureSimpleLinkToDefaultState()
{
	SlVersionFull   ver = {0};
	_WlanRxFilterOperationCommandBuff_t  RxFilterIdMask = {0};

	_u8           val = 1;
	_u8           configOpt = 0;
	_u8           configLen = 0;
	_u8           power = 0;

	_i32          retVal = -1;
	_i32          mode = -1;

	mode = sl_Start(0, 0, 0);
	ASSERT_ON_ERROR(mode);

	/* If the device is not in station-mode, try configuring it in station-mode */
	if (ROLE_STA != mode)
	{
		if (ROLE_AP == mode)
		{
			/* If the device is in AP mode, we need to wait for this event before doing anything */
			while(!IS_IP_ACQUIRED(g_Status)) { _SlNonOsMainLoopTask(); }
		}

		/* Switch to STA role and restart */
		retVal = sl_WlanSetMode(ROLE_STA);
		ASSERT_ON_ERROR(retVal);

		retVal = sl_Stop(SL_STOP_TIMEOUT);
		ASSERT_ON_ERROR(retVal);

		retVal = sl_Start(0, 0, 0);
		ASSERT_ON_ERROR(retVal);

		/* Check if the device is in station again */
		if (ROLE_STA != retVal)
		{
			/* We don't want to proceed if the device is not coming up in station-mode */
			ASSERT_ON_ERROR(DEVICE_NOT_IN_STATION_MODE);
		}
	}

	/* Get the device's version-information */
	configOpt = SL_DEVICE_GENERAL_VERSION;
	configLen = sizeof(ver);
	retVal = sl_DevGet(SL_DEVICE_GENERAL_CONFIGURATION, &configOpt, &configLen, (_u8 *)(&ver));
	ASSERT_ON_ERROR(retVal);

	/* Set connection policy to Auto + SmartConfig (Device's default connection policy) */
	retVal = sl_WlanPolicySet(SL_POLICY_CONNECTION, SL_CONNECTION_POLICY(1, 0, 0, 0, 1), NULL, 0);
	ASSERT_ON_ERROR(retVal);

	/* Remove all profiles */
	retVal = sl_WlanProfileDel(0xFF);
	ASSERT_ON_ERROR(retVal);

	/*
	 * Device in station-mode. Disconnect previous connection if any
	 * The function returns 0 if 'Disconnected done', negative number if already disconnected
	 * Wait for 'disconnection' event if 0 is returned, Ignore other return-codes
	 */
	retVal = sl_WlanDisconnect();
	if(0 == retVal)
	{
		/* Wait */
		while(IS_CONNECTED(g_Status)) { _SlNonOsMainLoopTask(); }
	}

	/* Enable DHCP client*/
	retVal = sl_NetCfgSet(SL_IPV4_STA_P2P_CL_DHCP_ENABLE,1,1,&val);
	ASSERT_ON_ERROR(retVal);

	/* Disable scan */
	configOpt = SL_SCAN_POLICY(0);
	retVal = sl_WlanPolicySet(SL_POLICY_SCAN , configOpt, NULL, 0);
	ASSERT_ON_ERROR(retVal);

	/* Set Tx power level for station mode
       Number between 0-15, as dB offset from max power - 0 will set maximum power */
	power = 0;
	retVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (_u8 *)&power);
	ASSERT_ON_ERROR(retVal);

	/* Set PM policy to normal */
	retVal = sl_WlanPolicySet(SL_POLICY_PM , SL_NORMAL_POLICY, NULL, 0);
	ASSERT_ON_ERROR(retVal);

	/* Unregister mDNS services */
	retVal = sl_NetAppMDNSUnRegisterService(0, 0);
	ASSERT_ON_ERROR(retVal);

	/* Remove  all 64 filters (8*8) */
	pal_Memset(RxFilterIdMask.FilterIdMask, 0xFF, 8);
	retVal = sl_WlanRxFilterSet(SL_REMOVE_RX_FILTER, (_u8 *)&RxFilterIdMask,
			sizeof(_WlanRxFilterOperationCommandBuff_t));
	ASSERT_ON_ERROR(retVal);

	retVal = sl_Stop(SL_STOP_TIMEOUT);
	ASSERT_ON_ERROR(retVal);

	retVal = initializeAppVariables();
	ASSERT_ON_ERROR(retVal);

	return SUCCESS;
}

/*!
    \brief Connecting to a WLAN Access point

    This function connects to the required AP (SSID_NAME).
    The function will return once we are connected and have acquired IP address

    \param[in]  None

    \return     0 on success, negative error-code on error

    \note

    \warning    If the WLAN connection fails or we don't acquire an IP address,
                We will be stuck in this function forever.
 */
static _i32 establishConnectionWithAP()
{
	SlSecParams_t secParams = {0};
	_i32 retVal = 0;

	secParams.Key = PASSKEY;
	secParams.KeyLen = pal_Strlen(PASSKEY);
	secParams.Type = SEC_TYPE;

	retVal = sl_WlanConnect(SSID_NAME, pal_Strlen(SSID_NAME), 0, &secParams, 0);
	ASSERT_ON_ERROR(retVal);

	/* Wait */
	while((!IS_CONNECTED(g_Status)) || (!IS_IP_ACQUIRED(g_Status))) { _SlNonOsMainLoopTask(); }

	return SUCCESS;
}

/*******************************************************************************/
/*Function Name: UDP Client                                                    */
/*                                                                             */
/*Operations:1. This function is used to open the UDP socket and waits for the */
/*              connection to the server with the given IP address & PORT_NUM  */
/*                                                                             */
/*           2. The functions sends the UDP packets using the function         */
/*              sl_SendTo.                                                     */
/*                                                                             */
/*           3. The data received from the ADC is divided by 20 as LM35 has a  */
/*              range from -55 to 150 degree C, thus, 205 units. The 12- bit   */
/*              ADC(4095) is divided in 205 parts which gives the value 20.    */
/*              The acquired data is then converted from integer to character  */
/*              and packed in the uBuf.BsdBuf[] buffer. The values are divided */
/*              from hundredth digit to units digit and then put in the buffer.*/
/*              by adding and decimal value of char '0'.                       */
/*                                                                             */
/*******************************************************************************/


static _i32 BsdUdpClient(_u16 Port)
{
	SlSockAddrIn_t  Addr;
	_u16            idx = 0;
	_u16            AddrSize = 0;
	_i16            SockID = 0;
	_i16            Status = 0;

	for (idx=0 ; idx<BUF_SIZE ; idx++)
	{
		uBuf.BsdBuf[idx] = (_u8)(idx % 10);
	}

	Addr.sin_family = SL_AF_INET;
	Addr.sin_port = sl_Htons((_u16)Port);
	Addr.sin_addr.s_addr = sl_Htonl((_u32)IP_ADDR);

	AddrSize = sizeof(SlSockAddrIn_t);

	SockID = sl_Socket(SL_AF_INET,SL_SOCK_DGRAM, 0);
	if( SockID < 0 )
	{
		ASSERT_ON_ERROR(SockID);
	}
	int count=0;

	while (1)
	{
		adc_init();
		if(count==0)
			count++;
		else{

			uBuf.BsdBuf[0] = 'a';
			uBuf.BsdBuf[1] = '=';

			int temp=0;
    		int data = (int)adc_val/20;
    		temp=data/100;
    		data=data-temp*100;
    		uBuf.BsdBuf[2]= temp+48;
    		temp=data/10;
    		uBuf.BsdBuf[3]=temp+48;
    		data=data-temp*10;
    		uBuf.BsdBuf[4]=data+48;

			Status = sl_SendTo(SockID, uBuf.BsdBuf, BUF_SIZE, 0,
					(SlSockAddr_t *)&Addr, AddrSize);
			if( Status <= 0 )
			{
				Status = sl_Close(SockID);
				ASSERT_ON_ERROR(BSD_UDP_CLIENT_FAILED);
			}
		}

	}

	Status = sl_Close(SockID);
	ASSERT_ON_ERROR(Status);

	return SUCCESS;
}

/*!
    \brief This function initializes the application variables

    \param[in]  None

    \return     0 on success, negative error-code on error
 */
static _i32 initializeAppVariables()
{
	g_Status = 0;
	pal_Memset(uBuf.BsdBuf, 0, sizeof(uBuf));

	return SUCCESS;
}

/*!
    \brief This function displays the application's banner

    \param      None

    \return     None
 */
static void displayBanner()
{
	CLI_Write("\n\r\n\r");
	CLI_Write(" UDP socket application - Version ");
	CLI_Write(APPLICATION_VERSION);
	CLI_Write("\n\r*******************************************************************************\n\r");
}
