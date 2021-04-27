/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 * Simplified (c) 2019, Hogeschool Rotterdam
 */

//*****************************************************************************
// network_if.h
//
// Networking interface macro and function prototypes for CC32xx device
//
//*****************************************************************************

#ifndef __NETWORK_IF__H__
#define __NETWORK_IF__H__

/* Simplelink includes                                                        */
#include <ti/drivers/net/wifi/simplelink.h>

/* Values for below macros shall be modified as per access-point(AP)          */
/* properties SimpleLink device will connect to following AP when application */
/* is executed.                                                               */

/* AP SSID                                                                    */
#define SSID_NAME               "Aquanaut"
/* Security type (OPEN or WEP or WPA)                                         */
#define SECURITY_TYPE           SL_WLAN_SEC_TYPE_WPA_WPA2
/* Password of the secured AP                                                 */
#define SECURITY_KEY            "*******"

#define SL_STOP_TIMEOUT         200

/* Status bits - used to set/reset the corresponding bits in given a variable */
typedef enum
{
    /* If this bit is set: Network Processor is powered up                    */
    STATUS_BIT_NWP_INIT = 0,

    /* If this bit is set: the device is connected to the AP or client is     */
    /* connected to device (AP)                                               */
    STATUS_BIT_CONNECTION,

    /* If this bit is set: the device has leased IP to any connected client.  */
    STATUS_BIT_IP_LEASED,

    /* If this bit is set: the device has acquired an IP                      */
    STATUS_BIT_IP_ACQUIRED,

    /* If this bit is set: the SmartConfiguration process is started from     */
    /* SmartConfig app                                                        */
    STATUS_BIT_SMARTCONFIG_START,

    /* If this bit is set: the device (P2P mode) found any p2p-device in scan */
    STATUS_BIT_P2P_DEV_FOUND,

    /* If this bit is set: the device (P2P mode) found any p2p-negotiation    */
    /* request                                                                */
    STATUS_BIT_P2P_REQ_RECEIVED,

    /* If this bit is set: the device(P2P mode) connection to client (or      */
    /* reverse way) is failed                                                 */
    STATUS_BIT_CONNECTION_FAILED,

    /* If this bit is set: the device has completed the ping operation        */
    STATUS_BIT_PING_DONE,

    /* If this bit is set: the device has acquired an IPv6 address.           */
    STATUS_BIT_IPV6L_ACQUIRED,

    /* If this bit is set: the device has acquired an IPv6 address.           */
    STATUS_BIT_IPV6G_ACQUIRED,
    STATUS_BIT_AUTHENTICATION_FAILED,
    STATUS_BIT_RESET_REQUIRED,
}e_StatusBits;

#define CLR_STATUS_BIT_ALL(status_variable)  (status_variable = 0)
#define SET_STATUS_BIT(status_variable, bit) (status_variable |= (1 << (bit)))
#define CLR_STATUS_BIT(status_variable, bit) (status_variable &= ~(1 << (bit)))
#define GET_STATUS_BIT(status_variable, bit) (0 != \
                                              (status_variable & (1 << (bit))))

#define IS_CONNECTED(status_variable)        GET_STATUS_BIT( \
        status_variable, \
        STATUS_BIT_CONNECTION)

#define IS_IP_ACQUIRED(status_variable)      GET_STATUS_BIT( \
        status_variable, \
        STATUS_BIT_IP_ACQUIRED)

//*****************************************************************************
// APIs
//*****************************************************************************
long Network_IF_InitDriver(uint32_t uiMode);
long Network_IF_DeInitDriver(void);
long Network_IF_ConnectAP(char * pcSsid,
                          SlWlanSecParams_t SecurityParams);
long Network_IF_DisconnectFromAP();
long Network_IF_IpConfigGet(unsigned long *aucIP,
                            unsigned long *aucSubnetMask,
                            unsigned long *aucDefaultGateway,
                            unsigned long *aucDNSServer);
long Network_IF_GetHostIP(char* pcHostName,
                          unsigned long * pDestinationIP);
unsigned long Network_IF_CurrentMCUState(void);
void Network_IF_UnsetMCUMachineState(char stat);
void Network_IF_SetMCUMachineState(char stat);
void Network_IF_ResetMCUStateMachine(void);

#endif // __NETWORK_IF__H__
