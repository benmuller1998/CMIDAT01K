/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 * Simplified (c) 2019, Hogeschool Rotterdam
 */

// Networking interface functions for CC32xx device

#include <string.h>
#include <stdlib.h>

#include <pthread.h>
#include <mqueue.h>
#include <unistd.h>

#include <ti/drivers/net/wifi/simplelink.h>

#include "network_if.h"
#include "uart_term.h"

/* Network App specific status and error codes which are used only in this file */
typedef enum
{
    /* Choosing this number to avoid overlap with host-driver's error codes */
    DEVICE_NOT_IN_STATION_MODE = -0x7F0,
    DEVICE_NOT_IN_AP_MODE = DEVICE_NOT_IN_STATION_MODE - 1,
    DEVICE_NOT_IN_P2P_MODE = DEVICE_NOT_IN_AP_MODE - 1,
    STATUS_CODE_MAX = -0xBB8
} e_NetAppStatusCodes;

/* Station IP address */
unsigned long g_ulStaIp = 0;
/* Network Gateway IP address */
unsigned long g_ulGatewayIP = 0;
/* Connection SSID */
unsigned char g_ucConnectionSSID[SL_WLAN_SSID_MAX_LENGTH + 1];
/* Connection BSSID */
unsigned char g_ucConnectionBSSID[SL_WLAN_BSSID_LENGTH ];
/* SimpleLink Status */
volatile unsigned long g_ulStatus = 0;
/* Connection time delay index */
volatile unsigned short g_usConnectIndex;

//*
// SimpleLink Asynchronous Event Handlers -- Start
//*
void SimpleLinkHttpServerEventHandler(
    SlNetAppHttpServerEvent_t *pSlHttpServerEvent,
    SlNetAppHttpServerResponse_t *
    pSlHttpServerResponse)
{
}

void SimpleLinkNetAppRequestEventHandler(
    SlNetAppRequest_t *pNetAppRequest,
    SlNetAppResponse_t *pNetAppResponse)
{
}

void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
}

//*
// On Successful completion of Wlan Connect, This function triggers connection
// status to be set.
//*
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pSlWlanEvent)
{
    SlWlanEventDisconnect_t* pEventData = NULL;

    switch (pSlWlanEvent->Id)
    {
        case SL_WLAN_EVENT_CONNECT:
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);

            memcpy(g_ucConnectionSSID, pSlWlanEvent->Data.Connect.SsidName,
                   pSlWlanEvent->Data.Connect.SsidLen);
            memcpy(g_ucConnectionBSSID, pSlWlanEvent->Data.Connect.Bssid,
                   SL_WLAN_BSSID_LENGTH);

            UART_PRINT(
                "[WLAN EVENT] STA Connected to the AP: %s, BSSID: "
                "%02x:%02x:%02x:%02x:%02x:%02x\n\r", g_ucConnectionSSID,
                g_ucConnectionBSSID[0], g_ucConnectionBSSID[1],
                g_ucConnectionBSSID[2], g_ucConnectionBSSID[3],
                g_ucConnectionBSSID[4], g_ucConnectionBSSID[5]);
            break;

        case SL_WLAN_EVENT_DISCONNECT:
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

            pEventData = &pSlWlanEvent->Data.Disconnect;

            /* If the user has initiated 'Disconnect' request, 'reason_code' */
            /* is SL_WLAN_DISCONNECT_USER_INITIATED                          */
            if (SL_WLAN_DISCONNECT_USER_INITIATED == pEventData->ReasonCode)
            {
                UART_PRINT("Device disconnected from the AP on application's request\n\r");
            }
            else
            {
                UART_PRINT("Device disconnected from the AP on an ERROR!\n\r");
            }
            break;

        case SL_WLAN_EVENT_STA_ADDED:
            /* When device is in AP mode and any client connects to it. */
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            break;

        case SL_WLAN_EVENT_STA_REMOVED:
            /* When device is in AP mode and any client disconnects from it. */
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            break;

        default:
            UART_PRINT("[WLAN EVENT] Unexpected event %d\n\r", pSlWlanEvent->Id);
            break;
    }
}

//*
// The Function Handles the Fatal errors
//*
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{
    switch (slFatalErrorEvent->Id)
    {
        case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
            UART_PRINT(
                "[ERROR] - FATAL ERROR: Abort NWP event detected: "
                "AbortType=%d, AbortData=0x%x\n\r",
                slFatalErrorEvent->Data.DeviceAssert.Code,
                slFatalErrorEvent->Data.DeviceAssert.Value);
            break;

        case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
            UART_PRINT("[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
            break;

        case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
            UART_PRINT(
                "[ERROR] - FATAL ERROR: No Cmd Ack detected "
                "[cmd opcode = 0x%x] \n\r",
                slFatalErrorEvent->Data.NoCmdAck.Code);
            break;

        case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
            UART_PRINT("[ERROR] - FATAL ERROR: Sync loss detected n\r");
            break;

        case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
            UART_PRINT(
                "[ERROR] - FATAL ERROR: Async event timeout detected "
                "[event opcode =0x%x]  \n\r",
                slFatalErrorEvent->Data.CmdTimeout.Code);
            break;

        default:
            UART_PRINT("[ERROR] - FATAL ERROR: Unspecified error detected \n\r");
            break;
    }
}

//*
// This function handles network events such as IP acquisition, IP leased, IP
// released etc.
//*
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch (pNetAppEvent->Id)
    {
        case SL_NETAPP_EVENT_IPV4_ACQUIRED:
        case SL_NETAPP_EVENT_IPV6_ACQUIRED:
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);
            UART_PRINT("[NETAPP EVENT] IP acquired by the device\n\r");
            break;

        case SL_NETAPP_EVENT_DHCPV4_LEASED:
            SET_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);
            g_ulStaIp = (pNetAppEvent)->Data.IpLeased.IpAddress;

            UART_PRINT("[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d , ",
                       SL_IPV4_BYTE(g_ulStaIp,
                                    3),
                       SL_IPV4_BYTE(g_ulStaIp,
                                    2),
                       SL_IPV4_BYTE(g_ulStaIp, 1), SL_IPV4_BYTE(g_ulStaIp, 0));
            break;

        case SL_NETAPP_EVENT_DHCPV4_RELEASED:
            CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_LEASED);

            UART_PRINT("[NETAPP EVENT] IP Released for Client: "
                       "IP=%d.%d.%d.%d , ", SL_IPV4_BYTE(g_ulStaIp,
                                                         3),
                       SL_IPV4_BYTE(g_ulStaIp,
                                    2),
                       SL_IPV4_BYTE(g_ulStaIp, 1), SL_IPV4_BYTE(g_ulStaIp, 0));
            break;

        default:
            UART_PRINT("[NETAPP EVENT] Unexpected event [0x%x] \n\r",
                       pNetAppEvent->Id);
            break;
    }
}

//*
// This function handles resource request
//*
void SimpleLinkNetAppRequestHandler(SlNetAppRequest_t *pNetAppRequest,
                                    SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

//*
// This function handles HTTP server events
//*
void SimpleLinkHttpServerCallback(SlNetAppHttpServerEvent_t *pHttpEvent,
                                  SlNetAppHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
}

//*
// This function handles General Events
//*
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    /* Most of the general errors are not FATAL. are to be handled */
    /* appropriately by the application. */
    UART_PRINT("[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
               pDevEvent->Data.Error.Code,
               pDevEvent->Data.Error.Source);
}

//*
// This function handles socket events indication
//*
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /* This application doesn't work w/ socket - Events are not expected */
    switch (pSock->Event)
    {
        case SL_SOCKET_TX_FAILED_EVENT:
            switch (pSock->SocketAsyncEvent.SockTxFailData.Status)
            {
                case SL_ERROR_BSD_ECLOSE:
                    UART_PRINT(
                        "[SOCK ERROR] - close socket (%d) operation "
                        "failed to transmit all queued packets\n\r",
                        pSock->SocketAsyncEvent.SockTxFailData.Sd);
                    break;
                default:
                    UART_PRINT(
                        "[SOCK ERROR] - TX FAILED  :  socket %d , "
                        "reason (%d) \n\n",
                        pSock->SocketAsyncEvent.SockTxFailData.Sd,
                        pSock->SocketAsyncEvent.SockTxFailData.Status);
                    break;
            }
            break;
        case SL_SOCKET_ASYNC_EVENT:
        {
            UART_PRINT("[SOCK ERROR] an event received on socket %d\r\n",
                       pSock->SocketAsyncEvent.SockAsyncData.Sd);
            switch (pSock->SocketAsyncEvent.SockAsyncData.Type)
            {
                case SL_SSL_NOTIFICATION_CONNECTED_SECURED:
                    UART_PRINT("[SOCK ERROR] SSL handshake done");
                    break;
                case SL_SSL_NOTIFICATION_HANDSHAKE_FAILED:
                    UART_PRINT("[SOCK ERROR] SSL handshake failed with error %d\r\n",
                               pSock->SocketAsyncEvent.SockAsyncData.Val);
                    break;
                case SL_SSL_ACCEPT:
                    UART_PRINT(
                        "[SOCK ERROR] Recoverable error occurred "
                        "during the handshake %d\r\n",
                        pSock->SocketAsyncEvent.SockAsyncData.Val);
                    break;
                case SL_OTHER_SIDE_CLOSE_SSL_DATA_NOT_ENCRYPTED:
                    UART_PRINT("[SOCK ERROR] Other peer terminated the SSL layer.\r\n");
                    break;
                case SL_SSL_NOTIFICATION_WRONG_ROOT_CA:
                    UART_PRINT("[SOCK ERROR] Used wrong CA to verify the peer.\r\n");
                    break;
                default:
                    break;
            }
            break;
        }
        default:
            UART_PRINT("[SOCK EVENT] - Unexpected Event [%x0x]\n\n", pSock->Event);
            break;
    }
}

//*
// SimpleLink Asynchronous Event Handlers -- End
//*

//*
// This function initializes the application variables
//*
void InitializeAppVariables(void)
{
    g_ulStatus = 0;
    g_ulStaIp = 0;
    g_ulGatewayIP = 0;

    memset(g_ucConnectionSSID, 0, sizeof(g_ucConnectionSSID));
    memset(g_ucConnectionBSSID, 0, sizeof(g_ucConnectionBSSID));
}

//*
// The function initializes a CC32xx device and triggers it to start operation
//*
long Network_IF_InitDriver(uint32_t uiMode)
{
    long lRetVal = -1;

    /* Reset CC32xx Network State Machine */
    InitializeAppVariables();

    /* Following function configure the device to default state by cleaning  */
    /* the persistent settings stored in NVMEM (viz. connection profiles     */
    /* & policies, power policy etc) Applications may choose to skip this    */
    /* step if the developer is sure that the device is in its default state */
    /* at start of application. Note that all profiles and persistent        */
    /* settings that were done on the device will be lost.                   */
    lRetVal = sl_Start(NULL, NULL, NULL);

    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start the device \n\r");
        while (1);
    }

    switch (lRetVal)
    {
        case ROLE_STA:
            UART_PRINT("Device came up in Station mode\n\r");
            break;
        case ROLE_AP:
            UART_PRINT("Device came up in Access-Point mode\n\r");
            break;
        case ROLE_P2P:
            UART_PRINT("Device came up in P2P mode\n\r");
            break;
        default:
            UART_PRINT("Error:unknown mode\n\r");
            break;
    }

    if (uiMode != lRetVal)
    {
        UART_PRINT("Switching Networking mode on application request\n\r");

        /* Switch to AP role and restart */
        lRetVal = sl_WlanSetMode(uiMode);
        if (lRetVal < 0)
        {
            ERR_PRINT(lRetVal);
            return lRetVal;
        }

        lRetVal = sl_Stop(SL_STOP_TIMEOUT);
        lRetVal = sl_Start(0, 0, 0);
        if (lRetVal < 0)
        {
            ERR_PRINT(lRetVal);
            return lRetVal;
        }

        if (lRetVal == uiMode)
        {
            switch (lRetVal)
            {
                case ROLE_STA:
                    UART_PRINT("Device came up in Station mode\n\r");
                    break;
                case ROLE_AP:
                    UART_PRINT("Device came up in Access-Point mode\n\r");
                    /* If the device is in AP mode, we need to wait for this */
                    /* event before doing anything. */
                    while (!IS_IP_ACQUIRED(g_ulStatus))
                    {
                        usleep(1000);
                    }
                    break;
                case ROLE_P2P:
                    UART_PRINT("Device came up in P2P mode\n\r");
                    break;
                default:
                    UART_PRINT("Error:unknown mode\n\r");
                    break;
            }
        }
        else
        {
            UART_PRINT("could not configure correct networking mode\n\r");
            while (1);
        }
    }
    else
    {
        if (lRetVal == ROLE_AP)
        {
            while (!IS_IP_ACQUIRED(g_ulStatus))
            {
                usleep(1000);
            }
        }
    }
    return 0;
}

//*
// The function de-initializes a CC32xx device
//*
long Network_IF_DeInitDriver(void)
{
    long lRetVal = -1;

    UART_PRINT("SL Disconnect.\n\r");

    /* Disconnect from the AP */
    lRetVal = Network_IF_DisconnectFromAP();

    /* Stop the simplelink host */
    sl_Stop(SL_STOP_TIMEOUT);

    /* Reset the state to uninitialized */
    Network_IF_ResetMCUStateMachine();
    return lRetVal;
}

//*
// Connect to an Access Point using the specified SSID
//*
long Network_IF_ConnectAP(char *pcSsid, SlWlanSecParams_t SecurityParams)
{
    char acCmdStore[128];
    unsigned short usConnTimeout;
    unsigned char ucRecvdAPDetails;
    long lRetVal;
    unsigned long ulIP = 0;
    unsigned long ulSubMask = 0;
    unsigned long ulDefGateway = 0;
    unsigned long ulDns = 0;

    g_usConnectIndex = 0;

    /* Disconnect from the AP */
    Network_IF_DisconnectFromAP();

    CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
    CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);

    /* Continue only if SSID is not empty */
    if (pcSsid != NULL)
    {
        /* This triggers the CC32xx to connect to a specific AP. */
        lRetVal =
            sl_WlanConnect((signed char *) pcSsid, strlen((const char *) pcSsid),
                           NULL, &SecurityParams, NULL);
        if (lRetVal < 0)
        {
            ERR_PRINT(lRetVal);
            return lRetVal;
        }

        /* Wait for ~10 sec to check if connection to desire AP succeeds */
        while (g_usConnectIndex < 10)
        {
            sleep(1);

            if (IS_CONNECTED(g_ulStatus) && IS_IP_ACQUIRED(g_ulStatus))
            {
                break;
            }
            g_usConnectIndex++;
        }
    }
    else
    {
        UART_PRINT("Empty SSID, Could not connect\n\r");
        return -1;
    }

    /* Check and loop until AP connection successful, else ask new AP SSID */
    while (!(IS_CONNECTED(g_ulStatus)) || !(IS_IP_ACQUIRED(g_ulStatus)))
    {
        /* Disconnect the previous attempt */
        Network_IF_DisconnectFromAP();

        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_CONNECTION);
        CLR_STATUS_BIT(g_ulStatus, STATUS_BIT_IP_ACQUIRED);
        UART_PRINT("Device could not connect to %s\n\r", pcSsid);

        do
        {
            ucRecvdAPDetails = 0;

            UART_PRINT("\n\r\n\rPlease enter the AP(open) SSID name # ");

            /* Get the AP name to connect over the UART */
            lRetVal = GetCmd(acCmdStore, sizeof(acCmdStore));
            if (lRetVal > 0)
            {
                /* remove start/end spaces if any                            */
                lRetVal = TrimSpace(acCmdStore);

                if (lRetVal)
                {
                    /* Parse the AP name */
                    strncpy(pcSsid, acCmdStore, lRetVal);
                    if (pcSsid != NULL)
                    {
                        ucRecvdAPDetails = 1;
                        pcSsid[lRetVal] = '\0';
                    }
                }
            }
        }
        while (ucRecvdAPDetails == 0);

        /* Set Security Parameters to OPEN security type. */
        SecurityParams.Key = (signed char *) "";
        SecurityParams.KeyLen = 0;
        SecurityParams.Type = SL_WLAN_SEC_TYPE_OPEN;

        UART_PRINT("\n\rTrying to connect to AP: %s ...\n\r", pcSsid);

        /* Get the current timer tick and setup the timeout accordingly. */
        usConnTimeout = g_usConnectIndex + 15;

        /* This triggers the CC32xx to connect to specific AP. */
        lRetVal =
            sl_WlanConnect((signed char *) pcSsid, strlen((const char *) pcSsid),
                           NULL, &SecurityParams,NULL);
        if (lRetVal < 0)
        {
            ERR_PRINT(lRetVal);
            return lRetVal;
        }

        /* Wait ~10 sec to check if connection to specified AP succeeds */
        while (!(IS_CONNECTED(g_ulStatus)) || !(IS_IP_ACQUIRED(g_ulStatus)))
        {
            sleep(1);

            if (g_usConnectIndex >= usConnTimeout)
            {
                break;
            }
            g_usConnectIndex++;
        }
    }

    /* Put message on UART */
    UART_PRINT("\n\rDevice has connected to %s\n\r", pcSsid);

    /* Get IP address */
    lRetVal = Network_IF_IpConfigGet(&ulIP, &ulSubMask, &ulDefGateway, &ulDns);
    if (lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        return lRetVal;
    }

    /* Send the information */
    UART_PRINT("Device IP Address is %d.%d.%d.%d \n\r\n\r",
               SL_IPV4_BYTE(ulIP, 3), SL_IPV4_BYTE(ulIP, 2), SL_IPV4_BYTE(ulIP,
                                                                          1),
               SL_IPV4_BYTE(ulIP, 0));
    return 0;
}

//*
// Disconnects from an Access Point
//*
long Network_IF_DisconnectFromAP(void)
{
    long lRetVal = 0;

    if (IS_CONNECTED(g_ulStatus))
    {
        lRetVal = sl_WlanDisconnect();
        if (0 == lRetVal)
        {
            while (IS_CONNECTED(g_ulStatus))
            {
                usleep(1000);
            }
            return lRetVal;
        }
        else
        {
            return lRetVal;
        }
    }
    else
    {
        return lRetVal;
    }
}

//*
// Get the IP Address of the device.
//*
long Network_IF_IpConfigGet(unsigned long *pulIP,
                            unsigned long *pulSubnetMask,
                            unsigned long *pulDefaultGateway,
                            unsigned long *pulDNSServer)
{
    unsigned short usDHCP = 0;
    long lRetVal = -1;
    unsigned short len = sizeof(SlNetCfgIpV4Args_t);
    SlNetCfgIpV4Args_t ipV4 = { 0 };

    /* get network configuration */
    lRetVal =
        sl_NetCfgGet(SL_NETCFG_IPV4_STA_ADDR_MODE, &usDHCP, &len,
                     (unsigned char *) &ipV4);
    if (lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        return lRetVal;
    }

    *pulIP = ipV4.Ip;
    *pulSubnetMask = ipV4.IpMask;
    *pulDefaultGateway = ipV4.IpGateway;
    *pulDefaultGateway = ipV4.IpDnsServer;

    return lRetVal;
}

//*
//  This function obtains the server IP address using a DNS lookup
//*
long Network_IF_GetHostIP(char* pcHostName, unsigned long * pDestinationIP)
{
    long lStatus = 0;

    lStatus =
        sl_NetAppDnsGetHostByName((signed char *) pcHostName, strlen(
                                      pcHostName), pDestinationIP, SL_AF_INET);
    if (lStatus < 0)
    {
        ERR_PRINT(lStatus);
        return lStatus;
    }

    UART_PRINT("Get Host IP succeeded.\n\rHost: %s IP: %d.%d.%d.%d \n\r\n\r",
               pcHostName, SL_IPV4_BYTE(*pDestinationIP,
                                        3), SL_IPV4_BYTE(*pDestinationIP,
                                                         2),
               SL_IPV4_BYTE(*pDestinationIP, 1),
               SL_IPV4_BYTE(*pDestinationIP, 0));

    return lStatus;
}

//*
// Reset state from the state machine
//*
void Network_IF_ResetMCUStateMachine()
{
    g_ulStatus = 0;
}

//*
// Return the current state bits
//*
unsigned long Network_IF_CurrentMCUState()
{
    return g_ulStatus;
}

//*
// Sets a state from the state machine
//*
void Network_IF_SetMCUMachineState(char cStat)
{
    SET_STATUS_BIT(g_ulStatus, cStat);
}

//*
// Unsets a state from the state machine
//*
void Network_IF_UnsetMCUMachineState(char cStat)
{
    CLR_STATUS_BIT(g_ulStatus, cStat);
}
