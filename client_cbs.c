/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 * Simplified (c) 2019, Hogeschool Rotterdam
 */

/* Standard includes                                                         */
#include <stdlib.h>

/* Kernel (Non OS/Free-RTOS/TI-RTOS) includes                                */
#include "pthread.h"
#include "mqueue.h"

/* Common interface includes                                                 */
#include "uart_term.h"

/* Application includes                                                      */
#include "client_cbs.h"

extern bool gResetApplication;

//*****************************************************************************
//                          LOCAL DEFINES
//*****************************************************************************
#define APP_PRINT               Report

#define OS_WAIT_FOREVER         (0xFFFFFFFF)
#define OS_NO_WAIT              (0)
#define OS_OK                   (0)

#define MQTTClientCbs_ConnackRC(data) (data & 0xff) 
/**< CONNACK: Return Code (LSB) */

//*****************************************************************************
//                 GLOBAL VARIABLES
//*****************************************************************************

//*****************************************************************************
//                 Queue external function
//*****************************************************************************
extern bool MQTT_SendMsgToQueue(msgType *queueElement);

//****************************************************************************
//                      CLIENT CALLBACKS
//****************************************************************************

//*****************************************************************************
//
//! Callback in case of various event (for clients connection with remote
//! broker)
//!
//! \param[in]  event       - is a event occurred
//! \param[in]  metaData    - is the pointer for the message buffer
//!                           (for this event)
//! \param[in]  metaDateLen - is the length of the message buffer
//! \param[in]  data        - is the pointer to the buffer for data
//!                           (for this event)
//! \param[in]  dataLen     - is the length of the buffer data
//!
//! return none
//
//*****************************************************************************
void MqttClientCallback(int32_t event, void * metaData, uint32_t metaDateLen,
    void *data, uint32_t dataLen)
{
    int32_t i = 0;

    switch((MQTTClient_EventCB)event)
    {
        case MQTTClient_OPERATION_CB_EVENT:
            switch(((MQTTClient_OperationMetaDataCB *)metaData)->messageType)
            {
                case MQTTCLIENT_OPERATION_CONNACK:
                {
                    uint16_t *ConnACK = (uint16_t*) data;
                    APP_PRINT("CONNACK:\n\r");
                    /* Check if Conn Ack return value is Success (0) or       */
                    /* Error - Negative value                                 */
                    if(0 == (MQTTClientCbs_ConnackRC(*ConnACK)))
                    {
                        APP_PRINT("Connection Success\n\r");
                    }
                    else
                    {
                        APP_PRINT("Connection Error: %d\n\r", *ConnACK);
                    }
                    break;
                }

                case MQTTCLIENT_OPERATION_EVT_PUBACK:
                {
                    char *PubAck = (char *) data;
                    APP_PRINT("PubAck:\n\r");
                    APP_PRINT("%s\n\r", PubAck);
                    break;
                }

                case MQTTCLIENT_OPERATION_SUBACK:
                    APP_PRINT("Sub Ack:\n\r");
                    APP_PRINT("Granted QoS Levels are:\n\r");
                    for(i = 0; i < dataLen; i++)
                    {
                        APP_PRINT("Topic %d :QoS %d\n\r", i,
                                  ((unsigned char*) data)[i]);
                    }
                    break;
                case MQTTCLIENT_OPERATION_UNSUBACK:
                {
                    char *UnSub = (char *) data;
                    APP_PRINT("UnSub Ack \n\r");
                    APP_PRINT("%s\n\r", UnSub);
                    break;
                }

                default:
                    break;
            }
            break;
        case MQTTClient_RECV_CB_EVENT:
        {
            MQTTClient_RecvMetaDataCB *recvMetaData = (MQTTClient_RecvMetaDataCB *)metaData;

            msgType msg;
            msg.event = MSG_RECV_BY_CLIENT;
            msg.retain = recvMetaData->retain;
            msg.dup = recvMetaData->dup;
            msg.qos = recvMetaData->qos;

            strncpy(msg.topic, recvMetaData->topic, MAX_TOPIC_LENGTH);
            if (recvMetaData->topLen < MAX_TOPIC_LENGTH)
            {
                msg.topic[recvMetaData->topLen] = '\0';
            }
            strncpy(msg.payload, (char *)data, MAX_PAYLOAD_LENGTH);
            if (dataLen < MAX_PAYLOAD_LENGTH)
            {
                msg.payload[dataLen] ='\0';
            }

//            APP_PRINT("\n\rMQTT message received\n\r");
//            APP_PRINT("TOPIC: %s\n\r", msg.topic);
//            APP_PRINT("PAYLOAD: %s\n\r", msg.payload);
//            APP_PRINT("QOS: %d\n\r", msg.qos);


            if (msg.retain)
            {
                APP_PRINT("Retained\n\r");
            }

            if (msg.dup)
            {
                APP_PRINT("Duplicate\n\r");
            }

            /* send message to message queue g_PBQueue */
            MQTT_SendMsgToQueue(&msg);
            break;
        }
        case MQTTClient_DISCONNECT_CB_EVENT:
        {
            gResetApplication = true;
            APP_PRINT("BRIDGE DISCONNECTION\n\r");
            break;
        }
    }
}
