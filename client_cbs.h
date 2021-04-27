/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 * Simplified (c) 2019, Hogeschool Rotterdam
 */


#ifndef __SERVER_CLIENT_CBS_H__
#define __SERVER_CLIENT_CBS_H__

/* MQTT library includes                                                      */
#include <ti/net/mqtt/mqttserver.h>
#include <ti/net/mqtt/mqttclient.h>

typedef enum {
    PUBLISH_PUSH_BUTTON_PRESSED,
    MSG_RECV_BY_CLIENT,
    LOCAL_CLIENT_DISCONNECTION,
    THREAD_TERMINATE_REQ
} Event;

// In this simple application the topic name can be at most MAX_TOPIC_LENGTH char's
#define MAX_TOPIC_LENGTH 63
// In this simple application the payload can be at most MAX_PAYLOAD_LENGTH char's
#define MAX_PAYLOAD_LENGTH 31

typedef struct
{
    Event event;
    // Next members are only valid when event == MSG_RECV_BY_CLIENT
    char topic[MAX_TOPIC_LENGTH + 1];
    char payload[MAX_PAYLOAD_LENGTH + 1];
    bool retain; // See https://www.hivemq.com/blog/mqtt-essentials-part-8-retained-messages/
    uint8_t qos; // See https://www.hivemq.com/blog/mqtt-essentials-part-6-mqtt-quality-of-service-levels/
    bool dup;
} msgType;

//******************************************************************************
// APIs
//******************************************************************************

extern void MqttClientCallback(int32_t event,
                               void * metaData,
                               uint32_t metaDateLen,
                               void *data,
                               uint32_t dataLen);

#endif // __SERVER_CLIENT_CBS_H__
