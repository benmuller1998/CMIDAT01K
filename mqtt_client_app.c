/*
 * Copyright (c) 2016, Texas Instruments Incorporated
 * All rights reserved.
 * Simplified (c) 2019, Hogeschool Rotterdam
 */

/*****************************************************************************

   Application Name     -  MQTT Client
   Application Overview -  The device is running a MQTT client which is
                           connected to the online broker. Three LEDs on the
                           device can be controlled from a web client by
                           publishing msg on appropriate topics. Similarly,
                           a message can be published on a pre-configured topic
                           by pressing the switch SW2 button on the device.

*****************************************************************************/

#include <stdlib.h>
#include <pthread.h>
#include <mqueue.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>


#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>

#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/slnetifwifi.h>

#include <ti/net/mqtt/mqttclient.h>

#include "network_if.h"
#include "uart_term.h"

#include "Board.h"
#include "client_cbs.h"

#define CLIENT_INIT_STATE (0x01)
#define MQTT_INIT_STATE (0x04)

/* BMP280 I2C Adres*/
#define BMP280_ADDR          0x77

/* Operate library in MQTT 3.1 mode. */
#define MQTT_3_1 true

/* Defining broker IP address and port number */
#define SERVER_ADDRESS "mqtt.thingspeak.com"
#define PORT_NUMBER 1883

/* Defining subscription topic values */
#define SUBSCRIPTION_TOPIC0 "channels/1337986/subscribe/fields/field1/QMMEA3ZC3JWETPR3"

/* Defining publish topic values */
#define PUBLISH_TOPIC0 "channels/1337986/publish/fields/field2/DIX8Q35UZ1EIGU8V"
#define PUBLISH_TOPIC1 "channels/1337986/publish/fields/field1/DIX8Q35UZ1EIGU8V"

void Mqtt_ClientStop(uint8_t disconnect);
int32_t MqttClient_start();

/* Global variables */

double Tempratuur;
double SetTempr;
char   tempC[5];

bool gResetApplication = false;
/* Connection state: (0) - connected, (negative) - disconnected */
static int32_t gApConnectionState = -1;
static uint32_t gInitState = 0;
static uint32_t gUiConnFlag = 0;
static MQTTClient_Handle gMqttClient;

/* If ClientId isn't set, the MAC address of the device will be copied into */
/* the ClientID variable. */
char ClientId[13] = {'\0'};
const char *ClientUsername = "cc3200";
const char *ClientPassword = "ABAO6FY824P7G5LQ";

/* Message Queue */
mqd_t g_PBQueue;

//*
// MQTT_SendMsgToQueue - Utility function that sends msgType parameter to
// the message queue g_PBQueue with timeout of 0.
// If the queue isn't full the parameter will be stored and the function
// will return true.
// If the queue is full and the timeout expired (because the timeout parameter
// is 0 it will expire immediately), the parameter is thrown away and the
// function will return false.
//*

void MaakTaak(unsigned char prioriteit, unsigned short stackSize, void*(*functie)(void * args))
{
    pthread_t           Thread;
    pthread_attr_t      attrs;
    struct sched_param  priParam;
    int                 retc;

    priParam.sched_priority = prioriteit;
    pthread_attr_init(&attrs);
    pthread_attr_setdetachstate(&attrs, PTHREAD_CREATE_DETACHED); //ontkoppelde taak
    pthread_attr_setschedparam(&attrs, &priParam); //prioriteit instellen
    pthread_attr_setstacksize(&attrs, stackSize); //stacksize instellen

    retc = pthread_create(&Thread, &attrs, functie, NULL); //taak starten
    if (retc != 0) {
        /* pthread_create() failed */
        while (1);
    }
}


bool MQTT_SendMsgToQueue(msgType *msg)
{
    struct timespec abstime = {0};

    clock_gettime(CLOCK_REALTIME, &abstime);

    if (g_PBQueue)
    {
        /* send message to the message queue */
        if (mq_timedsend(g_PBQueue, (char *) msg, sizeof(msg), 0, &abstime) == 0)
        {
            return true;
        }
        UART_PRINT("Message queue is full.\n\r");
        return false;
    }
    UART_PRINT("Message queue does not exist.\n\r");
    return false;
}


void *I2C_thread(void *args){
    uint8_t         schrijf_buffer[2];
    uint8_t         lees_buffer[2];
    uint8_t         buffer;
    uint8_t BMP_REG[3] = {0xFA,0xFB,0xFC};
    uint32_t        tempratuuruint;
    uint32_t        t_fine, var1, var2;

    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    int i=0;
    long int i2 = 0;
    unsigned short dig_T1 = 28842;   //sensor variable constante
    short dig_T2 = 26266;
    short dig_T3 = 50;

    I2C_init();

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
            if (i2c == NULL) {
                printf("fout bij het instaliren I2c \n");
                while (1);
            }
            /* I2C transaction setup */
            i2cTransaction.writeBuf   = schrijf_buffer;
            i2cTransaction.writeCount = 2;
            i2cTransaction.readBuf    = lees_buffer;
            i2cTransaction.readCount  = 1;


            i2cTransaction.slaveAddress = BMP280_ADDR;
            schrijf_buffer[0] = 0xF4;       //adress control reg
            schrijf_buffer[1] = 0b01000011; // Zet Tempratuur meting aan en R op 2 , druk meting uit en stroom op Normal
                if (!I2C_transfer(i2c, &i2cTransaction)) {
                 /* RTC niet gevonden */
                 printf("BMP280 niet gevonden \n");
                 while(1);
                  }
                if (I2C_transfer(i2c, &i2cTransaction)) {
                 printf("Control reg BMP280 ingesteld \n");
                }
            I2C_close(i2c);

while(1){
    for(i=0; i<3; i++){
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
        if (i2c == NULL) {
            printf("fout bij het instaliren I2c \n");
            while (1);
        }
        /* I2C transaction setup */
        i2cTransaction.writeBuf   = schrijf_buffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf    = lees_buffer;
        i2cTransaction.readCount  = 1;


        i2cTransaction.slaveAddress = BMP280_ADDR;
        schrijf_buffer[0] = BMP_REG[i];
            if (!I2C_transfer(i2c, &i2cTransaction)) {
             printf("BMP280 niet gevonden \n");
             while(1);
              }

            if (I2C_transfer(i2c, &i2cTransaction)) {
                buffer = lees_buffer[0];
               if(i==0){
                tempratuuruint = buffer << 12 ;
               }
               if(i==1){
                tempratuuruint = tempratuuruint+(buffer<<4);
               }
               if(i==2){
                tempratuuruint = tempratuuruint+(buffer>>4);
               }
            }
        I2C_close(i2c);
    }
    var1  = (((tempratuuruint>>3) - (dig_T1<<1)) * dig_T2) >> 11;
    var2  = ((((tempratuuruint>>4) - dig_T1) * ((tempratuuruint>>4) - dig_T1)) >> 12) * dig_T3 >> 14;
    t_fine = var1 + var2;
    Tempratuur  = ((t_fine* 5 + 128)>>8)/100.00;
    if(Tempratuur<SetTempr){
        GPIO_write(Board_GPIO_LED0_P64, Board_GPIO_LED_ON);
    }
    else{
        GPIO_write(Board_GPIO_LED0_P64,Board_GPIO_LED_OFF );
    }
    }
}


//upload om de 20 sec de huidige kamer temp
void *publish_thread(void *args){
    while(1){
    msgType msg;
    msg.event = PUBLISH_PUSH_BUTTON_PRESSED;
    MQTT_SendMsgToQueue(&msg);
    sleep(20);
    }
}
//*
// Push Button Handler1(GPIOSW2). Press push button0 (GPIOSW2) whenever user
// wants to publish a message. Write message into message queue signaling the
// event publish messages
//*
void pushButtonInterruptHandler2(uint_least8_t index)
{
    msgType msg;
    msg.event = PUBLISH_PUSH_BUTTON_PRESSED;

    MQTT_SendMsgToQueue(&msg);
}

//*
// Push Button Handler2(GPIOSW3). Press push button3 whenever you want to
// disconnect from the remote broker.
//*
void pushButtonInterruptHandler3(uint_least8_t index)
{
    gResetApplication = true;
}

void *MqttClientThread(void *pvParameters)
{
    MQTTClient_run(gMqttClient);

    msgType msg;
    msg.event = LOCAL_CLIENT_DISCONNECTION;

    if (MQTT_SendMsgToQueue(&msg) == false)
    {
        UART_PRINT("Throw away first message and send the new one.\n\r");
        msgType msgRecv;
        mq_receive(g_PBQueue, (char *)&msgRecv, sizeof(msgRecv), NULL);
        MQTT_SendMsgToQueue(&msg);
    }

    pthread_exit(0);

    return NULL;
}

//*
// Task implementing MQTT Server plus client bridge
//
// This function
//    1. Initializes network driver and connects to the default AP
//    2. Initializes the mqtt client librarie and set up MQTT
//       with the remote broker.
//    3. set up the button event and its callback (for publishing)
//    4. handles the callback signals
//
//*
void *MqttClient(void *pvParameters)
{
    /* Initializing client and subscribing to the broker. */
    if (gApConnectionState >= 0)
    {
        int32_t retVal = MqttClient_start();
        if (retVal == -1)
        {
            UART_PRINT("MQTT Client library initialization failed.\n\r");
            pthread_exit(0);
            return NULL;
        }
    }

    /* Handling the signals from various callbacks */
    while (1)
    {
        /* waiting for the message queue */
        msgType msgRecv;
        mq_receive(g_PBQueue, (char *)&msgRecv, sizeof(msgRecv), NULL);

        switch(msgRecv.event)
        {
            case PUBLISH_PUSH_BUTTON_PRESSED:
            {
                char publish_data[5];
                sprintf(publish_data, "%.2f", Tempratuur); // zet Tempratuur om in een char om te publishen

                int16_t retVal = MQTTClient_publish(gMqttClient, PUBLISH_TOPIC0, strlen(PUBLISH_TOPIC0),
                publish_data, strlen(publish_data), MQTT_QOS_0);


                if (retVal < 0)
                {
                    UART_PRINT("MQTT publish failed.\n\r");
                }
                else
                {
                    UART_PRINT("\n\rCC3200 publishes the following message:\n\r");
//                    UART_PRINT("Topic: %s\n\r", PUBLISH_TOPIC0);
                    UART_PRINT("Data: %s\n\r", publish_data);
                }
                break;
            }
            case MSG_RECV_BY_CLIENT:

                if (strcmp(msgRecv.topic, SUBSCRIPTION_TOPIC0) == 0)
                {
//                       MQTTClient_publish (gMqttClient, PUBLISH_TOPIC0, strlen(PUBLISH_TOPIC0),msgRecv.payload, strlen(msgRecv.payload), MQTT_QOS_0);
                       char setTemp[5];
                       int i;
                        UART_PRINT("\n\rCC3200 ontvangt op topic 0:\n\r");
                        UART_PRINT("Data: %s\n\r", msgRecv.payload);
                        for(i=0; i<5; i++){
                       setTemp[i] = msgRecv.payload[i];
                        }
                        SetTempr = atof(setTemp);
                }


            /* On-board client disconnected from remote broker */
            case LOCAL_CLIENT_DISCONNECTION:
//                UART_PRINT("\n\rOn-board client disconnected.\r\n");
                gUiConnFlag = 0;
                break;

            case THREAD_TERMINATE_REQ:
                gUiConnFlag = 0;
                pthread_exit(0);
                return NULL;
        }
    }
}

//*
// This function connect the MQTT device to an AP with the SSID which was
// configured in SSID_NAME definition which can be found in Network_if.h file,
// if the device can't connect to this AP a request from the user for other
// (open) SSID will appear.
//*
int32_t Mqtt_IF_Connect()
{
    /* Reset The state of the machine */
    Network_IF_ResetMCUStateMachine();

    /* Start the driver */
    long lRetVal = Network_IF_InitDriver(ROLE_STA);
    if (lRetVal < 0)
    {
        UART_PRINT("Failed to start SimpleLink Device. Error: %d.\n\r", lRetVal);
        return -1;
    }
    else
    {
        UART_PRINT("SimpleLink Device started successfully.\n\r");
    }

    /* Initialize AP security params */
    SlWlanSecParams_t SecurityParams = { 0 };
    SecurityParams.Key = SECURITY_KEY;
    SecurityParams.KeyLen = strlen(SECURITY_KEY);
    SecurityParams.Type = SECURITY_TYPE;

    /* Connect to the Access Point */
    lRetVal = Network_IF_ConnectAP(SSID_NAME, SecurityParams);
    if (lRetVal < 0)
    {
        UART_PRINT("Connection to AP %s failed\n\r", SSID_NAME);
        return -1;
    }
    else
    {
        UART_PRINT("SimpleLink Device connected to AP %s successfully.\n\r", SSID_NAME);
    }

    return 0;
}

//*
// MQTT Start - Initialize and create all the items required to run the MQTT
// protocol
//*
void Mqtt_start()
{
    mq_attr attr;
    attr.mq_maxmsg = 10;
    attr.mq_msgsize = sizeof(msgType);
    g_PBQueue = mq_open("g_PBQueue", O_CREAT|O_RDWR, 0, &attr);

    if (g_PBQueue == (mqd_t)-1)
    {
        UART_PRINT("MQTT Message Queue create failed.\n\r");
        gInitState &= ~MQTT_INIT_STATE;
        return;
    }

    /* Create mqttThread */
    pthread_attr_t pAttrs;
    pthread_attr_init(&pAttrs);
    struct sched_param priParam;
    priParam.sched_priority = 2;
    int retc = pthread_attr_setschedparam(&pAttrs, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs, 2048);
    retc |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    pthread_t mqttThread;
    retc |= pthread_create(&mqttThread, &pAttrs, MqttClient, NULL);

    if(retc != 0)
    {
        gInitState &= ~MQTT_INIT_STATE;
        UART_PRINT("MQTT thread create failed.\n\r");
        return;
    }

    /* Enable interrupt GPIO 22 (SW2) */
    GPIO_setCallback(Board_GPIO_BUTTON0, pushButtonInterruptHandler2);
    GPIO_enableInt(Board_GPIO_BUTTON0);
    /* Enable interrupt GPIO 13 (SW3) */
    GPIO_setCallback(Board_GPIO_BUTTON1, pushButtonInterruptHandler3);
    GPIO_enableInt(Board_GPIO_BUTTON1);

    gInitState &= ~MQTT_INIT_STATE;
}

//*
// MQTT Stop - Close the client instance and free all the items required to
// run the MQTT protocol
//*
void Mqtt_Stop()
{
    if (gApConnectionState >= 0)
    {
        Mqtt_ClientStop(1);
    }

    msgType msg;
    msg.event = THREAD_TERMINATE_REQ;

    if (MQTT_SendMsgToQueue(&msg) == false)
    {
        UART_PRINT("Throw away first message and send the new one.\n\r");
        msgType msgRecv;
        mq_receive(g_PBQueue, (char *)&msgRecv, sizeof(msgRecv), NULL);
        MQTT_SendMsgToQueue(&msg);
    }

    sleep(2);

    mq_close(g_PBQueue);
    g_PBQueue = NULL;

    sl_Stop(SL_STOP_TIMEOUT);
    UART_PRINT("Client stop completed.\r\n");

    /* Disable the SW2 interrupt */
    GPIO_disableInt(Board_GPIO_BUTTON0);
    /* Disable the SW3 interrupt */
    GPIO_disableInt(Board_GPIO_BUTTON1);
}

int32_t MqttClient_start()
{
    /* Open client receive thread and start the receive task. */

    MQTTClient_ConnParams Mqtt_ClientCtx =
    {
        MQTTCLIENT_NETCONN_URL, SERVER_ADDRESS, PORT_NUMBER, 0, 0, 0, NULL
    };

    MQTTClient_Params MqttClientExmple_params;
    MqttClientExmple_params.clientId = ClientId;
    MqttClientExmple_params.connParams = &Mqtt_ClientCtx;
    MqttClientExmple_params.mqttMode31 = MQTT_3_1;
    MqttClientExmple_params.blockingSend = true;

    gInitState |= CLIENT_INIT_STATE;

    /*Initialize MQTT client library */
    gMqttClient = MQTTClient_create(MqttClientCallback, &MqttClientExmple_params);

    if (gMqttClient == NULL)
    {
        /* lib initialization failed                                         */
        gInitState &= ~CLIENT_INIT_STATE;
        return -1;
    }

    pthread_attr_t pAttrs;
    pthread_attr_init(&pAttrs);
    struct sched_param priParam;
    priParam.sched_priority = 2;
    int retVal = pthread_attr_setschedparam(&pAttrs, &priParam);
    retVal |= pthread_attr_setstacksize(&pAttrs, 4096);
    retVal |= pthread_attr_setdetachstate(&pAttrs, PTHREAD_CREATE_DETACHED);
    pthread_t rx_task;
    retVal |= pthread_create(&rx_task, &pAttrs, MqttClientThread, NULL);
    if (retVal != 0)
    {
        UART_PRINT("Client thread create failed.\n\r");
        gInitState &= ~CLIENT_INIT_STATE;
        return -1;
    }
   // stelt client username/password in
    MQTTClient_set(gMqttClient, MQTTClient_USER_NAME, (void *)ClientUsername,
                   strlen(
                       (char*)ClientUsername));

    MQTTClient_set(gMqttClient, MQTTClient_PASSWORD, (void *)ClientPassword,
                   strlen(
                       (char*)ClientPassword));

    /* Initiate MQTT connect */
    if (gApConnectionState >= 0)
    {
        /* The return code of MQTTClient_connect is the ConnACK value that
         *  returns from the server */
        int16_t retVal = MQTTClient_connect(gMqttClient);

        /* Negative retVal means error, 0 means connection successful without
         * session stored by the server, greater than 0 means successful
         * connection with session stored by the server */
        if (retVal < 0)
        {
            /* Library initialization failed                                     */
            UART_PRINT("Connection to broker failed, Error code: %d.\n\r", retVal);
            gUiConnFlag = 0;
        }
        else
        {
            gUiConnFlag = 1;
        }
        /* Subscribe to topic when session is not stored by the server */
        if ((gUiConnFlag == 1) && (retVal == 0))
        {
            MQTTClient_SubscribeParams subscriptionInfo[] = {
                {SUBSCRIPTION_TOPIC0, NULL, MQTT_QOS_0, 0}
            };
            if (MQTTClient_subscribe(gMqttClient, subscriptionInfo,
                sizeof(subscriptionInfo)/sizeof(subscriptionInfo[0])) < 0)
            {
                UART_PRINT("Subscribe error.\n\r");
                MQTTClient_disconnect(gMqttClient);
                gUiConnFlag = 0;
            }
            else
            {
                uint8_t i;
                for (i = 0; i < sizeof(subscriptionInfo)/sizeof(subscriptionInfo[0]); i++)
                {
                    UART_PRINT("Client subscribed on %s\n\r", subscriptionInfo[i].topic);
                }
            }
        }
    }

    gInitState &= ~CLIENT_INIT_STATE;

    return 0;
}

void Mqtt_ClientStop(uint8_t disconnect)
{
    MQTTClient_UnsubscribeParams unsubscriptionInfo[] = {
        {SUBSCRIPTION_TOPIC0, 0}
    };

    if (MQTTClient_unsubscribe(gMqttClient, unsubscriptionInfo,
        sizeof(unsubscriptionInfo)/sizeof(unsubscriptionInfo[0])) < 0)
    {
        UART_PRINT("Unsubscribe error.\n\r");
        MQTTClient_disconnect(gMqttClient);
        gUiConnFlag = 0;
    }
    else
    {
        uint8_t i;
        for (i = 0; i < sizeof(unsubscriptionInfo)/sizeof(unsubscriptionInfo[0]); i++)
        {
            UART_PRINT("Client unsubscribed from %s\n\r", unsubscriptionInfo[i].topic);
        }
    }

    gUiConnFlag = 0;

    /*exiting the Client library */
    MQTTClient_delete(gMqttClient);
}

//*
// Set the ClientId with its own mac address
// This routine converts the mac address which is given
// by an integer type variable in hexadecimal base to ASCII
// representation, and copies it into the ClientId parameter.
//*
int32_t SetClientIdNamefromMacAddress()
{
    int32_t ret;
    uint8_t Client_Mac_Name[2];

    /* When ClientID isn't set, use the MAC address as ClientID */
    if(ClientId[0] == '\0')
    {
        /* Get the device MAC address */
        uint16_t macAddressLen = SL_MAC_ADDR_LEN;
        uint8_t macAddress[SL_MAC_ADDR_LEN];
        ret = sl_NetCfgGet(SL_NETCFG_MAC_ADDRESS_GET, 0, &macAddressLen, macAddress);

        uint8_t Index;
        for (Index = 0; Index < SL_MAC_ADDR_LEN; Index++)
        {
            /* Each MAC address byte contains two hexadecimal characters */
            /* Copy the 4 MSB - the most significant character */
            Client_Mac_Name[0] = (macAddress[Index] >> 4) & 0xf;
            /* Copy the 4 LSB - the least significant character */
            Client_Mac_Name[1] = macAddress[Index] & 0xf;

            uint8_t i;
            for (i = 0; i <= 1; i++)
            {
                if (Client_Mac_Name[i] > 9)
                {
                    /* Converts and copies from number that is greater than 9 in
                     * hexadecimal representation (a to f) into ASCII character */
                    ClientId[2 * Index + i] = Client_Mac_Name[i] + 'a' - 10;
                }
                else
                {
                    /* Converts and copies from number 0 - 9 in hexadecimal
                     * representation into ASCII character */
                    ClientId[2 * Index + i] = Client_Mac_Name[i] + '0';
                }
            }
        }
    }
    return ret;
}

void *mainThread(void *args)
{
    /* Initialize SlNetSock layer with CC32xx interface */
    SlNetIf_init(0);
    SlNetIf_add(SLNETIF_ID_1, "CC32xx", (const SlNetIf_Config_t *)&SlNetIfConfigWifi, 5);

    SlNetSock_init(0);
    SlNetUtil_init(0);

    GPIO_init();
    SPI_init();

    GPIO_write(Board_GPIO_LED0_P64, Board_GPIO_LED_OFF);
    GPIO_write(Board_GPIO_LED1, Board_GPIO_LED_OFF);
    GPIO_write(Board_GPIO_LED2, Board_GPIO_LED_OFF);

    /* Configure the UART */
    UART_Handle tUartHndl = InitTerm();
    /* Remove uart receive from LPDS dependency */
    UART_control(tUartHndl, UART_CMD_RXDISABLE, NULL);

    /*Create the sl_Task */
    pthread_attr_t pAttrs_spawn;
    pthread_attr_init(&pAttrs_spawn);
    struct sched_param priParam;
    priParam.sched_priority = 9;
    int retc = pthread_attr_setschedparam(&pAttrs_spawn, &priParam);
    retc |= pthread_attr_setstacksize(&pAttrs_spawn, 2048);
    retc |= pthread_attr_setdetachstate(&pAttrs_spawn, PTHREAD_CREATE_DETACHED);
    pthread_t spawn_thread;
    retc |= pthread_create(&spawn_thread, &pAttrs_spawn, sl_Task, NULL);

    MaakTaak(2, 2048,I2C_thread);
//    MaakTaak(2, 2048,publish_thread);

    if (retc != 0)
    {
        UART_PRINT("Could not create simplelink task.\r\n");
        while (1);
    }

    retc = sl_Start(0, 0, 0);
    if (retc < 0)
    {
        UART_PRINT("sl_Start failed.\r\n");
        while (1);
    }

    /* Output device information to the UART terminal */
    /* Set the ClientId with its own mac address */
    retc = SetClientIdNamefromMacAddress();

    retc |= sl_Stop(SL_STOP_TIMEOUT);
    if (retc < 0)
    {
        UART_PRINT("sl_Stop failed\r\n");
        while (1);
    }

    uint32_t count = 0;
    while (1)
    {
        gResetApplication = false;
        gInitState = 0;

        /* Connect to AP */
        gApConnectionState = Mqtt_IF_Connect();

        gInitState |= MQTT_INIT_STATE;
        /* Run MQTT main thread (it will open the client) */
        Mqtt_start();

        /* Wait for init to be completed */
        while (gInitState != 0)
        {
            UART_PRINT(".");
            sleep(1);
        }

        while (gResetApplication == false);

        UART_PRINT("TO Complete - Closing all threads and resources\r\n");

        /*Stop the MQTT Process                                              */
        Mqtt_Stop();

        UART_PRINT("\r\nReopen MQTT # %d\r\n", ++count);
    }
}
