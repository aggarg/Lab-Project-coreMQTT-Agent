/*
 * Lab-Project-coreMQTT-Agent 201215
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 */

/*
 * This demo creates multiple tasks, all of which use the MQTT agent API to
 * communicate with an MQTT broker through the same MQTT connection.
 *
 * This file contains the initial task created after the TCP/IP stack connects
 * to the network.  The task:
 *
 * 1) Connects to the MQTT broker.
 * 2) Creates the other demo tasks, in accordance with the #defines set in
 *    demo_config.h.  For example, if demo_config.h contains the following
 *    settings:
 *
 *    #define democonfigCREATE_LARGE_MESSAGE_SUB_PUB_TASK     1
 *    #define democonfigNUM_SIMPLE_SUB_PUB_TASKS_TO_CREATE 3
 *
 *    then the initial task will create the task implemented in
 *    LargeMessageSubScribePublish.c and three instances of the task
 *    implemented in SimpleSubscribePublishT.c.  See the comments at the top
 *    of those files for more information.
 *
 * 3) After creating the demo tasks the initial task could create the MQTT
 *    agent task.  However, as it has no other operations to perform, rather
 *    than create the MQTT agent as a separate task the initial task just calls
 *    the agent's implementing function - effectively turning itself into the
 *    MQTT agent.
 */


/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <assert.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

/* Demo Specific configs. */
#include "demo_config.h"

/* MQTT library includes. */
#include "core_mqtt.h"

/* MQTT agent include. */
#include "freertos_mqtt_agent.h"

/* Exponential backoff retry include. */
#include "exponential_backoff.h"


/* Transport interface include. */
#if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 )
    #include "using_mbedtls.h"
#else
    #include "using_plaintext.h"
#endif

/* This demo uses compile time options to select the demo tasks to created.
 * Ensure the compile time options are defined.  These should be defined in
 * demo_config.h. */
#ifndef democonfigCREATE_LARGE_MESSAGE_SUB_PUB_TASK
    #error Please define democonfigCREATE_LARGE_MESSAGE_SUB_PUB_TASK to 1 or 0 in demo_config.h - determines if vStartLargeMessageSubscribePublishTask() gets called or not.
#endif

#if ( democonfigCREATE_LARGE_MESSAGE_SUB_PUB_TASK != 0 ) && !defined( democonfigLARGE_MESSAGE_SUB_PUB_TASK_STACK_SIZE )
    #error Please define democonfigLARGE_MESSAGE_SUB_PUB_TASK_STACK_SIZE in demo_config.h to set the stack size (in words, not bytes) for the task created by vStartLargeMessageSubscribePublishTask().
#endif

#ifndef democonfigNUM_SIMPLE_SUB_PUB_TASKS_TO_CREATE
    #error Please set democonfigNUM_SIMPLE_SUB_PUB_TASKS_TO_CREATE to the number of tasks to create in vStartSimpleSubscribePublishTask().  Can be zero.
#endif

#if ( democonfigNUM_SIMPLE_SUB_PUB_TASKS_TO_CREATE > 0 ) && !defined( democonfigSIMPLE_SUB_PUB_TASK_STACK_SIZE )
    #error Please define democonfigSIMPLE_SUB_PUB_TASK_STACK_SIZE in demo_config.h to set the stack size (in words, not bytes) for the tasks created by vStartSimpleSubscribePublishTask().
#endif

#ifndef democonfigCREATE_CODE_SIGNING_OTA_DEMO
    #error Please define democonfigCREATE_CODE_SIGNING_OTA_DEMO to 1 or 0 in demo_config.h - determines if vStartOTACodeSigningDemo() gets called or not.
#endif

#if ( democonfigCREATE_CODE_SIGNING_OTA_DEMO != 0 ) && !defined( democonfigCODE_SIGNING_OTA_TASK_STACK_SIZE )
    #error Please define democonfigCODE_SIGNING_OTA_TASK_STACK_SIZE in demo_config.h to set the stack size (in words, not bytes) for the task created by vStartOTACodeSigningDemo().
#endif


/**
 * These configuration settings are required to run the demo.
 */

/**
 * @brief Timeout for receiving CONNACK after sending an MQTT CONNECT packet.
 * Defined in milliseconds.
 */
#define mqttexampleCONNACK_RECV_TIMEOUT_MS           ( 1000U )

/**
 * @brief The maximum time interval in seconds which is allowed to elapse
 *  between two Control Packets.
 *
 *  It is the responsibility of the Client to ensure that the interval between
 *  Control Packets being sent does not exceed the this Keep Alive value. In the
 *  absence of sending any other Control Packets, the Client MUST send a
 *  PINGREQ Packet.
 *//*_RB_ Move to be the responsibility of the agent. */
#define mqttexampleKEEP_ALIVE_INTERVAL_SECONDS       ( 60U )

/**
 * @brief Socket send and receive timeouts to use.  Specified in milliseconds.
 */
#define mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS    ( 750 )

/**
 * @brief Used to convert times to/from ticks and milliseconds.
 */
#define mqttexampleMILLISECONDS_PER_SECOND           ( 1000U )
#define mqttexampleMILLISECONDS_PER_TICK             ( mqttexampleMILLISECONDS_PER_SECOND / configTICK_RATE_HZ )

/**
 * @brief The MQTT agent manages the MQTT contexts.  This set the handle to the
 * context used by this demo.
 */
#define mqttexampleMQTT_CONTEXT_HANDLE               ( ( MQTTContextHandle_t ) 0 )


/*-----------------------------------------------------------*/

/**
 * @brief Initializes an MQTT context, including transport interface and
 * network buffer.
 *
 * @return `MQTTSuccess` if the initialization succeeds, else `MQTTBadParameter`.
 */
static MQTTStatus_t prvMQTTInit( void );

/**
 * @brief Sends an MQTT Connect packet over the already connected TCP socket.
 *
 * @param[in] pxMQTTContext MQTT context pointer.
 * @param[in] xCleanSession If a clean session should be established.
 *
 * @return `MQTTSuccess` if connection succeeds, else appropriate error code
 * from MQTT_Connect.
 */
static MQTTStatus_t prvMQTTConnect( bool xCleanSession );

/**
 * @brief Connect a TCP socket to the MQTT broker.
 *
 * @param[in] pxNetworkContext Network context.
 *
 * @return `pdPASS` if connection succeeds, else `pdFAIL`.
 */
static BaseType_t prvSocketConnect( NetworkContext_t * pxNetworkContext );

/**
 * @brief Disconnect a TCP connection.
 *
 * @param[in] pxNetworkContext Network context.
 *
 * @return `pdPASS` if disconnect succeeds, else `pdFAIL`.
 */
static BaseType_t prvSocketDisconnect( NetworkContext_t * pxNetworkContext );

/**
 * @brief Callback executed when there is activity on the TCP socket that is
 * connected to the MQTT broker.  If there are no messages in the MQTT agent's
 * command queue then the callback send a message to ensure the MQTT agent
 * task unblocks and can therefore process whatever is necessary on the socket
 * (if anything) as quickly as possible.
 *
 * @param[in] pxSocket Socket with data, unused.
 */
static void prvMQTTClientSocketWakeupCallback( Socket_t pxSocket );

/**
 * @brief Logs any incoming publish messages received on topics to which there
 * are no subscriptions.  This can happen if the MQTT broker sends control
 * information to the MQTT client on special control topics.
 *
 * @param[in] pxPublishInfo Info of incoming publish.
 * @param[in] The context specified when the MQTT connection was created.
 */
static void prvUnsolicitedIncomingPublishCallback( MQTTPublishInfo_t * pxPublishInfo,
                                                   void * pvContext );


/**
 * @brief Task used to run the MQTT agent.  In this example the first task that
 * is created is responsible for creating all the other demo tasks.  Then,
 * rather than create prvMQTTAgentTask() as a separate task, it simply calls
 * prvMQTTAgentTask() to become the agent task itself.
 *
 * This task calls MQTTAgent_CommandLoop() in a loop, until MQTTAgent_Terminate()
 * is called. If an error occurs in the command loop, then it will reconnect the
 * TCP and MQTT connections.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvMQTTAgentTask( void * pvParameters );

/**
 * @brief The main task used in the MQTT demo.
 *
 * After creating the publisher and subscriber tasks, this task will enter a
 * loop, processing commands from the command queue and calling the MQTT API.
 * After the termination command is received on the command queue, the task
 * will break from the loop.
 *
 * @param[in] pvParameters Parameters as passed at the time of task creation. Not
 * used in this example.
 */
static void prvConnectAndCreateDemoTasks( void * pvParameters );

/**
 * @brief The timer query function provided to the MQTT context.
 *
 * @return Time in milliseconds.
 */
static uint32_t prvGetTimeMs( void );

/**
 * @brief Connects a TCP socket to the MQTT broker, then creates and MQTT
 * connection to the same.
 */
static void prvConnectToMQTTBroker( void );

/*
 * Functions that start the tasks demonstrated by this project.
 */

extern void vStartLargeMessageSubscribePublishTask( configSTACK_DEPTH_TYPE uxStackSize,
                                                    UBaseType_t uxPriority );
extern void vStartSimpleSubscribePublishTask( uint32_t ulTaskNumber,
                                              configSTACK_DEPTH_TYPE uxStackSize,
                                              UBaseType_t uxPriority );

extern void vStartOTACodeSigningDemo( configSTACK_DEPTH_TYPE uxStackSize,
                                      UBaseType_t uxPriority );
extern void vSuspendOTACodeSigningDemo( void );
extern void vResumeOTACodeSigningDemo( void );
/*-----------------------------------------------------------*/

/**
 * @brief The network context used by the MQTT library transport interface.
 * See https://www.freertos.org/network-interface.html
 */
static NetworkContext_t xNetworkContext;

/**
 * @brief Global entry time into the application to use as a reference timestamp
 * in the #prvGetTimeMs function. #prvGetTimeMs will always return the difference
 * between the current time and the global entry time. This will reduce the chances
 * of overflow for the 32 bit unsigned integer used for holding the timestamp.
 */
static uint32_t ulGlobalEntryTimeMs;

/*-----------------------------------------------------------*/

/*
 * @brief Create the task that demonstrates the MQTT Connection sharing demo.
 */
void vStartSimpleMQTTDemo( void )
{
    /* prvConnectAndCreateDemoTasks() connects to the MQTT broker, creates the
     * tasks that will interact with the broker via the MQTT agent, then turns
     * itself into the MQTT agent task. */
    xTaskCreate( prvConnectAndCreateDemoTasks, /* Function that implements the task. */
                 "ConnectManager",             /* Text name for the task - only used for debugging. */
                 democonfigDEMO_STACKSIZE,     /* Size of stack (in words, not bytes) to allocate for the task. */
                 NULL,                         /* Optional - task parameter - not used in this case. */
                 tskIDLE_PRIORITY + 1,         /* Task priority, must be between 0 and configMAX_PRIORITIES - 1. */
                 NULL );                       /* Optional - used to pass out a handle to the created task. */
}

/*-----------------------------------------------------------*/

static MQTTStatus_t prvMQTTInit( void )
{
    TransportInterface_t xTransport;
    MQTTStatus_t xReturn;
    const MQTTContextHandle_t xGlobalMQTTContextHandle = 0;

    /* Fill in Transport Interface send and receive function pointers. */
    xTransport.pNetworkContext = &xNetworkContext;
    #if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 )
        xTransport.send = TLS_FreeRTOS_send;
        xTransport.recv = TLS_FreeRTOS_recv;
    #else
        xTransport.send = Plaintext_FreeRTOS_send;
        xTransport.recv = Plaintext_FreeRTOS_recv;
    #endif

    /* Initialize MQTT library. */
    xReturn = MQTTAgent_Init( xGlobalMQTTContextHandle,
                              &xTransport,
                              prvGetTimeMs,

                              /* Callback to execute if receiving publishes on
                               * topics for which there is no subscription. */
                              prvUnsolicitedIncomingPublishCallback,
                              /* Context to pass into the callback.  Not used. */
                              NULL );

    return xReturn;
}

/*-----------------------------------------------------------*/

static MQTTStatus_t prvMQTTConnect( bool xCleanSession )
{
    MQTTStatus_t xResult;
    MQTTConnectInfo_t xConnectInfo;
    bool xSessionPresent = false;

    /* Many fields are not used in this demo so start with everything at 0. */
    memset( &xConnectInfo, 0x00, sizeof( xConnectInfo ) );

    /* Start with a clean session i.e. direct the MQTT broker to discard any
     * previous session data. Also, establishing a connection with clean session
     * will ensure that the broker does not store any data when this client
     * gets disconnected. */
    xConnectInfo.cleanSession = xCleanSession;

    /* The client identifier is used to uniquely identify this MQTT client to
     * the MQTT broker. In a production device the identifier can be something
     * unique, such as a device serial number. */
    xConnectInfo.pClientIdentifier = democonfigCLIENT_IDENTIFIER;
    xConnectInfo.clientIdentifierLength = ( uint16_t ) strlen( democonfigCLIENT_IDENTIFIER );

    /* Set MQTT keep-alive period. It is the responsibility of the application
     * to ensure that the interval between Control Packets being sent does not
     * exceed the Keep Alive value. In the absence of sending any other Control
     * Packets, the Client MUST send a PINGREQ Packet.  This responsibility will
     * be moved inside the agent. */
    xConnectInfo.keepAliveSeconds = mqttexampleKEEP_ALIVE_INTERVAL_SECONDS;

    /* Append metrics when connecting to the AWS IoT Core broker. */
    #ifdef democonfigUSE_AWS_IOT_CORE_BROKER
        #ifdef democonfigCLIENT_USERNAME
            xConnectInfo.pUserName = CLIENT_USERNAME_WITH_METRICS;
            xConnectInfo.userNameLength = ( uint16_t ) strlen( CLIENT_USERNAME_WITH_METRICS );
            xConnectInfo.pPassword = democonfigCLIENT_PASSWORD;
            xConnectInfo.passwordLength = ( uint16_t ) strlen( democonfigCLIENT_PASSWORD );
        #else
            xConnectInfo.pUserName = AWS_IOT_METRICS_STRING;
            xConnectInfo.userNameLength = AWS_IOT_METRICS_STRING_LENGTH;
            /* Password for authentication is not used. */
            xConnectInfo.pPassword = NULL;
            xConnectInfo.passwordLength = 0U;
        #endif
    #else /* ifdef democonfigUSE_AWS_IOT_CORE_BROKER */
        #ifdef democonfigCLIENT_USERNAME
            xConnectInfo.pUserName = democonfigCLIENT_USERNAME;
            xConnectInfo.userNameLength = ( uint16_t ) strlen( democonfigCLIENT_USERNAME );
            xConnectInfo.pPassword = democonfigCLIENT_PASSWORD;
            xConnectInfo.passwordLength = ( uint16_t ) strlen( democonfigCLIENT_PASSWORD );
        #endif /* ifdef democonfigCLIENT_USERNAME */
    #endif /* ifdef democonfigUSE_AWS_IOT_CORE_BROKER */

    /* Send MQTT CONNECT packet to broker. MQTT's Last Will and Testament feature
     * is not used in this demo, so it is passed as NULL. */
    xResult = MQTTAgent_Connect( mqttexampleMQTT_CONTEXT_HANDLE,
                                 &xConnectInfo,
                                 NULL,
                                 mqttexampleCONNACK_RECV_TIMEOUT_MS,
                                 &xSessionPresent );

    LogInfo( ( "Session present: %d\n", xSessionPresent ) );

    /* Resume a session if desired. */
    if( ( xResult == MQTTSuccess ) && !xCleanSession )
    {
        xResult = MQTTAgent_ResumeSession( mqttexampleMQTT_CONTEXT_HANDLE, xSessionPresent );
    }

    return xResult;
}

/*-----------------------------------------------------------*/

static BaseType_t prvSocketConnect( NetworkContext_t * pxNetworkContext )
{
    BaseType_t xConnected = pdFAIL;
    RetryUtilsStatus_t xRetryUtilsStatus = RetryUtilsSuccess;
    RetryUtilsParams_t xReconnectParams;
    const TickType_t xTransportTimeout = 0UL;

    #if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 )
        TlsTransportStatus_t xNetworkStatus = TLS_TRANSPORT_CONNECT_FAILURE;
        NetworkCredentials_t xNetworkCredentials = { 0 };

        #ifdef democonfigUSE_AWS_IOT_CORE_BROKER

            /* ALPN protocols must be a NULL-terminated list of strings. Therefore,
             * the first entry will contain the actual ALPN protocol string while the
             * second entry must remain NULL. */
            char * pcAlpnProtocols[] = { NULL, NULL };

            /* The ALPN string changes depending on whether username/password authentication is used. */
            #ifdef democonfigCLIENT_USERNAME
                pcAlpnProtocols[ 0 ] = AWS_IOT_CUSTOM_AUTH_ALPN;
            #else
                pcAlpnProtocols[ 0 ] = AWS_IOT_MQTT_ALPN;
            #endif
            xNetworkCredentials.pAlpnProtos = ( const char ** ) pcAlpnProtocols;
        #endif /* ifdef democonfigUSE_AWS_IOT_CORE_BROKER */

        /* Set the credentials for establishing a TLS connection. */
        xNetworkCredentials.pRootCa = ( const unsigned char * ) democonfigROOT_CA_PEM;
        xNetworkCredentials.rootCaSize = sizeof( democonfigROOT_CA_PEM );
        #ifdef democonfigCLIENT_CERTIFICATE_PEM
            xNetworkCredentials.pClientCert = ( const unsigned char * ) democonfigCLIENT_CERTIFICATE_PEM;
            xNetworkCredentials.clientCertSize = sizeof( democonfigCLIENT_CERTIFICATE_PEM );
            xNetworkCredentials.pPrivateKey = ( const unsigned char * ) democonfigCLIENT_PRIVATE_KEY_PEM;
            xNetworkCredentials.privateKeySize = sizeof( democonfigCLIENT_PRIVATE_KEY_PEM );
        #endif
        xNetworkCredentials.disableSni = democonfigDISABLE_SNI;
    #else /* if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 ) */
        PlaintextTransportStatus_t xNetworkStatus = PLAINTEXT_TRANSPORT_CONNECT_FAILURE;
    #endif /* if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 ) */

    /* We will use a retry mechanism with an exponential backoff mechanism and
     * jitter.  That is done to prevent a fleet of IoT devices all trying to
     * reconnect at exactly the same time should they become disconnected at
     * the same time. We initialize reconnect attempts and interval here. */
    xReconnectParams.maxRetryAttempts = MAX_RETRY_ATTEMPTS;
    RetryUtils_ParamsReset( &xReconnectParams );

    /* Attempt to connect to MQTT broker. If connection fails, retry after a
     * timeout. Timeout value will exponentially increase until the maximum
     * number of attempts are reached.
     */
    do
    {
        /* Establish a TCP connection with the MQTT broker. This example connects to
         * the MQTT broker as specified in democonfigMQTT_BROKER_ENDPOINT and
         * democonfigMQTT_BROKER_PORT at the top of this file. */
        #if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 )
            LogInfo( ( "Creating a TLS connection to %s:%d.",
                       democonfigMQTT_BROKER_ENDPOINT,
                       democonfigMQTT_BROKER_PORT ) );
            xNetworkStatus = TLS_FreeRTOS_Connect( pxNetworkContext,
                                                   democonfigMQTT_BROKER_ENDPOINT,
                                                   democonfigMQTT_BROKER_PORT,
                                                   &xNetworkCredentials,
                                                   mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS,
                                                   mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS );
            xConnected = ( xNetworkStatus == TLS_TRANSPORT_SUCCESS ) ? pdPASS : pdFAIL;
        #else /* if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 ) */
            LogInfo( ( "Creating a TCP connection to %s:%d.",
                       democonfigMQTT_BROKER_ENDPOINT,
                       democonfigMQTT_BROKER_PORT ) );
            xNetworkStatus = Plaintext_FreeRTOS_Connect( pxNetworkContext,
                                                         democonfigMQTT_BROKER_ENDPOINT,
                                                         democonfigMQTT_BROKER_PORT,
                                                         mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS,
                                                         mqttexampleTRANSPORT_SEND_RECV_TIMEOUT_MS );
            xConnected = ( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS ) ? pdPASS : pdFAIL;
        #endif /* if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 ) */

        if( !xConnected )
        {
            LogWarn( ( "Connection to the broker failed. Retrying connection with backoff and jitter." ) );
            xRetryUtilsStatus = RetryUtils_BackoffAndSleep( &xReconnectParams );
        }

        if( xRetryUtilsStatus == RetryUtilsRetriesExhausted )
        {
            LogError( ( "Connection to the broker failed. All attempts exhausted." ) );
        }
    } while( ( xConnected != pdPASS ) && ( xRetryUtilsStatus == RetryUtilsSuccess ) );

    /* Set the socket wakeup callback and ensure the read block time. */
    if( xConnected )
    {
        ( void ) FreeRTOS_setsockopt( pxNetworkContext->tcpSocket,
                                      0, /* Level - Unused. */
                                      FREERTOS_SO_WAKEUP_CALLBACK,
                                      ( void * ) prvMQTTClientSocketWakeupCallback,
                                      sizeof( &( prvMQTTClientSocketWakeupCallback ) ) );

        ( void ) FreeRTOS_setsockopt( pxNetworkContext->tcpSocket,
                                      0,
                                      FREERTOS_SO_RCVTIMEO,
                                      &xTransportTimeout,
                                      sizeof( TickType_t ) );
    }

    return xConnected;
}

/*-----------------------------------------------------------*/

static BaseType_t prvSocketDisconnect( NetworkContext_t * pxNetworkContext )
{
    BaseType_t xDisconnected = pdFAIL;

    /* Set the wakeup callback to NULL since the socket will disconnect. */
    ( void ) FreeRTOS_setsockopt( pxNetworkContext->tcpSocket,
                                  0, /* Level - Unused. */
                                  FREERTOS_SO_WAKEUP_CALLBACK,
                                  ( void * ) NULL,
                                  sizeof( void * ) );

    #if defined( democonfigUSE_TLS ) && ( democonfigUSE_TLS == 1 )
        LogInfo( ( "Disconnecting TLS connection.\n" ) );
        TLS_FreeRTOS_Disconnect( pxNetworkContext );
        xDisconnected = pdPASS;
    #else
        LogInfo( ( "Disconnecting TCP connection.\n" ) );
        PlaintextTransportStatus_t xNetworkStatus = PLAINTEXT_TRANSPORT_CONNECT_FAILURE;
        xNetworkStatus = Plaintext_FreeRTOS_Disconnect( pxNetworkContext );
        xDisconnected = ( xNetworkStatus == PLAINTEXT_TRANSPORT_SUCCESS ) ? pdPASS : pdFAIL;
    #endif
    return xDisconnected;
}

/*-----------------------------------------------------------*/

static void prvMQTTClientSocketWakeupCallback( Socket_t pxSocket )
{
    /* Just to avoid compiler warnings.  The socket is not used but the function
     * prototype cannot be changed because this is a callback function. */
    ( void ) pxSocket;

    /* A socket used by the MQTT task may need attention.  Send an event
     * to the MQTT task to make sure the task is not blocked on xCommandQueue. */
    if( ( MQTTAgent_GetNumWaiting() == 0U ) && ( FreeRTOS_recvcount( pxSocket ) > 0 ) )
    {
        /* Don't block as this is called from the context of the IP task. */
        MQTTAgent_TriggerProcessLoop( mqttexampleMQTT_CONTEXT_HANDLE, 0 );
    }
}

/*-----------------------------------------------------------*/

static void prvUnsolicitedIncomingPublishCallback( MQTTPublishInfo_t * pxPublishInfo,
                                                   void * pvNotUsed )
{
    char cOriginalChar, * pcLocation;

    ( void ) pvNotUsed;

    /* Ensure the topic string is terminated for printing.  This will over-
     * write the message ID, which is restored afterwards. */
    pcLocation = ( char * ) &( pxPublishInfo->pTopicName[ pxPublishInfo->topicNameLength ] );
    cOriginalChar = *pcLocation;
    *pcLocation = 0x00;
    LogWarn( ( "WARN:  Received an unsolicited publish from topic %s", pxPublishInfo->pTopicName ) );
    *pcLocation = cOriginalChar;
}

/*-----------------------------------------------------------*/

static void prvMQTTAgentTask( void * pvParameters )
{
    BaseType_t xNetworkResult = pdFAIL;
    MQTTStatus_t xMQTTStatus = MQTTSuccess;
    MQTTContext_t * pMqttContext = NULL;

    ( void ) pvParameters;

    do
    {
        /* MQTTAgent_CommandLoop() is effectively the agent implementation.  It
         * will manage the MQTT protocol until such time that an error occurs,
         * which could be a disconnect.  If an error occurs the MQTT context on
         * which the error happened is returned so there can be an attempt to
         * clean up and reconnect however the application writer prefers. */
        pMqttContext = MQTTAgent_CommandLoop();

        /* Context is only returned if error occurred which will may have
         * disconnected the socket from the MQTT broker already. */
        if( pMqttContext != NULL )
        {
            #if ( democonfigCREATE_CODE_SIGNING_OTA_DEMO == 1 )
                {
                    vSuspendOTACodeSigningDemo();
                }
            #endif

            /* Reconnect TCP. */
            xNetworkResult = prvSocketDisconnect( &xNetworkContext );
            configASSERT( xNetworkResult == pdPASS );
            xNetworkResult = prvSocketConnect( &xNetworkContext );
            configASSERT( xNetworkResult == pdPASS );
            pMqttContext->connectStatus = MQTTNotConnected;
            /* MQTT Connect with a persistent session. */
            xMQTTStatus = prvMQTTConnect( false );

            #if ( democonfigCREATE_CODE_SIGNING_OTA_DEMO == 1 )
                {
                    if( xMQTTStatus == MQTTSuccess )
                    {
                        vResumeOTACodeSigningDemo();
                    }
                }
            #else
                ( void ) xMQTTStatus;
            #endif
        }
    } while( pMqttContext );
}

/*-----------------------------------------------------------*/

static void prvConnectToMQTTBroker( void )
{
    BaseType_t xNetworkStatus = pdFAIL;
    MQTTStatus_t xMQTTStatus;

    /* Connect a TCP socket to the broker. */
    xNetworkStatus = prvSocketConnect( &xNetworkContext );
    configASSERT( xNetworkStatus == pdPASS );

    /* Initialise the MQTT context with the buffer and transport interface. */
    xMQTTStatus = prvMQTTInit();
    configASSERT( xMQTTStatus == MQTTSuccess );

    /* Form an MQTT connection without a persistent session. */
    xMQTTStatus = prvMQTTConnect( true );
    configASSERT( xMQTTStatus == MQTTSuccess );
}
/*-----------------------------------------------------------*/

static void prvConnectAndCreateDemoTasks( void * pvParameters )
{
    ( void ) pvParameters;

    /* Miscellaneous initialisation. */
    ulGlobalEntryTimeMs = prvGetTimeMs();

    /* Create the TCP connection to the broker, then the MQTT connection to the
     * same. */
    prvConnectToMQTTBroker();

    /* Selectively create demo tasks as per the compile time constant settings. */
    #if ( democonfigCREATE_LARGE_MESSAGE_SUB_PUB_TASK == 1 )
        {
            vStartLargeMessageSubscribePublishTask( democonfigLARGE_MESSAGE_SUB_PUB_TASK_STACK_SIZE,
                                                    tskIDLE_PRIORITY );
        }
    #endif

    #if ( democonfigNUM_SIMPLE_SUB_PUB_TASKS_TO_CREATE > 0 )
        {
            vStartSimpleSubscribePublishTask( democonfigNUM_SIMPLE_SUB_PUB_TASKS_TO_CREATE,
                                              democonfigSIMPLE_SUB_PUB_TASK_STACK_SIZE,
                                              tskIDLE_PRIORITY );
        }
    #endif

    #if ( democonfigCREATE_CODE_SIGNING_OTA_DEMO == 1 )
        {
            vStartOTACodeSigningDemo( democonfigCODE_SIGNING_OTA_TASK_STACK_SIZE,
                                      tskIDLE_PRIORITY + 1 );
        }
    #endif

    /* This task has nothing left to do, so rather than create the MQTT
     * agent as a separate thread, it simply calls the function that implements
     * the agent - in effect turning itself into the agent. */
    prvMQTTAgentTask( NULL );

    /* Should not get here.  Force an assert if the task returns from
     * prvMQTTAgentTask(). */
    configASSERT( pvParameters == ( void * ) ~1 );
}

/*-----------------------------------------------------------*/

static uint32_t prvGetTimeMs( void )
{
    TickType_t xTickCount = 0;
    uint32_t ulTimeMs = 0UL;

    /* Get the current tick count. */
    xTickCount = xTaskGetTickCount();

    /* Convert the ticks to milliseconds. */
    ulTimeMs = ( uint32_t ) xTickCount * mqttexampleMILLISECONDS_PER_TICK;

    /* Reduce ulGlobalEntryTimeMs from obtained time so as to always return the
     * elapsed time in the application. */
    ulTimeMs = ( uint32_t ) ( ulTimeMs - ulGlobalEntryTimeMs );

    return ulTimeMs;
}

/*-----------------------------------------------------------*/
