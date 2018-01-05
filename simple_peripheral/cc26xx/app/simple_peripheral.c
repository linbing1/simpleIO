/******************************************************************************

 @file  simple_peripheral.c

 @brief This file contains the Simple BLE Peripheral sample application for use
        with the CC2650 Bluetooth Low Energy Protocol Stack.

 Group: WCS, BTS
 Target Device: CC2650, CC2640, CC1350

 ******************************************************************************
 
 Copyright (c) 2013-2016, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 ******************************************************************************
 Release Name: ble_sdk_2_02_01_18
 Release Date: 2016-10-26 15:20:04
 *****************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <string.h>
#include <stdio.h>

#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>

#include "hal_board.h"
#include "hci_tl.h"
#include "gatt.h"
#include "linkdb.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simple_gatt_profile.h"

#if defined(FEATURE_OAD) || defined(IMAGE_INVALIDATE)
#include "oad_target.h"
#include "oad.h"
#endif //FEATURE_OAD || IMAGE_INVALIDATE

#include "peripheral.h"
#include "gapbondmgr.h"

#include "osal_snv.h"
#include "icall_apimsg.h"

#include "util.h"

#ifdef USE_RCOSC
#include "rcosc_calibration.h"
#endif //USE_RCOSC

#include <ti/mw/display/Display.h>
#include "board_key.h"

/* TI-RTOS Header files */
#include <ti/drivers/UART.h>
#include <ti/drivers/Watchdog.h>

#include "ExtFlash.h"

#include "board.h"

#include "simple_peripheral.h"

/*********************************************************************
 * CONSTANTS
 */
#define UART_ENABLE

// Advertising interval when device is discoverable (units of 625us, 160=100ms)
#define DEFAULT_ADVERTISING_INTERVAL 1600

// Limited discoverable mode advertises for 30.72s, and then stops
// General discoverable mode advertises indefinitely
#define DEFAULT_DISCOVERABLE_MODE GAP_ADTYPE_FLAGS_GENERAL

#ifndef FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 80=100ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL 6

// Maximum connection interval (units of 1.25ms, 800=1000ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL 80
#else //!FEATURE_OAD
// Minimum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MIN_CONN_INTERVAL 6

// Maximum connection interval (units of 1.25ms, 8=10ms) if automatic
// parameter update request is enabled
#define DEFAULT_DESIRED_MAX_CONN_INTERVAL 80
#endif // FEATURE_OAD

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_DESIRED_SLAVE_LATENCY 0

// Supervision timeout value (units of 10ms, 1000=10s) if automatic parameter
// update request is enabled
#define DEFAULT_DESIRED_CONN_TIMEOUT 600

// Whether to enable automatic parameter update request when a connection is
// formed
#define DEFAULT_ENABLE_UPDATE_REQUEST GAPROLE_LINK_PARAM_UPDATE_INITIATE_BOTH_PARAMS

// Connection Pause Peripheral time value (in seconds)
#define DEFAULT_CONN_PAUSE_PERIPHERAL 5

// How often to perform periodic event (in msec)
#define SBP_PERIODIC_EVT_PERIOD (1000 * 3)
#define SBP_DISCONNECT_EVT_PERIOD 30000
#define SBP_BUZZER_EVT_PERIOD 200
#define SBP_LED_EVT_PERIOD 200
#define SBP_SHORTPRESS_EVT_PERIOD 50
#define SBP_LONGPRESS_EVT_PERIOD 5000

#ifdef FEATURE_OAD
// The size of an OAD packet.
#define OAD_PACKET_SIZE ((OAD_BLOCK_SIZE) + 2)
#endif // FEATURE_OAD

// Task configuration
#define SBP_TASK_PRIORITY 1

#ifndef SBP_TASK_STACK_SIZE
#define SBP_TASK_STACK_SIZE 644
#endif

// Internal Events for RTOS application
#define SBP_STATE_CHANGE_EVT 0x0001
#define SBP_CHAR_CHANGE_EVT 0x0002
#define SBP_CONN_EVT_END_EVT 0x0004
#define SBP_ADV_CB_EVT 0x0008

#define SBP_BUZZER_EVT 0x0010
#define SBP_LED_EVT 0x0020
#define SBP_SHORTPRESS_EVT 0x0040
#define SBP_LONGPRESS_EVT 0x0080

#define SBP_PASSCODE_EVT 0x0100
#define SBP_PAIRING_EVT 0x0200
#define SBP_PERIODIC_EVT 0x0400
#define SBP_DISCONNECT_EVT 0x0800

#define SBP_BUTTON1_EVT 0x1000
#define SBP_BUTTON2_EVT 0x2000
#define SBP_BUTTON3_EVT 0x4000
#define SBP_BUTTON4_EVT 0x8000

/*********************************************************************
 * TYPEDEFS
 */

// App event passed from profiles.
typedef struct
{
    appEvtHdr_t hdr; // event header.
    uint8_t *pData;  // Event data
} sbpEvt_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Display Interface
Display_Handle dispHandle = NULL;

/*********************************************************************
 * LOCAL VARIABLES
 */

/* Pin driver handles */
static PIN_Handle buttonPinHandle;
static PIN_Handle ledPinHandle;

/* Global memory storage for a PIN_Config table */
static PIN_State buttonPinState;
static PIN_State ledPinState;

PIN_Config ledPinTable[] = {
    LED1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED5 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED6 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED7 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    LED8 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    OUT1 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    OUT2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    OUT3 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    OUT4 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    BUZZER | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE};

PIN_Config buttonPinTable[] = {
    ID0 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS,
    ID1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_DIS,
    BUTTON1 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
    BUTTON2 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
    BUTTON3 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
    BUTTON4 | PIN_INPUT_EN | PIN_PULLUP | PIN_IRQ_BOTHEDGES,
    PIN_TERMINATE};

#ifdef UART_ENABLE
UART_Handle uartHandle;
UART_Params uartParams;
#endif

Watchdog_Handle wdtHandle;
Watchdog_Params wdtParams;
void wdtCallback(UArg a0)
{
    //Watchdog_clear(wdtHandle);
}

static uint8_t idValue = 0;
static uint8_t ledCount = 0;
static uint32_t outValue = 0;
static uint16_t version = 5;

static PIN_Id buttonCBPinId = BUTTON1;

#define SNV_BUF_LEN 4
#define SNV_ID_APP 0x80
static uint8 snv_buf[SNV_BUF_LEN] = {0x00, 0x0D, 0x90, 0x38}; //888888
#define SNV_ID_CONFIG 0x81
#define SNV_ID_RESET 0x82
static uint8_t resetValue = 0;

// Entity ID globally used to check for source and/or destination of messages
static ICall_EntityID selfEntity;

// Semaphore globally used to post events to the application thread
static ICall_Semaphore sem;

// Clock instances for internal periodic events.
static Clock_Struct periodicClock;
static Clock_Struct disconnectClock;
static Clock_Struct buzzerClock;
static Clock_Struct ledClock;
static Clock_Struct shortpressClock;
static Clock_Struct longpressClock;
static Clock_Struct button1Clock;
static Clock_Struct button2Clock;
static Clock_Struct button3Clock;
static Clock_Struct button4Clock;

// Queue object used for app messages
static Queue_Struct appMsg;
static Queue_Handle appMsgQueue;

#if defined(FEATURE_OAD)
// Event data from OAD profile.
static Queue_Struct oadQ;
static Queue_Handle hOadQ;
#endif //FEATURE_OAD

// events flag for internal application events.
static uint16_t events;

// Task configuration
Task_Struct sbpTask;
Char sbpTaskStack[SBP_TASK_STACK_SIZE];

// Profile state and parameters
static gaprole_States_t gapProfileState = GAPROLE_INIT;

// GAP - SCAN RSP data (max size = 31 bytes)
static uint8_t scanRspData[] =
{
    // complete name
    0x0D, // length of this data
    GAP_ADTYPE_LOCAL_NAME_COMPLETE,
    'L',
    'O',
    'L',
    'A',
    'R',
    '_',
    'S',
    'W',
    'I',
    'T',
    'C',
    'H',

    // connection interval range
    0x05, // length of this data
    GAP_ADTYPE_SLAVE_CONN_INTERVAL_RANGE,
    LO_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MIN_CONN_INTERVAL),
    LO_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),
    HI_UINT16(DEFAULT_DESIRED_MAX_CONN_INTERVAL),

    // Tx power level
    0x02, // length of this data
    GAP_ADTYPE_POWER_LEVEL,
    0 // 0dBm
};

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertisting)
static uint8_t advertData[] =
{
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    DEFAULT_DISCOVERABLE_MODE | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,
};

// GAP GATT Attributes
static uint8_t attDeviceName[GAP_DEVICE_NAME_LEN] = "LOLAR_SWITCH";

// Globals used for ATT Response retransmission
static gattMsgEvent_t *pAttRsp = NULL;
static uint8_t rspTxRetry = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */

static void SimpleBLEPeripheral_init(void);
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1);

static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg);
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg);
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg);
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState);
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID);
static void SimpleBLEPeripheral_performPeriodicTask(void);
static void SimpleBLEPeripheral_clockHandler(UArg arg);

static void SimpleBLEPeripheral_sendAttRsp(void);
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status);

static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState);
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID);
static void SimpleBLEPeripheral_enqueueMsg(uint16_t event, uint8_t state, uint8_t *pData);

#ifdef FEATURE_OAD
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData);
#endif //FEATURE_OAD

// Passcode.
static void Sbp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                           uint8_t uiInputs, uint8_t uiOutputs);
static void Sbp_processPasscodeEvt(uint16_t connHandle);

// Pairing state.
static void Sbp_pairStateCB(uint16_t connHandle, uint8_t state,
                            uint8_t status);
static void Sbp_processPairStateEvt(uint8_t state, uint8_t status);

static void SetLEDByOut(PIN_Id out)
{
    uint32_t currVal = 0;
    currVal = PIN_getOutputValue(out);
    if (out == OUT1)
    {
        if (idValue == 0)
        {
            if ((outValue & 0x0100) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED3, !currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED3, 0);
            }
            if ((outValue & 0x0200) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED4, currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED4, 0);
            }
        }
        else if (idValue == 1 || idValue == 2 || idValue == 3)
        {
            if ((outValue & 0x0100) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED1, !currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED1, 0);
            }
            if ((outValue & 0x0200) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED2, currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED2, 0);
            }
        }
    }
    else if (out == OUT2)
    {
        if (idValue == 1)
        {
            if ((outValue & 0x0100) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED5, !currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED5, 0);
            }
            if ((outValue & 0x0200) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED6, currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED6, 0);
            }
        }
        else if (idValue == 2 || idValue == 3)
        {
            if ((outValue & 0x0100) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED3, !currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED3, 0);
            }
            if ((outValue & 0x0200) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED4, currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED4, 0);
            }
        }
    }
    else if (out == OUT3)
    {
        if (idValue == 2 || idValue == 3)
        {
            if ((outValue & 0x0100) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED5, !currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED5, 0);
            }
            if ((outValue & 0x0200) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED6, currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED6, 0);
            }
        }
    }
    else if (out == OUT4)
    {
        if (idValue == 3)
        {
            if ((outValue & 0x0100) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED7, !currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED7, 0);
            }
            if ((outValue & 0x0200) == 0x0000)
            {
                PIN_setOutputValue(ledPinHandle, LED8, currVal);
            }
            else
            {
                PIN_setOutputValue(ledPinHandle, LED8, 0);
            }
        }
    }
}

static void ButtonPress(PIN_Id out)
{
    uint32_t currVal = 0;
    currVal = PIN_getOutputValue(out);
    PIN_setOutputValue(ledPinHandle, out, !currVal);
    SetLEDByOut(out);
    outValue ^= (1 << (out - OUT1));
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                               &outValue);
    uint8_t notiValue = outValue & 0x0F;
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                               &notiValue);

    if ((outValue & 0x0400) == 0x0000)
    {
        PIN_setOutputValue(ledPinHandle, BUZZER, 1);
        Util_startClock(&buzzerClock);
    }
}

static void ButtonCallback(PIN_Id pinId)
{
    if (!PIN_getInputValue(pinId))
    {
        switch (pinId)
        {
        case BUTTON1:
            if (idValue == 0)
            {
                ButtonPress(OUT1);
                Util_startClock(&longpressClock);
            }
            else if (idValue == 2 || idValue == 3)
            {
                ButtonPress(OUT2);
                Util_startClock(&longpressClock);
            }
            break;

        case BUTTON2:
            if (idValue == 1)
            {
                ButtonPress(OUT2);
            }
            else if (idValue == 2 || idValue == 3)
            {
                ButtonPress(OUT3);
            }
            break;

        case BUTTON3:
            if (idValue == 3)
            {
                ButtonPress(OUT4);
            }
            break;

        case BUTTON4:
            if (idValue == 2 || idValue == 3)
            {
                ButtonPress(OUT1);
            }
            if (idValue == 1)
            {
                ButtonPress(OUT1);
                Util_startClock(&longpressClock);
            }
            break;

        default:
            /* Do nothing */
            break;
        }
    }
    else
    {
        switch (pinId)
        {
        case BUTTON1:
            if (idValue == 0 || idValue == 2 || idValue == 3)
            {
                Util_stopClock(&longpressClock);
            }
            break;
        case BUTTON4:
            if (idValue == 1)
            {
                Util_stopClock(&longpressClock);
            }
            break;

        default:
            break;
        }
    }
}

static void ButtonCallbackFxn(PIN_Handle handle, PIN_Id pinId)
{
    /*
    if (!Util_isActive(&shortpressClock))
    {
        buttonCBPinId = pinId;
        Util_startClock(&shortpressClock);
    }
    */
    if (pinId == BUTTON1 && !Util_isActive(&button1Clock))
    {
        Util_startClock(&button1Clock);
    }
    if (pinId == BUTTON2 && !Util_isActive(&button2Clock))
    {
        Util_startClock(&button2Clock);
    }
    if (pinId == BUTTON3 && !Util_isActive(&button3Clock))
    {
        Util_startClock(&button3Clock);
    }
    if (pinId == BUTTON4 && !Util_isActive(&button4Clock))
    {
        Util_startClock(&button4Clock);
    }
}

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static gapRolesCBs_t SimpleBLEPeripheral_gapRoleCBs =
{
    SimpleBLEPeripheral_stateChangeCB // Profile State Change Callbacks
};

// GAP Bond Manager Callbacks
static gapBondCBs_t simpleBLEPeripheral_BondMgrCBs =
{
    (pfnPasscodeCB_t)Sbp_passcodeCB,
    Sbp_pairStateCB};

// Simple GATT Profile Callbacks
static simpleProfileCBs_t SimpleBLEPeripheral_simpleProfileCBs =
{
    SimpleBLEPeripheral_charValueChangeCB // Characteristic value change callback
};

#ifdef FEATURE_OAD
static oadTargetCBs_t simpleBLEPeripheral_oadCBs =
{
    SimpleBLEPeripheral_processOadWriteCB // Write Callback.
};
#endif //FEATURE_OAD

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLEPeripheral_createTask
 *
 * @brief   Task creation function for the Simple BLE Peripheral.
 *
 * @param   None.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_createTask(void)
{
    Task_Params taskParams;

    // Configure task
    Task_Params_init(&taskParams);
    taskParams.stack = sbpTaskStack;
    taskParams.stackSize = SBP_TASK_STACK_SIZE;
    taskParams.priority = SBP_TASK_PRIORITY;

    Task_construct(&sbpTask, SimpleBLEPeripheral_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_init
 *
 * @brief   Called during initialization and contains application
 *          specific initialization (ie. hardware initialization/setup,
 *          table initialization, power up notification, etc), and
 *          profile initialization/setup.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_init(void)
{
    // ******************************************************************
    // N0 STACK API CALLS CAN OCCUR BEFORE THIS CALL TO ICall_registerApp
    // ******************************************************************
    // Register the current thread as an ICall dispatcher application
    // so that the application can send and receive messages.
    ICall_registerApp(&selfEntity, &sem);

#ifdef USE_RCOSC
    RCOSC_enableCalibration();
#endif // USE_RCOSC

#ifdef UART_ENABLE
    UART_Params_init(&uartParams);
    uartParams.baudRate = 115200;
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartHandle = UART_open(Board_UART0, &uartParams);
    if (uartHandle == NULL)
    {
        //System_abort("Error opening the UART");
    }
#endif

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, ledPinTable);
    if (!ledPinHandle)
    {
        //System_abort("Error initializing board LED pins\n");
    }
    PIN_setOutputValue(ledPinHandle, LED1, 0);
    PIN_setOutputValue(ledPinHandle, LED2, 0);
    PIN_setOutputValue(ledPinHandle, LED3, 0);
    PIN_setOutputValue(ledPinHandle, LED4, 0);
    PIN_setOutputValue(ledPinHandle, LED5, 0);
    PIN_setOutputValue(ledPinHandle, LED6, 0);
    PIN_setOutputValue(ledPinHandle, LED7, 0);
    PIN_setOutputValue(ledPinHandle, LED8, 0);
    PIN_setOutputValue(ledPinHandle, OUT1, 0);
    PIN_setOutputValue(ledPinHandle, OUT2, 0);
    PIN_setOutputValue(ledPinHandle, OUT3, 0);
    PIN_setOutputValue(ledPinHandle, OUT4, 0);
    PIN_setOutputValue(ledPinHandle, BUZZER, 0);

    buttonPinHandle = PIN_open(&buttonPinState, buttonPinTable);
    if (!buttonPinHandle)
    {
        //System_abort("Error initializing button pins\n");
    }

    /* Setup callback for button pins */
    if (PIN_registerIntCb(buttonPinHandle, &ButtonCallbackFxn) != 0)
    {
        //System_abort("Error registering button callback function");
    }

    uint_t id0 = PIN_getInputValue(ID0);
    uint_t id1 = PIN_getInputValue(ID1);

#ifdef UART_ENABLE
    char uartString[] = "ID:00:00\r\n";
    sprintf(uartString, "ID:%2d:%2d\r\n", id0, id1);
    UART_write(uartHandle, uartString, sizeof(uartString));
#endif

    if (id0 == 0 && id1 == 0)
    {
        idValue = 0;
        SetLEDByOut(OUT1);
    }
    else if (id0 != 0 && id1 == 0)
    {
        idValue = 1;
        SetLEDByOut(OUT1);
        SetLEDByOut(OUT2);
    }
    else if (id0 == 0 && id1 != 0)
    {
        idValue = 2;
        SetLEDByOut(OUT1);
        SetLEDByOut(OUT2);
        SetLEDByOut(OUT3);
    }
    else if (id0 != 0 && id1 != 0)
    {
        idValue = 3;
        SetLEDByOut(OUT1);
        SetLEDByOut(OUT2);
        SetLEDByOut(OUT3);
        SetLEDByOut(OUT4);
    }
    outValue |= (idValue << 6);
    outValue |= (version << 16);
    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                               &outValue);

    uint8_t status = SUCCESS;
    uint8_t config = 0;
    status = osal_snv_read(SNV_ID_CONFIG, sizeof(uint8_t), (uint8 *)(&config));
    if (status == SUCCESS)
    {
        outValue |= (config << 8);
    }

    Watchdog_init();
    Watchdog_Params_init(&wdtParams);
    wdtParams.resetMode = Watchdog_RESET_ON;
    wdtParams.callbackFxn = wdtCallback;
    wdtHandle = Watchdog_open(CC2650_WATCHDOG0, &wdtParams);
    if (!wdtHandle) {
        //System_printf("Watchdog did not open");
    }
    uint32_t ticks = Watchdog_convertMsToTicks(wdtHandle, 5000);
    Watchdog_setReload(wdtHandle, ticks);

    // Create an RTOS queue for message from profile to be sent to app.
    appMsgQueue = Util_constructQueue(&appMsg);

    // Create one-shot clocks for internal periodic events.
    Util_constructClock(&periodicClock, SimpleBLEPeripheral_clockHandler,
                        SBP_PERIODIC_EVT_PERIOD, 0, false, SBP_PERIODIC_EVT);
    Util_constructClock(&disconnectClock, SimpleBLEPeripheral_clockHandler,
                        SBP_DISCONNECT_EVT_PERIOD, 0, false, SBP_DISCONNECT_EVT);

    Util_constructClock(&buzzerClock, SimpleBLEPeripheral_clockHandler,
                        SBP_BUZZER_EVT_PERIOD, 0, false, SBP_BUZZER_EVT);

    Util_constructClock(&ledClock, SimpleBLEPeripheral_clockHandler,
                        SBP_LED_EVT_PERIOD, 0, false, SBP_LED_EVT);

    Util_constructClock(&shortpressClock, SimpleBLEPeripheral_clockHandler,
                        SBP_SHORTPRESS_EVT_PERIOD, 0, false, SBP_SHORTPRESS_EVT);

    Util_constructClock(&longpressClock, SimpleBLEPeripheral_clockHandler,
                        SBP_LONGPRESS_EVT_PERIOD, 0, false, SBP_LONGPRESS_EVT);

    Util_constructClock(&button1Clock, SimpleBLEPeripheral_clockHandler,
                        SBP_SHORTPRESS_EVT_PERIOD, 0, false, SBP_BUTTON1_EVT);
    Util_constructClock(&button2Clock, SimpleBLEPeripheral_clockHandler,
                        SBP_SHORTPRESS_EVT_PERIOD, 0, false, SBP_BUTTON2_EVT);
    Util_constructClock(&button3Clock, SimpleBLEPeripheral_clockHandler,
                        SBP_SHORTPRESS_EVT_PERIOD, 0, false, SBP_BUTTON3_EVT);
    Util_constructClock(&button4Clock, SimpleBLEPeripheral_clockHandler,
                        SBP_SHORTPRESS_EVT_PERIOD, 0, false, SBP_BUTTON4_EVT);

    dispHandle = Display_open(Display_Type_LCD, NULL);

    // Setup the GAP
    GAP_SetParamValue(TGAP_CONN_PAUSE_PERIPHERAL, DEFAULT_CONN_PAUSE_PERIPHERAL);

    // Setup the GAP Peripheral Role Profile
    {
        // For all hardware platforms, device starts advertising upon initialization
        uint8_t initialAdvertEnable = TRUE;

        // By setting this to zero, the device will go into the waiting state after
        // being discoverable for 30.72 second, and will not being advertising again
        // until the enabler is set back to TRUE
        uint16_t advertOffTime = 0;

        uint8_t enableUpdateRequest = DEFAULT_ENABLE_UPDATE_REQUEST;
        uint16_t desiredMinInterval = DEFAULT_DESIRED_MIN_CONN_INTERVAL;
        uint16_t desiredMaxInterval = DEFAULT_DESIRED_MAX_CONN_INTERVAL;
        uint16_t desiredSlaveLatency = DEFAULT_DESIRED_SLAVE_LATENCY;
        uint16_t desiredConnTimeout = DEFAULT_DESIRED_CONN_TIMEOUT;

        // Set the GAP Role Parameters
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &initialAdvertEnable);
        GAPRole_SetParameter(GAPROLE_ADVERT_OFF_TIME, sizeof(uint16_t),
                             &advertOffTime);

        GAPRole_SetParameter(GAPROLE_SCAN_RSP_DATA, sizeof(scanRspData),
                             scanRspData);
        GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);

        GAPRole_SetParameter(GAPROLE_PARAM_UPDATE_ENABLE, sizeof(uint8_t),
                             &enableUpdateRequest);
        GAPRole_SetParameter(GAPROLE_MIN_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMinInterval);
        GAPRole_SetParameter(GAPROLE_MAX_CONN_INTERVAL, sizeof(uint16_t),
                             &desiredMaxInterval);
        GAPRole_SetParameter(GAPROLE_SLAVE_LATENCY, sizeof(uint16_t),
                             &desiredSlaveLatency);
        GAPRole_SetParameter(GAPROLE_TIMEOUT_MULTIPLIER, sizeof(uint16_t),
                             &desiredConnTimeout);
    }

    // Set the GAP Characteristics
    GGS_SetParameter(GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, attDeviceName);

    // Set advertising interval
    {
        uint16_t advInt = DEFAULT_ADVERTISING_INTERVAL;

        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_LIM_DISC_ADV_INT_MAX, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MIN, advInt);
        GAP_SetParamValue(TGAP_GEN_DISC_ADV_INT_MAX, advInt);
    }

    // Setup the GAP Bond Manager
    {
        uint32_t passkey = 0; // passkey "000000"
        uint8_t pairMode = GAPBOND_PAIRING_MODE_WAIT_FOR_REQ;
        uint8_t mitm = TRUE;
        uint8_t ioCap = GAPBOND_IO_CAP_DISPLAY_ONLY;
        uint8_t bonding = TRUE;

        GAPBondMgr_SetParameter(GAPBOND_DEFAULT_PASSCODE, sizeof(uint32_t),
                                &passkey);
        GAPBondMgr_SetParameter(GAPBOND_PAIRING_MODE, sizeof(uint8_t), &pairMode);
        GAPBondMgr_SetParameter(GAPBOND_MITM_PROTECTION, sizeof(uint8_t), &mitm);
        GAPBondMgr_SetParameter(GAPBOND_IO_CAPABILITIES, sizeof(uint8_t), &ioCap);
        GAPBondMgr_SetParameter(GAPBOND_BONDING_ENABLED, sizeof(uint8_t), &bonding);

        uint8 status = SUCCESS;
        status = osal_snv_read(SNV_ID_RESET, sizeof(uint8_t), (uint8 *)(&resetValue));
        if (status == SUCCESS)
        {
            if (resetValue == 1)
            {
                GAPBondMgr_SetParameter(GAPBOND_ERASE_ALLBONDS, 0, NULL);
            }
            resetValue = 0;
            osal_snv_write(SNV_ID_RESET, sizeof(uint8_t), (uint8 *)(&resetValue));
        }
    }

    // Initialize GATT attributes
    GGS_AddService(GATT_ALL_SERVICES);         // GAP
    GATTServApp_AddService(GATT_ALL_SERVICES); // GATT attributes
    DevInfo_AddService();                      // Device Information Service

#ifdef FEATURE_OAD
    VOID OAD_addService(); // OAD Profile
    OAD_register((oadTargetCBs_t *)&simpleBLEPeripheral_oadCBs);
    hOadQ = Util_constructQueue(&oadQ);
#endif //FEATURE_OAD

#ifdef IMAGE_INVALIDATE
    Reset_addService();
#endif //IMAGE_INVALIDATE

    SimpleProfile_AddService(GATT_ALL_SERVICES); // Simple GATT Profile

    // Setup the SimpleProfile Characteristic Values
    {
        uint8_t charValue1 = 1;
        uint32_t charValue2 = outValue;
        //    uint8_t charValue3 = 3;
        uint8_t charValue4 = outValue & 0x0F;
        uint8_t charValue5[SIMPLEPROFILE_CHAR5_LEN] = {0, 0, 0, 0};

        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR1, sizeof(uint8_t),
                                   &charValue1);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                                   &charValue2);
        //    SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR3, sizeof(uint8_t),
        //                               &charValue3);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR4, sizeof(uint8_t),
                                   &charValue4);
        SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR5, SIMPLEPROFILE_CHAR5_LEN,
                                   charValue5);
    }

    // Register callback with SimpleGATTprofile
    SimpleProfile_RegisterAppCBs(&SimpleBLEPeripheral_simpleProfileCBs);

    // Start the Device
    VOID GAPRole_StartDevice(&SimpleBLEPeripheral_gapRoleCBs);

    // Start Bond Manager
    VOID GAPBondMgr_Register(&simpleBLEPeripheral_BondMgrCBs);

    // Register with GAP for HCI/Host messages
    GAP_RegisterForMsgs(selfEntity);

    // Register for GATT local events and ATT Responses pending for transmission
    GATT_RegisterForMsgs(selfEntity);

    HCI_LE_ReadMaxDataLenCmd();

    HCI_EXT_AdvEventNoticeCmd(selfEntity, SBP_ADV_CB_EVT);
    
    if ((outValue & 0x0800) == 0x0000)
    {
        HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
    }
    else
    {
        HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
    }

    Display_print0(dispHandle, 0, 0, "BLE Peripheral");

    Util_startClock(&periodicClock);
    osal_snv_write(SNV_ID_APP, SNV_BUF_LEN, (uint8 *)snv_buf);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_taskFxn
 *
 * @brief   Application task entry point for the Simple BLE Peripheral.
 *
 * @param   a0, a1 - not used.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_taskFxn(UArg a0, UArg a1)
{
    // Initialize application
    SimpleBLEPeripheral_init();

    // Application main loop
    for (;;)
    {
        // Waits for a signal to the semaphore associated with the calling thread.
        // Note that the semaphore associated with a thread is signaled when a
        // message is queued to the message receive queue of the thread or when
        // ICall_signal() function is called onto the semaphore.
        ICall_Errno errno = ICall_wait(ICALL_TIMEOUT_FOREVER);

        if (errno == ICALL_ERRNO_SUCCESS)
        {
            ICall_EntityID dest;
            ICall_ServiceEnum src;
            ICall_HciExtEvt *pMsg = NULL;

            if (ICall_fetchServiceMsg(&src, &dest,
                                      (void **)&pMsg) == ICALL_ERRNO_SUCCESS)
            {
                uint8 safeToDealloc = TRUE;

                if ((src == ICALL_SERVICE_CLASS_BLE) && (dest == selfEntity))
                {
                    ICall_Stack_Event *pEvt = (ICall_Stack_Event *)pMsg;

                    // Check for BLE stack events first
                    if (pEvt->signature == 0xffff)
                    {
                        if (pEvt->event_flag & SBP_CONN_EVT_END_EVT)
                        {
                            // Try to retransmit pending ATT Response (if any)
                            SimpleBLEPeripheral_sendAttRsp();
                        }
                        else if (pEvt->event_flag & SBP_ADV_CB_EVT)
                        {
                            // Advertisment end
#ifdef UART_ENABLE
                            const char uartString[] = "Advertisment END\r\n";
                            UART_write(uartHandle, uartString, sizeof(uartString));
#endif
                        }
                    }
                    else
                    {
                        // Process inter-task message
                        safeToDealloc = SimpleBLEPeripheral_processStackMsg((ICall_Hdr *)pMsg);
                    }
                }

                if (pMsg && safeToDealloc)
                {
                    ICall_freeMsg(pMsg);
                }
            }

            // If RTOS queue is not empty, process app message.
            while (!Queue_empty(appMsgQueue))
            {
                sbpEvt_t *pMsg = (sbpEvt_t *)Util_dequeueMsg(appMsgQueue);
                if (pMsg)
                {
                    // Process message.
                    SimpleBLEPeripheral_processAppMsg(pMsg);

                    // Free the space from the message.
                    ICall_free(pMsg);
                }
            }
        }

        if (events & SBP_PERIODIC_EVT)
        {
            events &= ~SBP_PERIODIC_EVT;

            // Perform periodic application task
            //SimpleBLEPeripheral_performPeriodicTask();
            Watchdog_clear(wdtHandle);
            Util_startClock(&periodicClock);
        }

        if (events & SBP_DISCONNECT_EVT)
        {
            events &= ~SBP_DISCONNECT_EVT;
            GAPRole_TerminateConnection();
        }

        if (events & SBP_BUZZER_EVT)
        {
            events &= ~SBP_BUZZER_EVT;

            PIN_setOutputValue(ledPinHandle, BUZZER, 0);
        }

        if (events & SBP_LED_EVT)
        {
            events &= ~SBP_LED_EVT;

            if (ledCount < 7)
            {
                if (idValue == 0 || idValue == 2 || idValue == 3)
                {
                    uint32_t currVal = PIN_getOutputValue(LED4); /*LED4*/
                    PIN_setOutputValue(ledPinHandle, LED4, !currVal);
                }
                else if (idValue == 1)
                {
                    uint32_t currVal = PIN_getOutputValue(LED2); /*LED2*/
                    PIN_setOutputValue(ledPinHandle, LED2, !currVal);
                }
                ledCount++;
                Util_startClock(&ledClock);
            }
            else
            {
                snv_buf[0] = 0x00;
                snv_buf[1] = 0x0D;
                snv_buf[2] = 0x90;
                snv_buf[3] = 0x38;
                osal_snv_write(SNV_ID_APP, SNV_BUF_LEN, (uint8 *)snv_buf);

                resetValue = 1;
                osal_snv_write(SNV_ID_RESET, sizeof(uint8_t), (uint8 *)(&resetValue));

                // Reset to the bootloader.
                HAL_SYSTEM_RESET();
            }
        }

        if (events & SBP_SHORTPRESS_EVT)
        {
            events &= ~SBP_SHORTPRESS_EVT;

            ButtonCallback(buttonCBPinId);
        }

        if (events & SBP_LONGPRESS_EVT)
        {
            events &= ~SBP_LONGPRESS_EVT;

            if (idValue == 0 || idValue == 2 || idValue == 3)
            {
                PIN_setOutputValue(ledPinHandle, LED4, 1);
            }
            else if (idValue == 1)
            {
                PIN_setOutputValue(ledPinHandle, LED2, 1);
            }
            ledCount = 0;
            Util_startClock(&ledClock);
        }

        if (events & SBP_BUTTON1_EVT)
        {
            events &= ~SBP_BUTTON1_EVT;
            ButtonCallback(BUTTON1);
        }
        if (events & SBP_BUTTON2_EVT)
        {
            events &= ~SBP_BUTTON2_EVT;
            ButtonCallback(BUTTON2);
        }
        if (events & SBP_BUTTON3_EVT)
        {
            events &= ~SBP_BUTTON3_EVT;
            ButtonCallback(BUTTON3);
        }
        if (events & SBP_BUTTON4_EVT)
        {
            events &= ~SBP_BUTTON4_EVT;
            ButtonCallback(BUTTON4);
        }

#ifdef FEATURE_OAD
        while (!Queue_empty(hOadQ))
        {
            oadTargetWrite_t *oadWriteEvt = Queue_get(hOadQ);

            // Identify new image.
            if (oadWriteEvt->event == OAD_WRITE_IDENTIFY_REQ)
            {
                OAD_imgIdentifyWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
            }
            // Write a next block request.
            else if (oadWriteEvt->event == OAD_WRITE_BLOCK_REQ)
            {
                OAD_imgBlockWrite(oadWriteEvt->connHandle, oadWriteEvt->pData);
            }

            // Free buffer.
            ICall_free(oadWriteEvt);
        }
#endif //FEATURE_OAD
    }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStackMsg
 *
 * @brief   Process an incoming stack message.
 *
 * @param   pMsg - message to process
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processStackMsg(ICall_Hdr *pMsg)
{
    uint8_t safeToDealloc = TRUE;

    switch (pMsg->event)
    {
    case GATT_MSG_EVENT:
        // Process GATT message
        safeToDealloc = SimpleBLEPeripheral_processGATTMsg((gattMsgEvent_t *)pMsg);
        break;

    case HCI_GAP_EVENT_EVENT:
    {
        // Process HCI message
        switch (pMsg->status)
        {
        case HCI_COMMAND_COMPLETE_EVENT_CODE:
            // Process HCI Command Complete Event
            break;

        default:
            break;
        }
    }
    break;

    default:
        // do nothing
        break;
    }

    return (safeToDealloc);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processGATTMsg
 *
 * @brief   Process GATT messages and events.
 *
 * @return  TRUE if safe to deallocate incoming message, FALSE otherwise.
 */
static uint8_t SimpleBLEPeripheral_processGATTMsg(gattMsgEvent_t *pMsg)
{
    // See if GATT server was unable to transmit an ATT response
    if (pMsg->hdr.status == blePending)
    {
        // No HCI buffer was available. Let's try to retransmit the response
        // on the next connection event.
        if (HCI_EXT_ConnEventNoticeCmd(pMsg->connHandle, selfEntity,
                                       SBP_CONN_EVT_END_EVT) == SUCCESS)
        {
            // First free any pending response
            SimpleBLEPeripheral_freeAttRsp(FAILURE);

            // Hold on to the response message for retransmission
            pAttRsp = pMsg;

            // Don't free the response message yet
            return (FALSE);
        }
    }
    else if (pMsg->method == ATT_FLOW_CTRL_VIOLATED_EVENT)
    {
        // ATT request-response or indication-confirmation flow control is
        // violated. All subsequent ATT requests or indications will be dropped.
        // The app is informed in case it wants to drop the connection.

        // Display the opcode of the message that caused the violation.
        Display_print1(dispHandle, 5, 0, "FC Violated: %d", pMsg->msg.flowCtrlEvt.opcode);
    }
    else if (pMsg->method == ATT_MTU_UPDATED_EVENT)
    {
        // MTU size updated
        Display_print1(dispHandle, 5, 0, "MTU Size: $d", pMsg->msg.mtuEvt.MTU);
    }

    // Free message payload. Needed only for ATT Protocol messages
    GATT_bm_free(&pMsg->msg, pMsg->method);

    // It's safe to free the incoming message
    return (TRUE);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_sendAttRsp
 *
 * @brief   Send a pending ATT response message.
 *
 * @param   none
 *
 * @return  none
 */
static void SimpleBLEPeripheral_sendAttRsp(void)
{
    // See if there's a pending ATT Response to be transmitted
    if (pAttRsp != NULL)
    {
        uint8_t status;

        // Increment retransmission count
        rspTxRetry++;

        // Try to retransmit ATT response till either we're successful or
        // the ATT Client times out (after 30s) and drops the connection.
        status = GATT_SendRsp(pAttRsp->connHandle, pAttRsp->method, &(pAttRsp->msg));
        if ((status != blePending) && (status != MSG_BUFFER_NOT_AVAIL))
        {
            // Disable connection event end notice
            HCI_EXT_ConnEventNoticeCmd(pAttRsp->connHandle, selfEntity, 0);

            // We're done with the response message
            SimpleBLEPeripheral_freeAttRsp(status);
        }
        else
        {
            // Continue retrying
            Display_print1(dispHandle, 5, 0, "Rsp send retry: %d", rspTxRetry);
        }
    }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_freeAttRsp
 *
 * @brief   Free ATT response message.
 *
 * @param   status - response transmit status
 *
 * @return  none
 */
static void SimpleBLEPeripheral_freeAttRsp(uint8_t status)
{
    // See if there's a pending ATT response message
    if (pAttRsp != NULL)
    {
        // See if the response was sent out successfully
        if (status == SUCCESS)
        {
            Display_print1(dispHandle, 5, 0, "Rsp sent retry: %d", rspTxRetry);
        }
        else
        {
            // Free response payload
            GATT_bm_free(&pAttRsp->msg, pAttRsp->method);

            Display_print1(dispHandle, 5, 0, "Rsp retry failed: %d", rspTxRetry);
        }

        // Free response message
        ICall_freeMsg(pAttRsp);

        // Reset our globals
        pAttRsp = NULL;
        rspTxRetry = 0;
    }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processAppMsg
 *
 * @brief   Process an incoming callback from a profile.
 *
 * @param   pMsg - message to process
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processAppMsg(sbpEvt_t *pMsg)
{
    switch (pMsg->hdr.event)
    {
    case SBP_STATE_CHANGE_EVT:
        SimpleBLEPeripheral_processStateChangeEvt((gaprole_States_t)pMsg->hdr.state);
        break;

    case SBP_CHAR_CHANGE_EVT:
        SimpleBLEPeripheral_processCharValueChangeEvt(pMsg->hdr.state);
        break;

    // Pairing event.
    case SBP_PAIRING_EVT:
        Sbp_processPairStateEvt(pMsg->hdr.state, *pMsg->pData);

        ICall_free(pMsg->pData);
        break;

    // Passcode event.
    case SBP_PASSCODE_EVT:
        Sbp_processPasscodeEvt(*(uint16_t *)pMsg->pData);

        ICall_free(pMsg->pData);
        break;

    default:
        // Do nothing.
        break;
    }
}

/*********************************************************************
 * @fn      Sbp_pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void Sbp_pairStateCB(uint16_t connHandle, uint8_t state,
                            uint8_t status)
{
    uint8_t *pData;

    if ((pData = ICall_malloc(sizeof(uint8_t))))
    {
        *pData = status;

        SimpleBLEPeripheral_enqueueMsg(SBP_PAIRING_EVT, state, pData);
    }
}

/*********************************************************************
 * @fn      Sbp_processPairStateEvt
 *
 * @brief   Pairing state callback event processor.
 *
 * @return  none
 */
static void Sbp_processPairStateEvt(uint8_t state, uint8_t status)
{
#ifdef UART_ENABLE
    char uartString[] = "Pair:00:00\r\n";
    sprintf(uartString, "Pair:%2d:%2d\r\n", state, status);
    UART_write(uartHandle, uartString, sizeof(uartString));
#endif
    if (state == GAPBOND_PAIRING_STATE_COMPLETE || state == GAPBOND_PAIRING_STATE_BONDED)
    {
        if (status == SUCCESS)
        {
            // Store the address of the bonded address.
            Util_stopClock(&disconnectClock);
        }
        else
        {
            // error
            GAPRole_TerminateConnection();
        }
    }
}

/*********************************************************************
 * @fn      Sbp_passcodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void Sbp_passcodeCB(uint8_t *deviceAddr, uint16_t connHandle,
                           uint8_t uiInputs, uint8_t uiOutputs)
{
    uint16_t *pData;

    if ((pData = ICall_malloc(sizeof(uint16_t))))
    {
        *pData = connHandle;

        SimpleBLEPeripheral_enqueueMsg(SBP_PASSCODE_EVT, 0, (uint8_t *)pData);
    }
}

/*********************************************************************
 * @fn      Sbp_processPasscodeEvt.
 *
 * @brief   Passcode callback event processor.
 *
 * @return  none
 */
static void Sbp_processPasscodeEvt(uint16_t connHandle)
{
    uint8 status = SUCCESS;
    //Read from SNV flash
    status = osal_snv_read(SNV_ID_APP, SNV_BUF_LEN, (uint8 *)snv_buf);
    if (status != SUCCESS)
    {
        //Write first time to initialize SNV ID
        osal_snv_write(SNV_ID_APP, SNV_BUF_LEN, (uint8 *)snv_buf);
    }

    uint32_t passkey = 0;
    passkey += (((uint32_t)snv_buf[0]) << 24);
    passkey += (((uint32_t)snv_buf[1]) << 16);
    passkey += (((uint32_t)snv_buf[2]) << 8);
    passkey += (((uint32_t)snv_buf[3]) << 0);

#ifdef UART_ENABLE
    const char uartString[] = "passkey\r\n";
    UART_write(uartHandle, uartString, sizeof(uartString));
#endif
    // Send passcode response.
    GAPBondMgr_PasscodeRsp(connHandle, SUCCESS, passkey);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_stateChangeCB
 *
 * @brief   Callback from GAP Role indicating a role state change.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_stateChangeCB(gaprole_States_t newState)
{
    SimpleBLEPeripheral_enqueueMsg(SBP_STATE_CHANGE_EVT, newState, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processStateChangeEvt
 *
 * @brief   Process a pending GAP Role state change event.
 *
 * @param   newState - new state
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processStateChangeEvt(gaprole_States_t newState)
{
    if (newState == GAPROLE_STARTED)
    {
        uint8_t ownAddress[B_ADDR_LEN];
        uint8_t systemId[DEVINFO_SYSTEM_ID_LEN];

        GAPRole_GetParameter(GAPROLE_BD_ADDR, ownAddress);

        // use 6 bytes of device address for 8 bytes of system ID value
        systemId[0] = ownAddress[0];
        systemId[1] = ownAddress[1];
        systemId[2] = ownAddress[2];

        // set middle bytes to zero
        systemId[4] = 0x00;
        systemId[3] = 0x00;

        // shift three bytes up
        systemId[7] = ownAddress[5];
        systemId[6] = ownAddress[4];
        systemId[5] = ownAddress[3];

        DevInfo_SetParameter(DEVINFO_SYSTEM_ID, DEVINFO_SYSTEM_ID_LEN, systemId);

        // Display device address
        Display_print0(dispHandle, 1, 0, Util_convertBdAddr2Str(ownAddress));
        Display_print0(dispHandle, 2, 0, "Initialized");
    }
    if (newState == GAPROLE_ADVERTISING)
    {
#ifdef UART_ENABLE
        const char uartString[] = "Advertising\r\n";
        UART_write(uartHandle, uartString, sizeof(uartString));
#endif
        Display_print0(dispHandle, 2, 0, "Advertising");
        //Util_startClock(&periodicClock);
    }
    if (gapProfileState == GAPROLE_ADVERTISING &&
        newState == GAPROLE_WAITING)
    {
#ifdef UART_ENABLE
        const char uartString[] = "Waiting\r\n";
        UART_write(uartHandle, uartString, sizeof(uartString));
#endif
    }
    if (newState == GAPROLE_CONNECTED)
    {
        //Util_stopClock(&periodicClock);
        //PIN_setOutputValue(ledPinHandle, LED7, 1);

        Util_startClock(&disconnectClock);

        linkDBInfo_t linkInfo;
        uint8_t numActive = 0;
        numActive = linkDB_NumActive();

#ifdef UART_ENABLE
        char uartString[] = "Conns:0\r\n";
        sprintf(uartString, "Conns:%d\r\n", numActive);
        UART_write(uartHandle, uartString, sizeof(uartString));
#endif
        // Use numActive to determine the connection handle of the last
        // connection
        if (linkDB_GetInfo(numActive - 1, &linkInfo) == SUCCESS)
        {
            Display_print1(dispHandle, 2, 0, "Num Conns: %d", (uint16_t)numActive);
            Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(linkInfo.addr));
        }
        else
        {
            uint8_t peerAddress[B_ADDR_LEN];

            GAPRole_GetParameter(GAPROLE_CONN_BD_ADDR, peerAddress);

            Display_print0(dispHandle, 2, 0, "Connected");
            Display_print0(dispHandle, 3, 0, Util_convertBdAddr2Str(peerAddress));
        }
    }
    if (gapProfileState == GAPROLE_CONNECTED &&
        newState != GAPROLE_CONNECTED)
    {
        // disconnect
        // Util_stopClock(&periodicClock);

        SimpleBLEPeripheral_freeAttRsp(bleNotConnected);

#if defined(FEATURE_OAD)
        OADTarget_close();
#endif
#ifdef UART_ENABLE
        const char uartString[] = "Disconnected\r\n";
        UART_write(uartHandle, uartString, sizeof(uartString));
        if (newState == GAPROLE_WAITING_AFTER_TIMEOUT)
        {
            const char uartString[] = "Timeout\r\n";
            UART_write(uartHandle, uartString, sizeof(uartString));
        }
#endif
        Display_print0(dispHandle, 2, 0, "Disconnected");
        // Clear remaining lines
        Display_clearLines(dispHandle, 3, 5);

        uint8_t initialAdvertEnable = TRUE;
        GAPRole_SetParameter(GAPROLE_ADVERT_ENABLED, sizeof(uint8_t),
                             &initialAdvertEnable);
    }
    if (newState == GAPROLE_ERROR)
    {
#ifdef UART_ENABLE
        const char uartString[] = "Error\r\n";
        UART_write(uartHandle, uartString, sizeof(uartString));
#endif
        Display_print0(dispHandle, 2, 0, "Error");
    }

    // Update the state
    gapProfileState = newState;
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_charValueChangeCB
 *
 * @brief   Callback from Simple Profile indicating a characteristic
 *          value change.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_charValueChangeCB(uint8_t paramID)
{
    SimpleBLEPeripheral_enqueueMsg(SBP_CHAR_CHANGE_EVT, paramID, NULL);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_processCharValueChangeEvt
 *
 * @brief   Process a pending Simple Profile characteristic value change
 *          event.
 *
 * @param   paramID - parameter ID of the value that was changed.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_processCharValueChangeEvt(uint8_t paramID)
{
    uint8_t newValue;

    switch (paramID)
    {
    case SIMPLEPROFILE_CHAR1:
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR1, &newValue);

        Display_print1(dispHandle, 4, 0, "Char 1: %d", (uint16_t)newValue);

        if (idValue == 0)
        {
            if (newValue == 1)
            {
                ButtonPress(OUT1);
            }
        }
        else if (idValue == 1)
        {
            if (newValue == 1)
            {
                ButtonPress(OUT1);
            }
            else if (newValue == 2)
            {
                ButtonPress(OUT2);
            }
        }
        else if (idValue == 2)
        {
            if (newValue == 1)
            {
                ButtonPress(OUT1);
            }
            else if (newValue == 2)
            {
                ButtonPress(OUT2);
            }
            else if (newValue == 3)
            {
                ButtonPress(OUT3);
            }
        }
        else if (idValue == 3)
        {
            if (newValue == 1)
            {
                ButtonPress(OUT1);
            }
            else if (newValue == 2)
            {
                ButtonPress(OUT2);
            }
            else if (newValue == 3)
            {
                ButtonPress(OUT3);
            }
            else if (newValue == 4)
            {
                ButtonPress(OUT4);
            }
        }

        if (newValue == 5)
        {
            outValue ^= (0x01 << 10);
            SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                                       &outValue);
            osal_snv_write(SNV_ID_CONFIG, sizeof(uint8_t), (uint8 *)(&outValue) + 1);
        }
        else if (newValue == 6)
        {
            outValue ^= (0x01 << 8);
            SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                                       &outValue);
            osal_snv_write(SNV_ID_CONFIG, sizeof(uint8_t), (uint8 *)(&outValue) + 1);

            SetLEDByOut(OUT1);
            SetLEDByOut(OUT2);
            SetLEDByOut(OUT3);
            SetLEDByOut(OUT4);
        }
        else if (newValue == 7)
        {
            outValue ^= (0x01 << 9);
            SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                                       &outValue);
            osal_snv_write(SNV_ID_CONFIG, sizeof(uint8_t), (uint8 *)(&outValue) + 1);

            SetLEDByOut(OUT1);
            SetLEDByOut(OUT2);
            SetLEDByOut(OUT3);
            SetLEDByOut(OUT4);
        }
        else if (newValue == 8)
        {
            outValue ^= (0x01 << 11);

            if ((outValue & 0x0800) == 0x0000)
            {
                HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_0_DBM);
            }
            else
            {
                HCI_EXT_SetTxPowerCmd(HCI_EXT_TX_POWER_5_DBM);
            }

            SimpleProfile_SetParameter(SIMPLEPROFILE_CHAR2, sizeof(uint32_t),
                                       &outValue);
            osal_snv_write(SNV_ID_CONFIG, sizeof(uint8_t), (uint8 *)(&outValue) + 1);
        }

        break;

    case SIMPLEPROFILE_CHAR5:
        SimpleProfile_GetParameter(SIMPLEPROFILE_CHAR5, &snv_buf);

        osal_snv_write(SNV_ID_APP, SNV_BUF_LEN, (uint8 *)snv_buf);
        break;

    default:
        // should not reach here!
        break;
    }
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_performPeriodicTask
 *
 * @brief   Perform a periodic application task. This function gets called
 *          every five seconds (SBP_PERIODIC_EVT_PERIOD). In this example,
 *          the value of the third characteristic in the SimpleGATTProfile
 *          service is retrieved from the profile, and then copied into the
 *          value of the the fourth characteristic.
 *
 * @param   None.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_performPeriodicTask(void)
{
#ifdef UART_ENABLE
    //const char uartString[] = "HelloWorld\r\n";
    //UART_write(uartHandle, uartString, sizeof(uartString));
#endif
    //uint32_t currVal = 0;
    //currVal = PIN_getOutputValue(LED7);
    //PIN_setOutputValue(ledPinHandle, LED7, !currVal);
}

#ifdef FEATURE_OAD
/*********************************************************************
 * @fn      SimpleBLEPeripheral_processOadWriteCB
 *
 * @brief   Process a write request to the OAD profile.
 *
 * @param   event      - event type:
 *                       OAD_WRITE_IDENTIFY_REQ
 *                       OAD_WRITE_BLOCK_REQ
 * @param   connHandle - the connection Handle this request is from.
 * @param   pData      - pointer to data for processing and/or storing.
 *
 * @return  None.
 */
void SimpleBLEPeripheral_processOadWriteCB(uint8_t event, uint16_t connHandle,
                                           uint8_t *pData)
{
    oadTargetWrite_t *oadWriteEvt = ICall_malloc(sizeof(oadTargetWrite_t) +
                                                 sizeof(uint8_t) * OAD_PACKET_SIZE);

    if (oadWriteEvt != NULL)
    {
        oadWriteEvt->event = event;
        oadWriteEvt->connHandle = connHandle;

        oadWriteEvt->pData = (uint8_t *)(&oadWriteEvt->pData + 1);
        memcpy(oadWriteEvt->pData, pData, OAD_PACKET_SIZE);

        Queue_put(hOadQ, (Queue_Elem *)oadWriteEvt);

        // Post the application's semaphore.
        Semaphore_post(sem);
    }
    else
    {
        // Fail silently.
    }
}
#endif //FEATURE_OAD

/*********************************************************************
 * @fn      SimpleBLEPeripheral_clockHandler
 *
 * @brief   Handler function for clock timeouts.
 *
 * @param   arg - event type
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_clockHandler(UArg arg)
{
    // Store the event.
    events |= arg;

    // Wake up the application.
    Semaphore_post(sem);
}

/*********************************************************************
 * @fn      SimpleBLEPeripheral_enqueueMsg
 *
 * @brief   Creates a message and puts the message in RTOS queue.
 *
 * @param   event - message event.
 * @param   state - message state.
 *
 * @return  None.
 */
static void SimpleBLEPeripheral_enqueueMsg(uint16_t event, uint8_t state, uint8_t *pData)
{
    sbpEvt_t *pMsg;

    // Create dynamic pointer to message.
    if ((pMsg = ICall_malloc(sizeof(sbpEvt_t))))
    {
        pMsg->hdr.event = event;
        pMsg->hdr.state = state;
        pMsg->pData = pData;

        // Enqueue the message.
        Util_enqueueMsg(appMsgQueue, sem, (uint8 *)pMsg);
    }
}
