/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"

#include "npi.h"

/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// 系统定时器间隔时间
#define SBP_PERIODIC_EVT_PERIOD               1600 //ms 连接从机的间隔，不能太短， 建议时间



// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  8 // 最大扫描从机个数，最大为8， 2540 为3 

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 3000 // 开始扫描到返回扫描结果的时间间隔， 不能太小的原因是与从机的广播间隔有关

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL // 扫描所有从机， 当然你可以指定扫描

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000 // 这个是使能读取rssi是每次返回结果之间的时间间隔

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
// 连接间隔与数据发送量有关， 连接间隔越短， 单位时间内就能发送越多的数据
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      8//400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      8//800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           100 // 这个是参数更新的定时时间， 不能太长， 否则影响数据发送----请多做实验再修改该值

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           100 // 这个的意思是连接上从机后延时多长时间去获取从机的服务， 为了加快速度，这里我们设置为 100ms


// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE


static void oneConnectedDev_CharX_Option(uint8 index, uint8 charIdx, uint8 value[20], uint8 len);
// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR1,               // Characteristic discovery
  BLE_DISC_STATE_CHAR2,     
  BLE_DISC_STATE_CHAR3,     
  BLE_DISC_STATE_CHAR4,     
  BLE_DISC_STATE_CHAR5, 
  BLE_DISC_STATE_CHAR6, 
  BLE_DISC_STATE_CHAR7, 
};


// 这个枚举定义了选择哪个数据通道，即特征值
enum
{
    BLE_CHAR1 = 0,
    BLE_CHAR2,
    BLE_CHAR3,
    BLE_CHAR4,
    BLE_CHAR5,
    BLE_CHAR6,
    BLE_CHAR7,
};
/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes; // 扫描结果， 表现为个数 ，




// 表示当前处理的是哪一个从机 最先出现在 simpleBLECentralEventCB函数中的
// GAP_DEVICE_DISCOVERY_EVENT 事件，该事件是扫描完成的回掉函数
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];// 扫描结果列表, 内含扫描到的从机地址

// Scanning state
static uint8 simpleBLEScanning = FALSE;



static uint8 connectedPeripheralNum = 0;

#define  MAX_PERIPHERAL_NUM   DEFAULT_MAX_SCAN_RES
#define  MAX_CHAR_NUM         7



//把上面屏蔽的这些参数放到一个结构体中去， 方便我们做1主3从的连接控制， 相当于每一个连接有一组自己的参数
typedef struct ble_dev{
    
    uint16 simpleBLEConnHandle;             // 连接上后的handle值， 用于对该设备的操作， 这是个源头， 根据这个handle 可以得到很多你需要的参数
    uint8 simpleBLEDevIdx;                  // 设备下标 0xff 我无效值
    
    uint8 simpleBLEState;                   // 连接状态， 比如空闲、连接中、已连接、连接断开等
 
    uint8 simpleBLEDiscState;               // 发现状态，
    
    uint16 simpleBLESvcStartHdl;            // 服务的开始handle
    uint16 simpleBLESvcEndHdl;              // 服务的结束handle
    
    uint16 simpleBLECharHdl[MAX_CHAR_NUM];  // 特征值handle， 用于读写数据
    
    uint8 simpleBLECharVal;                 // 用于写数据
    
    uint8 simpleBLERssi;                    // 是否查询 rssi 值， 这个查询必须是在连接从机之后才可以查询
    
    bool simpleBLEDoWrite;                  // 是否在可写数据， 否则就可以读数据
    
    // GATT read/write procedure tate
    bool simpleBLEProcedureInProgress;      // [MAX_PERIPHERAL_NUM] = FALSE; // 是否正在操作中
    
    int8 rssi;                              // rssi 信号值
    
    bool updateFlag;                        // 参数更新 
}ble_dev_t;




// 用于 对上面的 BLE_DEV 这个结构体的初始化 
// 该结构体在任务初始化中，设备连接失败，设备断开连接中赋值
static const ble_dev_t __g_dev_init_value =
{
    GAP_CONNHANDLE_INIT,    // simpleBLEConnHandle;
    0xff,                   // 设备下标 0xff 我无效值 simpleBLEDevIdx

    BLE_STATE_IDLE,         // simpleBLEState
 
    BLE_DISC_STATE_IDLE,    // simpleBLEDiscState 

    0,                      // uint16 simpleBLESvcStartHdl = 0;
    0,                      // uint16 simpleBLESvcEndHdl = 0;

    {0, 0, 0, 0, 0, 0, 0},  // uint16 simpleBLECharHdl

    0,                      // uint8 simpleBLECharVal = 0;

    FALSE,                  // 是否查询 rssi 值， 这个查询必须是在连接从机之后才可以查询 simpleBLERssi

    TRUE,                   // bool simpleBLEDoWrite = FALSE;

    FALSE,                  // bool simpleBLEProcedureInProgress
    
    0,                      // rssi 信号值

    FALSE,                  // 参数更新 updateFlag
};


//定义每一个从机连接数组
static ble_dev_t __g_dev[MAX_PERIPHERAL_NUM] = {0};  



/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
static ble_dev_t *simpleBLECentralGetDev(uint16 connHandle);
char *bdAddr2Str ( uint8 *pAddr );

static void NpiSerialCallback( uint8 port, uint8 events );


/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};



#define  MAX_DEVICE_NUM  3

//预存Mac地址
//十六进制序列逆序存储！与扫描存储的Mac地址对应，0x5CF821A243B8
//0X3881D70C7C0B
static uint8 devMacList[MAX_DEVICE_NUM][B_ADDR_LEN] = {
  {0x0b,0x7c,0x0c,0xd7,0x81,0x38},   // device 1
  {0xf3,0x10,0x2c,0x02,0xE2,0x90},   // usb dongle simulate
  {0x0D,0x15,0x2F,0xDE,0x03,0x34},   // 34 03 DE 2F 15 0D
};



static uint16 count;             //用于定时器计数

static int osal_continuous_scan_flag = 0;

#define LED2 P1_1       // P1.1口控制LED1


// 测试使用的计数器
static uint32 __g_count = 0;

//定时器T3中断处理函数
#pragma vector = T3_VECTOR 
__interrupt void T3_ISR(void) 
{ 
    IRCON = 0x00;               //清中断标志, 也可由硬件自动完成 
    
    
    if((count++ > 2000) && (connectedPeripheralNum == 3)) {   // 约1s
    //if((count++ > 2000)) {   // 约1s
        //NPI_PrintString("clk\n");
        NPI_PrintValue(" __g_count : ", __g_count, 10);
        count = 0;
        __g_count = 0;
    }
    
    
    
        
    if(count++ > 7000)          //245次中断后LED取反，闪烁一轮（约为245 -> 0.5 秒时间） 
    {                           //经过示波器测量确保精确
        count = 0;              //计数清零 
        
        // 如果设备不再扫秒过程中，并且设备连接数小于三 才进行扫描操作
        if ((osal_continuous_scan_flag == 1) && 
            ( !simpleBLEScanning )           && 
            (connectedPeripheralNum < 3)) {
            
            //NPI_PrintString("scan\n");
            //NPI_PrintValue(" num : ", connectedPeripheralNum, 10);
            
            simpleBLEScanRes = 0; 
            simpleBLEScanning = TRUE;   // 在扫描设备之前保证这个flag = true
            //osal_continuous_scan_flag = 0;
            // 定时器执行扫描函数
            GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE, // DEFAULT_DISCOVERY_MODE,
                                           DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                           DEFAULT_DISCOVERY_WHITE_LIST );  
        }
    } 
}


//数组比较函数，两个数组完全相等返回TRUE,否则返回FALSE
static bool isArrayEqual(uint8 arr1[],uint8 arr2[],uint8 arr1_length,uint8 arr2_length)
{
    int i = 0;
    if(arr1_length != arr2_length)return FALSE;
    for(i = 0; i < arr1_length; i++)
    {
        if(arr1[i] != arr2[i])return FALSE;
    }
    return TRUE;
}


//Mac地址匹配算法
//循环将simpleBLEDevList[i].addr的值提取，在devMacList中查找匹配的mac，若存在则记录对应的下标
 int findMacAddrMatching(int *devMacRes)
 {
     int k = 0,num = 0;
     int i = 0,j = 0;
     for(i = 0; i < simpleBLEScanRes; i++)  //对扫描结果循环
     {
         for(j = 0; j < MAX_DEVICE_NUM; j++)
         {
             if(isArrayEqual(simpleBLEDevList[i].addr, *(devMacList+j), B_ADDR_LEN, B_ADDR_LEN) == FALSE)
                 continue;//不相等匹配下一个
             
             //按照预存Mac的顺序存储设备下标，故自动连接时获取的句柄MultiConnHandle[]也是固定顺序，便于分别设定通知开关的句柄
             devMacRes[j] = i;
             k++;
             //若相等记录下标并跳出该层循环（mac唯一）
             break;
         }
     }
     //统计匹配到的mac设备个数
     for(k = 0; k < MAX_DEVICE_NUM; k++)
     {
         if(devMacRes[k] != -1)
             num++;
     }
     return num;
 }



/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;
  
  //Npi Uart Init
  NPI_InitTransport(NpiSerialCallback);
  NPI_WriteTransport("Hello World\n",12);

  
  // 初始化结构体
  for(int i = 0; i < MAX_PERIPHERAL_NUM; i++) {
      osal_memcpy(&(__g_dev[i]), &__g_dev_init_value, sizeof(ble_dev_t));
  }
  
  
  
  
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE,     sizeof( uint8 ) , &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION,  sizeof( uint8 ) , &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES,  sizeof( uint8 ) , &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED,  sizeof( uint8 ) , &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
  
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
    
    VOID task_id; // OSAL required parameter that isn't used in this function
    
    if ( events & SYS_EVENT_MSG ) // 系统事件
    {
        uint8 *pMsg;
        
        if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
        {
            simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );
            
            // Release the OSAL message
            VOID osal_msg_deallocate( pMsg );
        }
        
        // return unprocessed events
        return (events ^ SYS_EVENT_MSG);
    }
    
    if ( events & START_DEVICE_EVT ) // 初始化后就执行这个
    {
        // Start the Device
        VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );
        
        // Register with bond manager after starting device
        GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );
        
        //自动开始搜索
        //    if ( !simpleBLEScanning & simpleBLEScanRes == 0 )
        {
            simpleBLEScanning = TRUE;
            simpleBLEScanRes = 0;
            GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                          DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                          DEFAULT_DISCOVERY_WHITE_LIST );   
            LCD_WRITE_STRING( "Scanning...", HAL_LCD_LINE_1 );
        }
        
        return ( events ^ START_DEVICE_EVT );
    }
    
    if ( events & START_DISCOVERY_EVT )
    {
        simpleBLECentralStartDiscovery( );
        
        return ( events ^ START_DISCOVERY_EVT );
    }
    
    
    // 暂时保留事件接口
    if ( events & AUTO_DISC_EVT )
    {
        ble_dev_t *p = &(__g_dev[simpleBLEScanIdx]);
        uint8 ValueBuf[1] = {0};  
        NPI_PrintString("AUTO_DISC_EVT\n");
        
        
        
        // 从char1 发送handle
        ValueBuf[0] = p->simpleBLEConnHandle;
        oneConnectedDev_CharX_Option(p->simpleBLEConnHandle, BLE_CHAR1, ValueBuf, 1);
        //osal_start_timerEx(simpleBLETaskId, AUTO_DISC_EVT, 10000);
        return ( events ^ AUTO_DISC_EVT );
    }
    
    
    
    if ( events & ENABLE_CHAR4_NOTICE_EVT )
    {
        
        //NPI_PrintString("ENABLE_CHAR4_NOTICE_EVT \n");
        ble_dev_t *p = &(__g_dev[simpleBLEScanIdx]);
        attWriteReq_t AttReq;       
        uint8 ValueBuf[2];         
        
        AttReq.handle = 0x002F;
        AttReq.len = 2;
        AttReq.sig = 0;
        AttReq.cmd = 0;
        ValueBuf[0] = 0x01;
        ValueBuf[1] = 0x00;
        osal_memcpy(AttReq.value,ValueBuf,2);
        GATT_WriteCharValue( p->simpleBLEConnHandle, &AttReq, simpleBLETaskId );
        
        //osal_delay(100);
        osal_start_timerEx(simpleBLETaskId, AUTO_DISC_EVT, 2000);
        
//        // 从char1 发送handle
//        ValueBuf[0] = p->simpleBLEConnHandle;
//        oneConnectedDev_CharX_Option(p->simpleBLEConnHandle, BLE_CHAR1, ValueBuf, 1);
        
        
        // __g_beat_flag = 2;
        return ( events ^ ENABLE_CHAR4_NOTICE_EVT );
    }
    
    
    
    // Discard unknown events
    return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
    switch ( pMsg->event )
    {
    case KEY_CHANGE:
        simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
        break;
        
    case GATT_MSG_EVENT:
        simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
        break;
    }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
//uint8 gStatus;
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
    (void)shift;  // Intentionally unreferenced parameter
    
    if ( keys & HAL_KEY_SW_1 )    //S1按键
    {     
        uint8 ValueBuf[1] = {0x00};
        //ble_dev_t *p = &(__g_dev[0]);
        //NPI_PrintValue("  handle: ", p->simpleBLECharHdl[0], 16);
        
        ValueBuf[0] = __g_dev[0].simpleBLEConnHandle;
        oneConnectedDev_CharX_Option(0, BLE_CHAR1, ValueBuf, 1);
        
        ValueBuf[0] = __g_dev[1].simpleBLEConnHandle;
        oneConnectedDev_CharX_Option(1, BLE_CHAR1, ValueBuf, 1);
        
        ValueBuf[0] = __g_dev[2].simpleBLEConnHandle;
        oneConnectedDev_CharX_Option(2, BLE_CHAR1, ValueBuf, 1);
        NPI_WriteTransport("Enable Notice\n", 14 ); 
    }
    
    if ( keys & HAL_KEY_SW_2 )    //S2按键
    {
        uint8 s_buf[] = " young2zq";
        NPI_WriteTransport("KEY K2\n",7);
        
        s_buf[0] = (uint8)osal_strlen( (char*)s_buf );
        
        oneConnectedDev_CharX_Option(0, BLE_CHAR6, s_buf, SIMPLEPROFILE_CHAR6_LEN);
    }
    
}



// 寻找当前的connHandle （已连接的从机句柄）对应的设备结构地址
static ble_dev_t *simpleBLECentralGetDev(uint16 connHandle)
{
    
    for(int i = 0; i < MAX_PERIPHERAL_NUM; i++) {
        
        if (connHandle == __g_dev[i].simpleBLEConnHandle) {
            return &(__g_dev[i]);
            
        }
    }
    
    return NULL;
}



/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
    
    ble_dev_t *p = simpleBLECentralGetDev(pMsg->connHandle);
    
    
    if(p == NULL) {  // 未找到相应的connhandle(已连接的从机句柄)
        
        NPI_PrintString("ERROR simpleBLECentralProcessGATTMsg \n");
        NPI_PrintValue("  connHandle: ", pMsg->connHandle, 16);
        return;
    }
    
    
    
    if ( p->simpleBLEState != BLE_STATE_CONNECTED )
    {
        // In case a GATT message came after a connection has dropped,
        // ignore the message
        return;
    }
    
    if ( ( pMsg->method == ATT_READ_RSP ) ||
        ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
    {
        if ( pMsg->method == ATT_ERROR_RSP )
        {
            uint8 status = pMsg->msg.errorRsp.errCode;
            
            LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
        }
        else
        {
            // After a successful read, display the read value
            uint8 valueRead = pMsg->msg.readRsp.value[0];
            
            LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
        }
        
        
        
        
        p->simpleBLEProcedureInProgress = FALSE;
    }
    else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
             ( ( pMsg->method == ATT_ERROR_RSP ) &&
              ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
    {
        
        if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
        {
            uint8 status = pMsg->msg.errorRsp.errCode;
            
            LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
        }
        else
        {
            // After a succesful write, display the value that was written and increment value
            //LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
            //simpleBLEChar6DoWrite = TRUE;
        }
        
        p->simpleBLEProcedureInProgress = FALSE;    
        
    }
    else if ( p->simpleBLEDiscState != BLE_DISC_STATE_IDLE )
    {
        simpleBLEGATTDiscoveryEvent( pMsg );
    }
    else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) )   //通知
    {
        // 从机给主机发送给通知的数据接口  
        if( pMsg->msg.handleValueNoti.handle == 0x0038)   //CHAR7的通知  串口打印
        {
            if(pMsg->msg.handleValueNoti.value[0]>=20)
            {
                NPI_WriteTransport(&pMsg->msg.handleValueNoti.value[1],19 ); 
                NPI_WriteTransport("...\n",4 ); 
            }
            else
            {
                
                __g_count += pMsg->msg.handleValueNoti.value[0];
//                NPI_PrintString("\r\n");
               NPI_PrintValue("value[0] = ", pMsg->msg.handleValueNoti.value[0], 10);
                
//                NPI_PrintValue("  connhandle = ", pMsg->connHandle, 10);
//                NPI_PrintString("\r\n");
//                
                NPI_WriteTransport(&pMsg->msg.handleValueNoti.value[1],pMsg->msg.handleValueNoti.value[0] ); 
            }
        }
        
        
                // 从机给主机发送给通知的数据接口  
        if( pMsg->msg.handleValueNoti.handle == 0x002E)   //CHAR4的通知  串口打印
        {
        
            //NPI_WriteTransport(&pMsg->msg.handleValueNoti.value[1],14 );
            NPI_PrintValue("char4: ", pMsg->msg.handleValueNoti.value[0], 10); 
            //NPI_PrintValue("  connhandle = ", pMsg->connHandle, 10);
            
        }
    }
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
              
              //NPI_PrintString("GAP_DEVICE_INFO_EVENT \n");
            //向设备发现列表中添加一个设备
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT: //设备发现完成
        {      
            
            simpleBLEScanning = FALSE;
            // if not filtering device discovery results based on service UUID
            if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
            {
                // Copy results
                simpleBLEScanRes = pEvent->discCmpl.numDevs;
                osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                            (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
            }
            
            
            LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                   10, HAL_LCD_LINE_1 );
            
            
            
            uint8 k = 0;
            
            //打印所有设备Mac
            
            NPI_PrintString("device list:\n");    
            for(k = 0; k < simpleBLEScanRes; k++) {
                NPI_PrintString((uint8*)bdAddr2Str( simpleBLEDevList[k].addr ));
                NPI_PrintString("  \n");
            } 
            
            
            osal_continuous_scan_flag = 1;
            
            
            uint8 connidx = 0;  // 临时使用 作为链接的索引号，值可以设置为0，1，2
            if ( simpleBLEScanRes > 0 )
            {
                
                
                
                NPI_PrintValue("GAP_DEVICE_DISCOVERY_EVENT  simpleBLEScanIdx : 0x", simpleBLEScanIdx, 16);
                    

                LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
                
                // 判断是否是我们需要的mac
                HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );   //开LED1
                //if ( simpleBLEState == BLE_STATE_IDLE )
                //{           
                    uint8 addrType;
                    uint8 *peerAddr;
                    int devMacRes[MAX_DEVICE_NUM] = {-1, -1, -1}; // 下标保存
                    int devMacNum = 0;
                    //simpleBLEScanIdx = 0;
                    
                    //执行Mac匹配算法
                    devMacNum=findMacAddrMatching(devMacRes);
                    
                    // 没有匹配搭配到的数据
                    if (devMacNum == 0) {
                        
                        NPI_PrintString(" no found match mac\n");
                        break;
                    }
                    
                    // 到这里为止表示有匹配到数据
                    
                    for (int i = 0; i < MAX_DEVICE_NUM; i++) {
                        if(devMacRes[i] != -1) {
                            connidx = i;
                            break;
                        }
                        
                    }
                    
                    simpleBLEScanIdx = devMacRes[connidx];
                    
                    // connect to current device in scan result 
                    peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
                    addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
               
                    ble_dev_t *p = &(__g_dev[connidx]);
                    p->simpleBLEState = BLE_STATE_CONNECTING;
                    
                    GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                  DEFAULT_LINK_WHITE_LIST,
                                                  addrType, peerAddr );
                //}
                
                HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF ); 
            }
            
            
            // 修改过
            simpleBLEScanIdx = connidx;
        }
      break;

    case GAP_LINK_ESTABLISHED_EVENT: //设备连接
        {
            ble_dev_t *p = &(__g_dev[simpleBLEScanIdx]);
            p->simpleBLEDevIdx = simpleBLEScanIdx;

            if ( pEvent->gap.hdr.status == SUCCESS )
            {          
                p->simpleBLEState = BLE_STATE_CONNECTED;
                
                //将handle存起来，供主机发送数据的时候使用
                p->simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
                p->simpleBLEProcedureInProgress = TRUE;    

                NPI_PrintValue("handle : ", p->simpleBLEConnHandle, 10);
                
                connectedPeripheralNum++;
               
                for(int j = 0; j < MAX_CHAR_NUM; j++) {
                    p->simpleBLECharHdl[j] = 0;
                }
                
                // If service discovery not performed initiate service discovery
                if ( p->simpleBLECharHdl[0] == 0 )
                {
                    osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
                }
                
                LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
                LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );  
                HalLedSet(HAL_LED_1, HAL_LED_MODE_ON );   //开LED3
            }
            else
            {
                // 似乎没必要赋值
//                p->simpleBLEState = BLE_STATE_IDLE;
//                p->simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
//                p->simpleBLERssi = FALSE;
//                p->simpleBLEDiscState = BLE_DISC_STATE_IDLE;
                
                simpleBLEScanning = FALSE;
                
                
                
                osal_memcpy(p, &__g_dev_init_value, sizeof(ble_dev_t));
                
                
                LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
                LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
                HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );   //关LED3
            }
        }
      break;

    case GAP_LINK_TERMINATED_EVENT: //断开连接
        {
            //ble_dev_t *p = &(__g_dev[pEvent->linkTerminate.connectionHandle]);
            ble_dev_t *p = simpleBLECentralGetDev(pEvent->linkTerminate.connectionHandle);
            connectedPeripheralNum--;
            
//            p->simpleBLEState = BLE_STATE_IDLE;
//            p->simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
//            p->simpleBLERssi = FALSE;
//            p->simpleBLEDiscState = BLE_DISC_STATE_IDLE;
//            simpleBLECharHdl = 0;
//            p->simpleBLEProcedureInProgress = FALSE;
            
            
            // 初始化
            osal_memcpy(p, &__g_dev_init_value, sizeof(ble_dev_t));
            
            
            
            
            LCD_WRITE_STRING( "Disconnected", HAL_LCD_LINE_1 );
            LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
                                   10, HAL_LCD_LINE_2 );
            HalLedSet(HAL_LED_1, HAL_LED_MODE_OFF );   //关LED3
            
            
            NPI_PrintValue("handle : ", pEvent->linkTerminate.connectionHandle, 10);
            
            simpleBLEScanning = FALSE;
            //NPI_WriteTransport("YYY YY\n", 7);
        }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT: //参数更新
        {
            
            // 这段是使能 char4 与 char7 的通知
            ble_dev_t *p = &(__g_dev[simpleBLEScanIdx]);
            attWriteReq_t AttReq;       
            uint8 ValueBuf[2];
            LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1);           
            
            AttReq.handle = 0x0039;
            AttReq.len = 2;
            AttReq.sig = 0;
            AttReq.cmd = 0;
            ValueBuf[0] = 0x01;
            ValueBuf[1] = 0x00;
            osal_memcpy(AttReq.value,ValueBuf,2);
            GATT_WriteCharValue( p->simpleBLEConnHandle, &AttReq, simpleBLETaskId );
            
            NPI_WriteTransport("Enable Notice\n", 14 ); 
        }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
    if ( state == GAPBOND_PAIRING_STATE_STARTED )
    {
        LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
    }
    else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
    {
        if ( status == SUCCESS )
        {
            LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
        }
        else
        {
            LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
        }
    }
    else if ( state == GAPBOND_PAIRING_STATE_BONDED )
    {
        if ( status == SUCCESS )
        {
            LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
        }
    }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
    uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
    HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
    
    
    ble_dev_t *p = &(__g_dev[simpleBLEScanIdx]);
    
    // Initialize cached handles
    p->simpleBLESvcStartHdl = p->simpleBLESvcEndHdl = 0;
    p->simpleBLEDiscState = BLE_DISC_STATE_SVC;
    
    // Discovery simple BLE service
    GATT_DiscPrimaryServiceByUUID( p->simpleBLEConnHandle,
                                  uuid,
                                  ATT_BT_UUID_SIZE,
                                  simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
    attReadByTypeReq_t req;
    ble_dev_t *p = &(__g_dev[simpleBLEScanIdx]);
    
    
    if ( p->simpleBLEDiscState == BLE_DISC_STATE_SVC )
    {
        // Service found, store handles
        if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
            pMsg->msg.findByTypeValueRsp.numInfo > 0 )
        {
            p->simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
            p->simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
        }
        
        // If procedure complete
        if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
              pMsg->hdr.status == bleProcedureComplete ) ||
            ( pMsg->method == ATT_ERROR_RSP ) )
        {
            if ( p->simpleBLESvcStartHdl != 0 )
            {
                // Discover characteristic
                p->simpleBLEDiscState = BLE_DISC_STATE_CHAR1;
                
                req.startHandle = p->simpleBLESvcStartHdl;
                req.endHandle = p->simpleBLESvcEndHdl;
                req.type.len = ATT_BT_UUID_SIZE;
                req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
                req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);
                
                GATT_ReadUsingCharUUID( p->simpleBLEConnHandle, &req, simpleBLETaskId );

            }
        }
    }
    else if (p->simpleBLEDiscState == BLE_DISC_STATE_CHAR1 ) 
    {
        // Characteristic found, store handle
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
            pMsg->msg.readByTypeRsp.numPairs > 0 )
        {
            p->simpleBLECharHdl[0] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                                  pMsg->msg.readByTypeRsp.dataList[1] );
            
            
            NPI_PrintValue("  handle1:0x", p->simpleBLECharHdl[0] , 16);
            
            LCD_WRITE_STRING( "  Char1 Found", HAL_LCD_LINE_1 );
            
            
            p->simpleBLEProcedureInProgress = TRUE;
        }
        else 
        {
            p->simpleBLEDiscState = BLE_DISC_STATE_CHAR6;
            
            req.startHandle = p->simpleBLESvcStartHdl;
            req.endHandle = p->simpleBLESvcEndHdl;
            req.type.len = ATT_BT_UUID_SIZE;
            req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR6_UUID);
            req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR6_UUID);
            
            GATT_ReadUsingCharUUID( p->simpleBLEConnHandle, &req, simpleBLETaskId );
        }
    }
    
    
    else if ( p->simpleBLEDiscState == BLE_DISC_STATE_CHAR6 )
    {
        // Characteristic found, store handle
        if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
            pMsg->msg.readByTypeRsp.numPairs > 0 )
        {
            p->simpleBLECharHdl[5] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                                  pMsg->msg.readByTypeRsp.dataList[1] );
            
            
            NPI_PrintValue("  handle6:0x", p->simpleBLECharHdl[5] , 16);
            
            LCD_WRITE_STRING( "  Char6 Found", HAL_LCD_LINE_1 );
            
            
            p->simpleBLEProcedureInProgress = FALSE;
        }
        
        p->simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        
        osal_start_timerEx(simpleBLETaskId, ENABLE_CHAR4_NOTICE_EVT, 500);
    }    
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
    uint8 adLen;
    uint8 adType;
    uint8 *pEnd;
    
    pEnd = pData + dataLen - 1;
    
    // While end of data not reached
    while ( pData < pEnd )
    {
        // Get length of next AD item
        adLen = *pData++;
        if ( adLen > 0 )
        {
            adType = *pData;
            
            // If AD type is for 16-bit service UUID
            if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
            {
                pData++;
                adLen--;
                
                // For each UUID in list
                while ( adLen >= 2 && pData < pEnd )
                {
                    // Check for match
                    if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
                    {
                        // Match found
                        return TRUE;
                    }
                    
                    // Go to next
                    pData += 2;
                    adLen -= 2;
                }
                
                // Handle possible erroneous extra byte in UUID list
                if ( adLen == 1 )
                {
                    pData++;
                }
            }
            else
            {
                // Go to next item
                pData += adLen;
            }
        }
    }
    
    // Match not found
    return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
    uint8 i;
    
    // If result count not at max
    if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
    {
        // Check if device is already in scan results
        for ( i = 0; i < simpleBLEScanRes; i++ )
        {
            if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
            {
                return;
            }
        }
        
        // Add addr to scan result list
        osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
        simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
        
        // Increment scan result count
        simpleBLEScanRes++;
    }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
    uint8       i;
    char        hex[] = "0123456789ABCDEF";
    static char str[B_ADDR_STR_LEN];
    char        *pStr = str;
    
    *pStr++ = '0';
    *pStr++ = 'x';
    
    // Start from end of addr
    pAddr += B_ADDR_LEN;
    
    for ( i = B_ADDR_LEN; i > 0; i-- )
    {
        *pStr++ = hex[*--pAddr >> 4];
        *pStr++ = hex[*pAddr & 0x0F];
    }
    
    *pStr = 0;
    
    return str;
}


static void NpiSerialCallback( uint8 port, uint8 events )
{
//    (void)port;
//    uint8 numBytes = 0;
//    uint8 buf[128];
//    
//    if (events & HAL_UART_RX_TIMEOUT)   //串口有数据
//    {
//        numBytes = NPI_RxBufLen();       //读出串口缓冲区有多少字节
//        
//        
//        //NPI_PrintValue("numBytes = ", numBytes, 10);
//        //NPI_PrintString("\n");
//        
//        
//        
//        if(numBytes)
//        {     
//            if ( ( p->simpleBLEState == BLE_STATE_CONNECTED  ) && ( simpleBLECharHd6 != 0 ) ) //已连接并获取完CHAR6的Handle就写CHAR6
//            {
//                if(simpleBLEChar6DoWrite)               //写入成功后再写入
//                {
//                    attWriteReq_t AttReq;       
//                    if ( numBytes >= SIMPLEPROFILE_CHAR6_LEN ) buf[0] = SIMPLEPROFILE_CHAR6_LEN-1;
//                    else buf[0] = numBytes;
//                    NPI_ReadTransport(&buf[1],buf[0]);    //从串口读出数据
//                    
//                    AttReq.handle = simpleBLECharHd6;
//                    AttReq.len = SIMPLEPROFILE_CHAR6_LEN;
//                    AttReq.sig = 0;
//                    AttReq.cmd = 0;
//                    osal_memcpy(AttReq.value,buf,SIMPLEPROFILE_CHAR6_LEN);
//                    GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
//                    simpleBLEChar6DoWrite = FALSE;
//                }
//            }
//            else
//            {
//                NPI_WriteTransport("Not Ready\n", 10 ); 
//                NPI_ReadTransport(buf,numBytes);     //释放串口数据
//            }
//        }
//    }
}


/**
 * 该函数为数据写函数
 *
 * index        从机下标0, 1, 2 等
 * charIdx      特征值索引 CHAR1~ CHAR7
 * value[20]    要发送的数据 buffer
 * len          要发送的数据长度，要注意 char1 ~ char 4 都只是一个字节的
 *              CHAR5是5个字节，CHAR6 为15个字节， 那CHAR7为notice 20字节
 */
static void oneConnectedDev_CharX_Option(uint8 index, uint8 charIdx, uint8 value[20], uint8 len)
{
    ble_dev_t *p = &(__g_dev[index]);
    
    if(p->simpleBLEState == BLE_STATE_CONNECTED &&
       p->simpleBLECharHdl[charIdx] != 0        &&
        p->simpleBLEConnHandle != GAP_CONNHANDLE_INIT) {
               
        uint8 status;
               
        if( p->simpleBLEDoWrite ) {
            // 执行写操作
            attWriteReq_t req;
                   
            req.handle = p->simpleBLECharHdl[charIdx];
            req.len = len;
                   
            osal_memcpy(req.value, value, len);
                   
            req.sig = 0;
            req.cmd = 0;
                   
            status = GATT_WriteCharValue(p->simpleBLEConnHandle, &req, simpleBLETaskId);
                   
            if (SUCCESS != status) {
                NPI_PrintValue("Write Error: ", status, 10);
            }
                   
        } else {
            // 执行读操作  保留  这里用不到   先写着备用
            attReadReq_t req;
            req.handle = p->simpleBLECharHdl[charIdx];
            NPI_PrintValue("Do read : ", req.handle, 10);
            status = GATT_ReadCharValue(p->simpleBLEConnHandle, &req, simpleBLETaskId); 
        }
               
        p->simpleBLEProcedureInProgress = TRUE;
               
    }
}
/*********************************************************************
*********************************************************************/
