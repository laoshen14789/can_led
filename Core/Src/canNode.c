

/*
  A simple example DroneCAN node, this implements 2 features:

   - announces on the bus using NodeStatus at 1Hz
   - answers GetNodeInfo requests

  This example uses socketcan on Linux for CAN transport

  Example usage: ./simple_node vcan0
*/
/*
 This example application is distributed under the terms of CC0 (public domain dedication).
 More info: https://creativecommons.org/publicdomain/zero/1.0/
*/

#ifndef _GNU_SOURCE
# define _GNU_SOURCE
#endif

#include "canard.h"
#include "canard_stm32.h"
#include "stm32f1xx_hal.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <stdbool.h>

// include the headers for the generated DroneCAN messages from the
// dronecan_dsdlc compiler
#include <dronecan_msgs.h>

#include "gpio.h"
/*
  libcanard library instance and a memory pool for it to use
 */
static CanardInstance canard;
static uint8_t memory_pool[1024];

/*
  in this simple example we will use a fixed CAN node ID. This would
  need to be a parameter or use dynamic node allocation in a real
  application
 */
#define MY_NODE_ID 97
// some convenience macros
#define MIN(a,b) ((a)<(b)?(a):(b))
#define C_TO_KELVIN(temp) (temp + 273.15f)
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(x[0]))

static struct parameter {
    char *name;
    enum uavcan_protocol_param_Value_type_t type;
    float value;
    float min_value;
    float max_value;
} parameters[] = {
    { "CAN_NODE", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, MY_NODE_ID, 0, 127 },
    { "STATUS_LED_SWITCH", UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE, 1, 0, 1 },
    { "LED1_SWITCH", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 1, 0, 2 },
    { "LED2_SWITCH", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 1, 0, 2 },
    { "LED3_SWITCH", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 1, 0, 2 },
    { "LED4_SWITCH", UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE, 1, 0, 2 },
};

/*
  hold our node status as a static variable. It will be updated on any errors
 */
static struct uavcan_protocol_NodeStatus node_status;

uint32_t GetUid(uint8_t* pUid)
{
	uint32_t chipId[3] = {0};
		
	//获取CPU唯一ID
	#if 1//STM32F1系列
	chipId[0] =*(volatile unsigned long *)(0x1ffff7e8); //按全字（32位）读取
	chipId[1] =*(volatile unsigned long *)(0x1ffff7ec);
	chipId[2] =*(volatile unsigned long *)(0x1ffff7f0);
	#endif
	
	#if 0//STM32F4系列
	chipId[0]=*(volatile unsigned long *)(0x1fff7a10);
	chipId[1]=*(volatile unsigned long *)(0x1fff7a14);
	chipId[2]=*(volatile unsigned long *)(0x1fff7a18);
//	/* printf the chipid */
//	printf("\r\n芯片的唯一ID为: %X-%X-%X\r\n",
//	            chipId[0],chipId[1],chipId[2]);
//	printf("\r\n芯片flash的容量为: %dK \r\n", *(uint16_t *)(0X1FFF7a22));
	#endif
	
	//按字节（8位）读取
	pUid[0] = (uint8_t)(chipId[0] & 0x000000FF);
	pUid[1] = (uint8_t)((chipId[0] & 0xFF00) >>8);
	pUid[2] = (uint8_t)((chipId[0] & 0xFF0000) >>16);
	pUid[3] = (uint8_t)((chipId[0] & 0xFF000000) >>24);
	
	pUid[4] = (uint8_t)(chipId[1] & 0xFF);
	pUid[5] = (uint8_t)((chipId[1] & 0xFF00) >>8);
	pUid[6] = (uint8_t)((chipId[1] & 0xFF0000) >>16);
	pUid[7] = (uint8_t)((chipId[1] & 0xFF000000) >>24);
	
	pUid[8] = (uint8_t)(chipId[2] & 0xFF);
	pUid[9] = (uint8_t)((chipId[2] & 0xFF00) >>8);
	pUid[10] = (uint8_t)((chipId[2] & 0xFF0000) >>16);
	pUid[11] = (uint8_t)((chipId[2] & 0xFF000000) >>24);

	return (chipId[0]>>1)+(chipId[1]>>2)+(chipId[2]>>3);
}

uint32_t Get_sys_time_ms(void)
{
	return HAL_GetTick();
}

uint32_t Get_sys_time_us(void)
{
  uint32_t sys_us = 1000*HAL_GetTick()+TIM4->CNT;
  return sys_us;
}

/*
  get a 64 bit monotonic timestamp in microseconds since start. This
  is platform specific
 */
static uint64_t micros64(void)
{
    return Get_sys_time_us();
}

/*
  get a 16 byte unique ID for this node, this should be based on the CPU unique ID or other unique ID
 */
static void getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    GetUid(id);
    id[12] = 0xAA;
    id[13] = 0XBB;
    id[14] = 0xCC;
    id[15] = 0XDD;
}

/*
  handle a GetNodeInfo request
*/
static void handle_GetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer)
{
    printf("GetNodeInfo request from %d\n", transfer->source_node_id);

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse pkt;

    memset(&pkt, 0, sizeof(pkt));

    node_status.uptime_sec = micros64() / 1000000ULL;
    pkt.status = node_status;

    // fill in your major and minor firmware version
    pkt.software_version.major = 1;
    pkt.software_version.minor = 2;
    pkt.software_version.optional_field_flags = 0;
    pkt.software_version.vcs_commit = 0; // should put git hash in here

    // should fill in hardware version
    pkt.hardware_version.major = 2;
    pkt.hardware_version.minor = 3;

    getUniqueID(pkt.hardware_version.unique_id);

    strncpy((char*)pkt.name.data, "114514Node", sizeof(pkt.name.data));
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter GetSet request
 */
static void handle_param_GetSet(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

    struct parameter *p = NULL;
    if (req.name.len != 0) {
        for (uint16_t i=0; i<ARRAY_SIZE(parameters); i++) {
            if (req.name.len == strlen(parameters[i].name) &&
                strncmp((const char *)req.name.data, parameters[i].name, req.name.len) == 0) {
                p = &parameters[i];
                break;
            }
        }
    } else if (req.index < ARRAY_SIZE(parameters)) {
        p = &parameters[req.index];
    }
    if (p != NULL && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        /*
          this is a parameter set command. The implementation can
          either choose to store the value in a persistent manner
          immediately or can instead store it in memory and save to permanent storage on a
         */
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            p->value = req.value.integer_value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            p->value = req.value.boolean_value;
            break;
        default:
            return;
        }
    }

    /*
      for both set and get we reply with the current value
     */
    struct uavcan_protocol_param_GetSetResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    if (p != NULL) {
        pkt.value.union_tag = p->type;
        switch (p->type) {
        case UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE:
            pkt.value.integer_value = p->value;
            break;
        case UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE:
            pkt.value.boolean_value = p->value;
            break;
        default:
            return;
        }
        pkt.name.len = strlen(p->name);
        strcpy((char *)pkt.name.data, p->name);
    }

    if(parameters[1].value == 1)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
    // if(parameters[2].value == 1)
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    // }
    // else
    // {
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    // }
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
  handle parameter executeopcode request
 */
static void handle_param_ExecuteOpcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        // here is where you would reset all parameters to defaults
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        // here is where you would save all the changed parameters to permanent storage
    }

    struct uavcan_protocol_param_ExecuteOpcodeResponse pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer);

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size);
}

/*
 This callback is invoked by the library when a new message or request or response is received.
*/
static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    // switch on data type ID to pass to the right handler function
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            handle_GetNodeInfo(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            handle_param_GetSet(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            handle_param_ExecuteOpcode(ins, transfer);
            break;
        }
        }
    }
}


/*
 This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 by the local node.
 If the callback returns true, the library will receive the transfer.
 If the callback returns false, the library will ignore the transfer.
 All transfers that are addressed to other nodes are always ignored.

 This function must fill in the out_data_type_signature to be the signature of the message.
 */
static bool shouldAcceptTransfer(const CanardInstance *ins,
                                 uint64_t *out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    if (transfer_type == CanardTransferTypeRequest) {
        // check if we want to handle a specific service request
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_REQUEST_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        }
    }
    // we don't want any other messages
    return false;
}


/*
  send the 1Hz NodeStatus message. This is what allows a node to show
  up in the DroneCAN GUI tool and in the flight controller logs
 */
static void send_NodeStatus(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    node_status.uptime_sec = micros64() / 1000000ULL;
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status.sub_mode = 0;
    // put whatever you like in here for display in GUI
    node_status.vendor_specific_status_code = 1234;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer);

    // we need a static variable for the transfer ID. This is
    // incremeneted on each transfer, allowing for detection of packet
    // loss
    static uint8_t transfer_id;

    canardBroadcast(&canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
      Purge transfers that are no longer transmitted. This can free up some memory
    */
    canardCleanupStaleTransfers(&canard, timestamp_usec);

    /*
     Transmit the node status message
     */
    send_NodeStatus();
}


/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
static void processTxRxOnce(int32_t timeout_msec)
{
    // Transmitting
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&canard)) != NULL;) {
        const int16_t tx_res = canardSTM32Transmit(txf);
        if (tx_res < 0) {         // Failure - drop the frame
            canardPopTxQueue(&canard);
        }
        else if (tx_res > 0)    // Success - just drop the frame
        {
            canardPopTxQueue(&canard);
        }
        else                    // Timeout - just exit and try again later
        {
            break;
        }
    }

    // Receiving
    CanardCANFrame rx_frame;

    const uint64_t timestamp = micros64();
    const int16_t rx_res = canardSTM32Receive(&rx_frame);
    if (rx_res < 0) {
        (void)fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res, strerror(errno));
    }
    else if (rx_res > 0)        // Success - process the frame
    {
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

/*
  main program entry point
 */
int init_can_node()
{
    CanardSTM32CANTimings CANTiming;
    CANTiming.bit_rate_prescaler = 3;
    CANTiming.bit_segment_1 = 5;
    CANTiming.bit_segment_2 = 2;
    CANTiming.max_resynchronization_jump_width = 1;
    /*
     * Initializing the CAN backend driver; in this example we're using SocketCAN
     */

    int16_t res = canardSTM32Init(&CANTiming, CanardSTM32IfaceModeNormal);
    if (res < 0) {
        // (void)fprintf(stderr, "Failed to open CAN iface '%s'\n", can_iface_name);
        return 1;
    }

    /*
     Initializing the Libcanard instance.
     */
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               NULL);

    canardSetLocalNodeID(&canard, MY_NODE_ID);

    /*
      Run the main loop.
     */
    uint64_t next_1hz_service_at = micros64();

    while (true) {
        processTxRxOnce(10);

        const uint64_t ts = micros64();

        if (ts >= next_1hz_service_at) {
            next_1hz_service_at += 1000000ULL;
            process1HzTasks(ts);
        }
    }

    return 0;
}
