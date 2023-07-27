#include "canard_stm32.h"
#include "canard.h"

static CanardInstance canard;
static uint8_t memory_pool[1024];

void init_can_node()
{
    
  CanardSTM32CANTimings stm32Timings = {0};
  canardSTM32Init(&stm32Timings, CanardSTM32IfaceModeSilent);
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
}

/*
  Transmits all frames from the TX queue, receives up to one frame.
*/
void processTxRxOnce(int32_t timeout_msec)
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
        // (void)fprintf(stderr, "Receive error %d, errno '%s'\n", rx_res, strerror(errno));
    }
    else if (rx_res > 0)        // Success - process the frame
    {
        canardHandleRxFrame(&canard, &rx_frame, timestamp);
    }
}

/*
  This function is called at 1 Hz rate from the main loop.
*/
void process1HzTasks(uint64_t timestamp_usec)
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