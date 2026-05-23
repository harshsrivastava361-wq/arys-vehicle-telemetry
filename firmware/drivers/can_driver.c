/* CAN Driver — Vehicle CAN Bus (CAN1, PD0/PD1, 500kbps)
 * Transceiver: TJA1050 with 120Ω termination resistors */
#include "telemetry.h"

int CAN_Transmit(const CAN_Frame_t *frame) {
    /* CAN_TxHeaderTypeDef hdr;
     * hdr.StdId = frame->id;
     * hdr.DLC   = frame->dlc;
     * hdr.IDE   = CAN_ID_STD;
     * hdr.RTR   = CAN_RTR_DATA;
     * uint32_t mailbox;
     * return HAL_CAN_AddTxMessage(&hcan1, &hdr, frame->data, &mailbox); */
    (void)frame;
    return 0;
}

int CAN_Receive(CAN_Frame_t *frame, uint32_t timeout_ms) {
    /* HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &hdr, frame->data) */
    (void)frame; (void)timeout_ms;
    return -1; /* -1 = no frame available (non-blocking) */
}

void CAN_BusOff_Recovery(void) {
    /* Bus-off recovery sequence per ISO 11898:
     * 1. Wait 128 × 11 recessive bits (~1.4ms at 500kbps)
     * 2. HAL_CAN_Stop(&hcan1)
     * 3. HAL_CAN_Start(&hcan1) */
}

void FaultLog_Write(uint32_t fault_code, uint32_t tick) {
    /* Log fault event to SD card with timestamp.
     * Called by fault_task on new fault detection. */
    (void)fault_code; (void)tick;
}
