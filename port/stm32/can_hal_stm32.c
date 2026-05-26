#include "can_hal_stm32.h"
#include <string.h>

static FDCAN_HandleTypeDef *stm32_fdcan = NULL;

static uint32_t can_hal_stm32_dlc_to_fdcan(uint8_t dlc)
{
    switch (dlc)
    {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        case 8: return FDCAN_DLC_BYTES_8;
        default: return FDCAN_DLC_BYTES_8;
    }
}

static uint8_t can_hal_stm32_fdcan_to_dlc(uint32_t fdcan_dlc)
{
    switch (fdcan_dlc)
    {
        case FDCAN_DLC_BYTES_0: return 0;
        case FDCAN_DLC_BYTES_1: return 1;
        case FDCAN_DLC_BYTES_2: return 2;
        case FDCAN_DLC_BYTES_3: return 3;
        case FDCAN_DLC_BYTES_4: return 4;
        case FDCAN_DLC_BYTES_5: return 5;
        case FDCAN_DLC_BYTES_6: return 6;
        case FDCAN_DLC_BYTES_7: return 7;
        case FDCAN_DLC_BYTES_8: return 8;
        default: return 0;
    }
}


can_hal_status_t can_hal_stm32_init(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == NULL)
    {
        return CAN_HAL_ERROR;
    }

    stm32_fdcan = hfdcan;

    if (HAL_FDCAN_Start(stm32_fdcan) != HAL_OK)
    {
        return CAN_HAL_ERROR;
    }

    return CAN_HAL_OK;
}

can_hal_status_t can_hal_stm32_send(const can_frame_t *frame)
{
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_data[8];

    if ((stm32_fdcan == NULL) || (frame == NULL))
    {
        return CAN_HAL_ERROR;
    }

    if (frame->id > 0x7FFU)
    {
        return CAN_HAL_ERROR;
    }

    if (frame->dlc > 8U)
    {
        return CAN_HAL_ERROR;
    }

    memset(&tx_header, 0, sizeof(tx_header));
    memset(tx_data, 0, sizeof(tx_data));

    tx_header.Identifier = frame->id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = can_hal_stm32_dlc_to_fdcan(frame->dlc);
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    memcpy(tx_data, frame->data, frame->dlc);
    int fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(stm32_fdcan);
    while(fifo_free_level<1){
    	fifo_free_level = HAL_FDCAN_GetTxFifoFreeLevel(stm32_fdcan);
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(stm32_fdcan, &tx_header, tx_data) != HAL_OK)
    {
        return CAN_HAL_ERROR;
    }

    return CAN_HAL_OK;
}


can_hal_status_t can_hal_stm32_receive(can_frame_t *frame)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    uint8_t dlc;

    if ((stm32_fdcan == NULL) || (frame == NULL))
    {
        return CAN_HAL_ERROR;
    }

    if (HAL_FDCAN_GetRxFifoFillLevel(stm32_fdcan, FDCAN_RX_FIFO0) == 0U)
    {
        return CAN_HAL_RX_EMPTY;
    }

    memset(&rx_header, 0, sizeof(rx_header));
    memset(rx_data, 0, sizeof(rx_data));

    if (HAL_FDCAN_GetRxMessage(stm32_fdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
        return CAN_HAL_ERROR;
    }

    if (rx_header.IdType != FDCAN_STANDARD_ID)
    {
        return CAN_HAL_ERROR;
    }

//    if (rx_header.RxFrameType != FDCAN_DATA_FRAME)
//    {
//        return CAN_HAL_ERROR;
//    }

    dlc = can_hal_stm32_fdcan_to_dlc(rx_header.DataLength);

    frame->id = rx_header.Identifier;
    frame->dlc = dlc;

    memset(frame->data, 0, sizeof(frame->data));
    memcpy(frame->data, rx_data, dlc);

    return CAN_HAL_OK;
}


can_hal_t can0 =
{
    .send = can_hal_stm32_send,
    .receive = can_hal_stm32_receive
};




