/*
 * plm_data.c
 *
 *  Created on: Mar 20, 2023
 *      Author: jonathan
 */

#include "plm_data.h"
#include "GopherCAN.h"

static uint8_t b1[PLM_SD_BUFFER_SIZE];
static PLM_BUFFER buffer1 = {
    .bytes = b1,
    .size = PLM_SD_BUFFER_SIZE,
    .fill = 0
};

static uint8_t b2[PLM_SD_BUFFER_SIZE];
static PLM_BUFFER buffer2 = {
    .bytes = b2,
    .size = PLM_SD_BUFFER_SIZE,
    .fill = 0
};

static uint8_t b3[PLM_XB_BUFFER_SIZE];
static PLM_BUFFER buffer3 = {
    .bytes = b3,
    .size = PLM_XB_BUFFER_SIZE,
    .fill = 0
};

static uint8_t b4[PLM_XB_BUFFER_SIZE];
static PLM_BUFFER buffer4 = {
    .bytes = b4,
    .size = PLM_XB_BUFFER_SIZE,
    .fill = 0
};

PLM_DBL_BUFFER SD_DB = {
    .buffers = { &buffer1, &buffer2 },
    .write_index = 0,
    .tx_cplt = 1
};

PLM_DBL_BUFFER XB_DB = {
    .buffers = { &buffer3, &buffer4 },
    .write_index = 0,
    .tx_cplt = 1
};

static void append_byte(PLM_BUFFER* buffer, uint8_t byte);

PLM_RES plm_data_record_param(PLM_BUFFER* buffer, CAN_INFO_STRUCT* param) {
    uint32_t timestamp = param->last_rx;
    uint16_t id = param->ID;
    void* data = NULL;
    uint8_t checksum = 0;
    U8* bytes;
    U8 i;

    // make sure packet will fit
    uint8_t packet_size = 1 + sizeof(timestamp) + sizeof(id) + param->SIZE + sizeof(checksum);
    if (packet_size * 2 > buffer->size - buffer->fill) {
        packetsDropped_ul.data += 1;
        packetsDropped_ul.info.last_rx = HAL_GetTick();
        return PLM_ERR_BUFFER_FULL;
    }

    // get pointer to data
    switch (param->TYPE) {
        case UNSIGNED8:
            data = &((U8_CAN_STRUCT*)param)->data;
            break;
        case UNSIGNED16:
            data = &((U16_CAN_STRUCT*)param)->data;
            break;
        case UNSIGNED32:
            data = &((U32_CAN_STRUCT*)param)->data;
            break;
        case UNSIGNED64:
            data = &((U64_CAN_STRUCT*)param)->data;
            break;
        case SIGNED8:
            data = &((S8_CAN_STRUCT*)param)->data;
            break;
        case SIGNED16:
            data = &((S16_CAN_STRUCT*)param)->data;
            break;
        case SIGNED32:
            data = &((S32_CAN_STRUCT*)param)->data;
            break;
        case SIGNED64:
            data = &((S64_CAN_STRUCT*)param)->data;
            break;
        case FLOATING:
            data = &((FLOAT_CAN_STRUCT*)param)->data;
            break;
        default:
            return PLM_ERR_PACKET;
    }

    // begin writing packet to buffer
    buffer->bytes[buffer->fill++] = START_BYTE;
    checksum += START_BYTE;

    // append components with MSB first
	bytes = (U8*) &(timestamp);
	for (i = sizeof(timestamp); i > 0; i--)
	{
		append_byte(buffer, bytes[i - 1]);
		checksum += bytes[i - 1];
	}

	bytes = (U8*) &(id);
	for (i = sizeof(id); i > 0; i--)
	{
		append_byte(buffer, bytes[i - 1]);
		checksum += bytes[i - 1];
	}

	bytes = (U8*) data;
	for (i = param->SIZE; i > 0; i--)
	{
		append_byte(buffer, bytes[i - 1]);
		checksum += bytes[i - 1];
	}

	// NOTE: if there is an escape character in the checksum that is
	// not counted towards the total checksum
	append_byte(buffer, checksum);

	packetsLogged_ul.data += 1;
	packetsLogged_ul.info.last_rx = HAL_GetTick();

    return PLM_OK;
}

static void append_byte(PLM_BUFFER *buffer, uint8_t byte) {
    // check for a control byte
    if (byte == START_BYTE || byte == ESCAPE_BYTE) {
        // append escape byte
        buffer->bytes[buffer->fill++] = ESCAPE_BYTE;
        // append the desired byte, escaped
        buffer->bytes[buffer->fill++] = byte ^ ESCAPE_XOR;
    } else {
        // append the raw byte
        buffer->bytes[buffer->fill++] = byte;
    }
}
