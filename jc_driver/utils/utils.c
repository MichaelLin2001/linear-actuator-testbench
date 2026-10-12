#include "utils.h"

void dump_buffer(const uint8_t* buffer, uint32_t length)
{
    for (int i = 0; i < length; i++) {
        if ((i % 8 != 0 || i == 0) && i != (length - 1)) {
            printf("0x%02x-", buffer[i]);
        } else {
            printf("0x%02x\n", buffer[i]);
        }
    }
}

uint16_t modbus_crc16(const uint8_t* data, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; ++j) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}