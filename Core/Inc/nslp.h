#ifndef INC_NSLP_H_
#define INC_NSLP_H_

#include "stm32g4xx_hal.h"

#define FRAME_START 0x7E
#define FRAME_START_SIZE 1
#define HEADER_SIZE 2
#define CHECKSUM_SIZE 4

struct Packet {
	uint8_t type;
	uint8_t size;
	uint8_t *payload;
};

struct Packet* receive_packet(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc);

void send_packet(struct Packet *p, UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc);

#endif /* INC_NSLP_H_ */
