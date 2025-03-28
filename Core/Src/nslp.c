#include "nslp.h"


// Receive instance
uint8_t receiveData[HEADER_SIZE + 255 + CHECKSUM_SIZE];
struct Packet receiveInstance = {
	0x00,
	0,
	NULL
};

struct Packet* receive_packet(UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc) {
	// Wait for frame start
	uint8_t frameStart = 0;
	if(HAL_UART_Receive(huart, &frameStart, FRAME_START_SIZE, 100) != HAL_OK) {
		return &receiveInstance;
	}

	if(frameStart != FRAME_START) {
		return &receiveInstance;
	}

	// Receive header
	if(HAL_UART_Receive(huart, receiveData, HEADER_SIZE, 100) != HAL_OK) {
		return &receiveInstance;
	}

	receiveInstance.type = (char) receiveData[0];
	receiveInstance.size = receiveData[1];

	if (receiveInstance.size == 0) {
		return &receiveInstance;
	}

	// Read data
	if(HAL_UART_Receive_DMA(huart, &receiveData[HEADER_SIZE], receiveInstance.size) != HAL_OK) {
		return &receiveInstance;
	}
	receiveInstance.payload = &receiveData[HEADER_SIZE];

	// Read checksum
	if(HAL_UART_Receive_DMA(huart, &receiveData[HEADER_SIZE + receiveInstance.size], CHECKSUM_SIZE) != HAL_OK) {
		return &receiveInstance;
	}

	// Perform checksum
	uint32_t calculatedCrc = HAL_CRC_Calculate(
		hcrc,
		(uint32_t *) receiveData,
		HEADER_SIZE + receiveInstance.size
	);

	uint32_t receivedCrc = *(uint32_t *) (&receiveData[HEADER_SIZE + receiveInstance.size]);
	if (receivedCrc != calculatedCrc) {
		return &receiveInstance;
	}

	return &receiveInstance;
}

void send_packet(struct Packet *p, UART_HandleTypeDef *huart, CRC_HandleTypeDef *hcrc) {
	size_t packetSize = HEADER_SIZE + p->size;
	size_t totalPacketSize = FRAME_START_SIZE + packetSize + CHECKSUM_SIZE;
	uint8_t pData[totalPacketSize];

	pData[0] = FRAME_START;
	pData[1] = (uint8_t) p->type;
	pData[2] = p->size;

	const uint8_t *data = p->payload;
	for (uint8_t i = 0; i < p->size; i++) {
		pData[FRAME_START_SIZE + HEADER_SIZE + i] = data[i];
	}

	uint32_t crc = HAL_CRC_Calculate(hcrc, (uint32_t *) &pData[1], packetSize);
	pData[totalPacketSize - 4] = (uint8_t) (crc & 0xFF);
	pData[totalPacketSize - 3] = (uint8_t) ((crc >> 8) & 0xFF);
	pData[totalPacketSize - 2] = (uint8_t) ((crc >> 16) & 0xFF);
	pData[totalPacketSize - 1] = (uint8_t) ((crc >> 24) & 0xFF);

	HAL_UART_Transmit(huart, pData, totalPacketSize, 100);

}
