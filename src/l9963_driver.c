/**
 * @file l9963.c
 * @brief L9963E low level driver implementation
 *
 * @author Antonio Capone
 * @date 2025-02-11
 * @version 1.0
 */

#include <stdlib.h>
#include "l9963_driver.h"
#include "l9963_registers.h"

/* CRC macros */
#define WORD_LEN 40UL
#define CRC_LEN 6UL
#define CRC_POLY (uint64_t)0x0000000000000059 /*L9963 CRC Poly:x^3+x^2+x+1*/
#define CRC_SEED (uint64_t)0x0000000000000038
#define CRC_INIT_SEED_MASK (CRC_SEED << (WORD_LEN - CRC_LEN))
#define CRC_INIT_MASK (CRC_POLY << (WORD_LEN - CRC_LEN - 1))
#define FIRST_BIT_MASK ((uint64_t)1 << (WORD_LEN - 1))	// 0x80000000
#define CRC_LOWER_MASK ((uint8_t)(1 << CRC_LEN) - 1)	// 0b111

/* CRC variables */
uint8_t crc6_lut[64] = {
	0x0,  0x19, 0x32, 0x2b, 0x3d, 0x24, 0xf,  0x16, 0x23, 0x3a, 0x11, 0x8,	0x1e, 0x7,	0x2c, 0x35,
	0x1f, 0x6,	0x2d, 0x34, 0x22, 0x3b, 0x10, 0x9,	0x3c, 0x25, 0xe,  0x17, 0x1,  0x18, 0x33, 0x2a,
	0x3e, 0x27, 0xc,  0x15, 0x3,  0x1a, 0x31, 0x28, 0x1d, 0x4,	0x2f, 0x36, 0x20, 0x39, 0x12, 0xb,
	0x21, 0x38, 0x13, 0xa,	0x1c, 0x5,	0x2e, 0x37, 0x2,  0x1b, 0x30, 0x29, 0x3f, 0x26, 0xd,  0x14
};

void L9963_Driver_Init(L9963_Driver_Handle* dev, L9963_Platform interface) {
	dev->interface = interface;

	/* Deselect SPI device on startup */
	dev->interface.L9963_Platform_GPIO_WritePin(L9963_Platform_CS, L9963_Platform_GPIO_PIN_RESET);
}

void L9963_Driver_Select(L9963_Driver_Handle* dev) {
	dev->interface.L9963_Platform_GPIO_WritePin(L9963_Platform_CS, L9963_Platform_GPIO_PIN_RESET);
}

void L9963_Driver_Deselect(L9963_Driver_Handle* dev) {
	dev->interface.L9963_Platform_GPIO_WritePin(L9963_Platform_CS, L9963_Platform_GPIO_PIN_SET);
}

L9963_Status L9963_Driver_SPI_Transmit(L9963_Driver_Handle* dev, uint8_t* data, uint16_t len, uint32_t timeout) {
	L9963_Driver_Select(dev);
	L9963_Status ret = dev->interface.L9963_Platform_SPI_Transmit(data, len, timeout);
	L9963_Driver_Deselect(dev);

	return ret;
}

L9963_Status L9963_Driver_SPI_Receive(L9963_Driver_Handle* dev, uint8_t* data, uint16_t len, uint32_t timeout) {
	L9963_Driver_Select(dev);
	L9963_Status ret = dev->interface.L9963_Platform_SPI_Receive(data, len, timeout);
	L9963_Driver_Deselect(dev);

	return ret;
}

L9963_Status L9963_Driver_RegisterCmd(L9963_Driver_Handle* dev, uint8_t rw, uint8_t dev_id, L9963_RegistersAddr reg_addr, L9963_RegisterUnion* data, uint8_t timeout) {
	if (dev == NULL || data == NULL) {
		return L9963_NOT_OK;
	}

	L9963_Driver_FrameUnion frame;
	L9963_Status ret;

	uint8_t binary_frame[5]; /* 40-bit raw frame */
	L9963_Driver_BuildFrame(binary_frame, 1, rw, dev_id, reg_addr, rw ? data->generic : 0);

	/* Perform command send */
	ret = L9963_Driver_SPI_Transmit(dev, binary_frame, 5, timeout);
	if (ret != L9963_OK) {
		return ret;
	}

	/* Read response. If command was write then we get an ack-read, otherwise we get the desired register value */
	ret = L9963_Driver_WaitReceive(dev, &frame, dev_id, timeout);
	if (ret != L9963_OK) {
		return ret;
	}

	/* In case of write command, check if the ack-response corresponds with the written value */
	if (rw && frame.cmd.data != data->generic) {
		ret = L9963_NOT_OK; /* TODO: make better error handling */
	}

	return ret;
}

L9963_Status L9963_Driver_RegisterRead(L9963_Driver_Handle* dev, uint8_t dev_id, L9963_RegistersAddr reg_addr, L9963_RegisterUnion* data, uint8_t timeout) {
	return L9963_Driver_RegisterCmd(dev, 0, dev_id, reg_addr, data, timeout);
}

L9963_Status L9963_Driver_RegisterWrite(L9963_Driver_Handle* dev, uint8_t dev_id, L9963_RegistersAddr reg_addr, L9963_RegisterUnion* data, uint8_t timeout) {
	return L9963_Driver_RegisterCmd(dev, 1, dev_id, reg_addr, data, timeout);
}

L9963_Status L9963_Driver_Wakeup(L9963_Driver_Handle* dev) {
	uint8_t dummy_frame[5] = {0x0};
	return L9963_Driver_SPI_Transmit(dev, dummy_frame, 5, 50);
}

L9963_Status L9963_Driver_WaitReceive(L9963_Driver_Handle* dev, L9963_Driver_FrameUnion* frame, uint8_t dev_id, uint8_t timeout) {
	uint8_t raw[5];

	frame->cmd.addr = -1;
	frame->cmd.devid = -1;
	frame->cmd.data = -1;

	uint32_t start_tick = L9963_Driver_GetTick(dev);
	L9963_Status status = L9963_OK;

	while (frame->cmd.devid != dev_id && (L9963_Driver_GetTick(dev) - start_tick) < timeout) {
		/* TODO: status is capable to catch timeout errors? */
		status = L9963_Driver_SPI_Receive(dev, raw, 5, 10);
	}

	/* Check SPI errors */
	if (status != L9963_OK) {
		return status;
	}

	L9963_Driver_FrameSwitchEndianess(raw, (uint8_t *)&frame->val);

	/* Check packet integrity via CRC check */
	if (frame->cmd.crc != L9963_Driver_ComputeCRC(frame->val)) {
		status = L9963_NOT_OK; /* TODO: better error handling */
	}

	return status;
}

L9963_Status L9963_Driver_SendBurst(L9963_Driver_Handle* dev, uint8_t dev_id, L9963_Driver_BurstCmd command, L9963_Driver_BurstUnion* data, uint8_t expected_frames_n, uint8_t timeout) {
	if (dev == NULL || data == NULL) {
		return L9963_NOT_OK;
	}

	L9963_Driver_FrameUnion rcv_frame;
	uint8_t raw[5];

	/* Build a frame with empty data field, to perform the burst command request */
	L9963_Driver_BuildFrame(raw, 1, 0, dev_id, command, 0);
	L9963_Status status = L9963_Driver_SPI_Transmit(dev, raw, 5, timeout);
	if (status != L9963_OK) {
		return status;
	}

	/* Receive response */
	for (uint8_t i = 0; i < expected_frames_n; i++) {
		status = L9963_Driver_WaitReceive(dev, &rcv_frame, dev_id, timeout);

		if (status != L9963_OK) {
			return status;
		}

		if (rcv_frame.cmd.addr == command) {
			rcv_frame.cmd.addr = 1;
		}

		data->generics[(rcv_frame.cmd.addr & 0b11111) - 1] = rcv_frame.cmd.data;
	}

	return L9963_OK;
}

void L9963_Driver_FrameSwitchEndianess(uint8_t* in, uint8_t* out) {
	out[0] = in[4];
	out[1] = in[3];
	out[2] = in[2];
	out[3] = in[1];
	out[4] = in[0];
}

void L9963_Driver_BuildFrame(uint8_t* out, uint8_t pa, uint8_t rw, uint8_t devid, uint8_t addr_command, uint32_t data) {
	L9963_Driver_FrameUnion frame;
	frame.cmd.pa = pa;
	frame.cmd.rw_burst = rw;
	frame.cmd.devid = devid;
	frame.cmd.addr = addr_command;
	frame.cmd.data = data;
	frame.cmd.crc = L9963_Driver_ComputeCRC(frame.val);

	L9963_Driver_FrameSwitchEndianess((uint8_t*)&frame.val, out);
}

uint8_t L9963_Driver_ComputeCRC(uint64_t InputWord) {
	uint64_t TestBitMask;
	uint64_t CRCMask;
	uint8_t BitCount;
	uint8_t crc = 0;

	InputWord =
		(InputWord & 0xFFFFFFFFC0) ^ CRC_INIT_SEED_MASK; /* Clear the CRC bit in the data frame*/

	// first 4 bit executed as standard crc (shift and xor)
	// in order to reach a multiple of CRC_LEN data length
	TestBitMask = FIRST_BIT_MASK;
	CRCMask = CRC_INIT_MASK;  // 1111 <<
	BitCount = WORD_LEN % CRC_LEN;
	while (0 != BitCount--) {
		if (0 != (InputWord & TestBitMask)) {
			InputWord ^= CRCMask;
		} /* endif */
		CRCMask >>= 1;
		TestBitMask >>= 1;
	} /* endwhile */

	// then proceed with a lut-based calculation
	for (int8_t i = WORD_LEN - (WORD_LEN % CRC_LEN) - CRC_LEN; i > 0; i -= CRC_LEN) {
		crc = crc6_lut[((InputWord >> i) & 0b111111) ^ crc];
	}

	return crc;
}

uint32_t L9963_Driver_GetTick(L9963_Driver_Handle *dev) {
	return dev->interface.L9963_Platform_GetTickMs();
}

void L9963_Driver_DelayMs(L9963_Driver_Handle* dev, uint32_t delay) {
	dev->interface.L9963_Platform_DelayMs(delay);
}
