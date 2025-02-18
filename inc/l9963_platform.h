/**
 * @file l9963_platform.h
 * @brief L9963 platform dependent isolation header
 *
 * @author Antonio Capone
 * @date 2025-02-17
 * @version 1.0
 */

#ifndef __L9963_PLATFORM_H_
#define __L9963_PLATFORM_H_

#include "l9963.h"
#include "l9963_status.h"

#include <inttypes.h>

typedef enum {
	L9963_Platform_CS,
	// L9963_Platform_TXEN,
	// L9963_Platform_BNE,
	// L9963_Platform_ISOFREQ,
	// L9963_Platform_DIS
} L9963_Platform_PINS;

typedef enum {
    L9963_Platform_GPIO_PIN_RESET = 0,
    L9963_Platform_GPIO_PIN_SET,
} L9963_Platform_PinState;

// typedef L9963_Platform_PinState (*L9963_Platform_GPIO_ReadPin_Ptr)(L9963_Platform_PINS pin);

typedef L9963_Status (*L9963_Platform_GPIO_WritePin_Ptr)(L9963_Platform_PINS pin, L9963_Platform_PinState state);

typedef L9963_Status (*L9963_Platform_SPI_Receive_Ptr)(uint8_t *data, uint8_t size, uint8_t timeout_ms);

typedef L9963_Status (*L9963_Platform_SPI_Transmit_Ptr)(uint8_t *data, uint8_t size, uint8_t timeout_ms);

typedef uint32_t (*L9963_Platform_GetTickMs_Ptr)(void);

typedef void (*L9963_Platform_DelayMs_Ptr)(uint32_t delay);

typedef struct {
    // L9963_Platform_GPIO_ReadPin_Ptr L9963_Platform_GPIO_ReadPin;
    L9963_Platform_GPIO_WritePin_Ptr L9963_Platform_GPIO_WritePin;
    L9963_Platform_SPI_Receive_Ptr L9963_Platform_SPI_Receive;
    L9963_Platform_SPI_Transmit_Ptr L9963_Platform_SPI_Transmit;
    L9963_Platform_GetTickMs_Ptr L9963_Platform_GetTickMs;
    L9963_Platform_DelayMs_Ptr L9963_Platform_DelayMs;
} L9963_Platform;


#endif /* __L9963_PLATFORM_H_ */