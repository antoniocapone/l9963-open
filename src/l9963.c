/**
 * @file l9963.c
 * @brief L9963E high level driver implementation
 *
 * @author Antonio Capone
 * @date 2025-02-17
 * @version 1.0
 */

#include "l9963.h"
#include "l9963_driver.h"
#include "l9963_platform.h"
#include <stdlib.h> /* NULL */

L9963_Status L9963_init(L9963_Handle* handle, L9963_Platform interface) {
	if (interface.L9963_Platform_DelayMs == NULL || interface.L9963_Platform_GetTickMs == NULL || interface.L9963_Platform_GPIO_WritePin == NULL || interface.L9963_Platform_SPI_Receive == NULL || interface.L9963_Platform_SPI_Transmit == NULL) {
		return L9963_NOT_OK;
	}

	L9963_Driver_Handle drv_handle;
	L9963_Driver_Init(&drv_handle, interface);

	handle->drv_handle = drv_handle;

	return L9963_OK;
}
