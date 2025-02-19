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

L9963_Status L9963_Init(L9963_Handle* handle, L9963_Platform interface) {
	if (interface.L9963_Platform_DelayMs == NULL || interface.L9963_Platform_GetTickMs == NULL || interface.L9963_Platform_GPIO_WritePin == NULL || interface.L9963_Platform_SPI_Receive == NULL || interface.L9963_Platform_SPI_Transmit == NULL) {
		return L9963_NOT_OK;
	}

	L9963_Driver_Init(&handle->drv_handle, interface);

	return L9963_OK;
}

L9963_Status L9963_SetDevID(L9963_Handle* handle, uint8_t dev_id) {
	L9963_Status status = L9963_OK;
	L9963_RegisterUnion to_write;

	/* Perform the wakeup from the sleep state, the device goes on the init state */
	status = L9963_Driver_Wakeup(&handle->drv_handle);
	if (status != L9963_OK) {
		return status;
	}
	L9963_Driver_DelayMs(&handle->drv_handle, L9963_T_WAKEUP_MS);

	/* Sets all DEV_GEN_CFG fields */
	to_write.generic = L9963_DEV_GEN_CFG_DEFAULT;
	to_write.DEV_GEN_CFG.chip_ID = dev_id & 0b00011111;

	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id & 0b00011111,
										L9963_DEV_GEN_CFG_ADDR, &to_write, 50);

	return status;
}

L9963_Status L9963_setCommTimeout(L9963_Handle* handle, L9963_CommTimeoutEnum commTimeout, uint8_t dev_id) {
	L9963_Status status = L9963_OK;
	L9963_RegisterUnion fastch_baluv_reg;

	/* Perform the fastch_baluv register read, in this way the other reg fields will be not changed
	 */
	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_fastch_baluv_ADDR,
									   &fastch_baluv_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	/* Perform the fastch_baluv register write with the new commTimeout value */
	fastch_baluv_reg.fastch_baluv.CommTimeout = commTimeout;
	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_fastch_baluv_ADDR,
										&fastch_baluv_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	return status;
}
