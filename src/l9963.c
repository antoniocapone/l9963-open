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

L9963_Status L9963_SetCommTimeout(L9963_Handle* handle, L9963_CommTimeoutEnum commTimeout, uint8_t dev_id) {
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

L9963_Status L9963_GetFsmStatus(L9963_Handle* handle, uint8_t dev_id, L9963_FSM_Status* status) {
	L9963_Status ret = L9963_OK;
	L9963_RegisterUnion fsm_reg;

	ret = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_FSM_ADDR, &fsm_reg, 50);
	if (ret != L9963_OK) {
		return ret;
	}

	*status = (L9963_FSM_Status)fsm_reg.FSM.FSMstatus;

	return ret;
}

L9963_Status L9963_SoftwareReset(L9963_Handle* handle, uint8_t dev_id) {
	L9963_Status ret = L9963_OK;
	L9963_RegisterUnion fsm_reg;

	ret = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_FSM_ADDR, &fsm_reg, 50);
	if (ret != L9963_OK) {
		return ret;
	}

	/* Perform the FSM register write with the new SW_RST value */
	fsm_reg.FSM.SW_RST = 0b10;
	ret = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_FSM_ADDR, &fsm_reg, 50);

	return ret;
}

L9963_Status L9963_GoToSleep(L9963_Handle* handle, uint8_t dev_id) {
	L9963_Status ret = L9963_OK;
	L9963_RegisterUnion fsm_reg;

	ret = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_FSM_ADDR, &fsm_reg, 50);
	if (ret != L9963_OK) {
		return ret;
	}

	/* Perform the FSM register write with the new GO2SLP value */
	fsm_reg.FSM.GO2SLP = 0b10;
	ret = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_FSM_ADDR, &fsm_reg, 50);

	return ret;
}

L9963_Status L9963_SetEnabledCells(L9963_Handle* handle, uint8_t dev_id, uint16_t en_cells_mask) {
	L9963_Status status = L9963_OK;
	L9963_RegisterUnion vcells_en_reg = {0};

	for (uint8_t i = 0; i < L9963_TOTAL_CELLS; i++) {
		if (en_cells_mask & (1 << i)) {
			// the i-th cell must be enabled
			switch (i + 1) {
				case 1:
					vcells_en_reg.VCELLS_EN.VCELL1_EN = 1;
					break;
				case 2:
					vcells_en_reg.VCELLS_EN.VCELL2_EN = 1;
					break;
				case 3:
					vcells_en_reg.VCELLS_EN.VCELL3_EN = 1;
					break;
				case 4:
					vcells_en_reg.VCELLS_EN.VCELL4_EN = 1;
					break;
				case 5:
					vcells_en_reg.VCELLS_EN.VCELL5_EN = 1;
					break;
				case 6:
					vcells_en_reg.VCELLS_EN.VCELL6_EN = 1;
					break;
				case 7:
					vcells_en_reg.VCELLS_EN.VCELL7_EN = 1;
					break;
				case 8:
					vcells_en_reg.VCELLS_EN.VCELL8_EN = 1;
					break;
				case 9:
					vcells_en_reg.VCELLS_EN.VCELL9_EN = 1;
					break;
				case 10:
					vcells_en_reg.VCELLS_EN.VCELL10_EN = 1;
					break;
				case 11:
					vcells_en_reg.VCELLS_EN.VCELL11_EN = 1;
					break;
				case 12:
					vcells_en_reg.VCELLS_EN.VCELL12_EN = 1;
					break;
				case 13:
					vcells_en_reg.VCELLS_EN.VCELL13_EN = 1;
					break;
				case 14:
					vcells_en_reg.VCELLS_EN.VCELL14_EN = 1;
					break;
				default:
					break;
			}
		}
	}

	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_VCELLS_EN_ADDR,
										&vcells_en_reg, 50);

	return status;
}
