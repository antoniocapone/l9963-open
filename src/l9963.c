/**
 * @file l9963.c
 * @brief L9963 high level driver implementation
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

L9963_Status L9963_StartOnDemandConversion(L9963_Handle* handle, uint8_t dev_id,
										   L9963_CellConvTimeEnum adc_filter_soc,
										   uint8_t gpio_conv_en, uint8_t hwsc) {
	L9963_Status status = L9963_OK;
	L9963_RegisterUnion ADCV_CONV_reg;

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_ADCV_CONV_ADDR,
									   &ADCV_CONV_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	ADCV_CONV_reg.ADCV_CONV.SOC = 1U;
	ADCV_CONV_reg.ADCV_CONV.ADC_FILTER_SOC = adc_filter_soc;
	ADCV_CONV_reg.ADCV_CONV.GPIO_CONV = gpio_conv_en & 1U;
	ADCV_CONV_reg.ADCV_CONV.HWSC = hwsc; /* Perform the Hardware Self-Check during convertion */

	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_ADCV_CONV_ADDR,
										&ADCV_CONV_reg, 50);

	return status;
}

L9963_Status L9963_IsOnDemandConversionFinished(L9963_Handle* handle, uint8_t dev_id,
												uint8_t* finished) {
	L9963_Status status = L9963_OK;
	L9963_RegisterUnion ADCV_CONV_reg;

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_ADCV_CONV_ADDR,
									   &ADCV_CONV_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	*finished = ADCV_CONV_reg.ADCV_CONV.DUTY_ON;

	return status;
}

L9963_Status L9963_ReadCellVoltage(L9963_Handle* handle, uint8_t dev_id, L9963_CellsEnum cell,
								   uint16_t *vcell, uint8_t* data_ready) {

	L9963_Status status = L9963_OK;
	L9963_RegistersAddr addr;
	L9963_RegisterUnion Vcellx_reg = {0};

	/* Searching for cell register address */
	switch (cell) {
		case L9963_CELL1:
			addr = L9963_Vcell1_ADDR;
			break;
		case L9963_CELL2:
			addr = L9963_Vcell2_ADDR;
			break;
		case L9963_CELL3:
			addr = L9963_Vcell3_ADDR;
			break;
		case L9963_CELL4:
			addr = L9963_Vcell4_ADDR;
			break;
		case L9963_CELL5:
			addr = L9963_Vcell5_ADDR;
			break;
		case L9963_CELL6:
			addr = L9963_Vcell6_ADDR;
			break;
		case L9963_CELL7:
			addr = L9963_Vcell7_ADDR;
			break;
		case L9963_CELL8:
			addr = L9963_Vcell8_ADDR;
			break;
		case L9963_CELL9:
			addr = L9963_Vcell9_ADDR;
			break;
		case L9963_CELL10:
			addr = L9963_Vcell10_ADDR;
			break;
		case L9963_CELL11:
			addr = L9963_Vcell11_ADDR;
			break;
		case L9963_CELL12:
			addr = L9963_Vcell12_ADDR;
			break;
		case L9963_CELL13:
			addr = L9963_Vcell13_ADDR;
			break;
		case L9963_CELL14:
			addr = L9963_Vcell14_ADDR;
			break;
		default:
			addr = L9963_Vcell1_ADDR;
			break;
	}

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, addr, &Vcellx_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	/* Here we use Vcell1 because all Vcellx register structures are the same at bit level */
	*vcell = Vcellx_reg.Vcell1.VCell1;
	*data_ready = Vcellx_reg.Vcell1.d_rdy_Vcell1;

	return status;
}

L9963_Status L9963_ReadBatteryVoltage(L9963_Handle* handle, uint8_t dev_id, uint16_t* vbatt_monitor,
									  uint32_t* vbatt_sum) {

	L9963_Status status = L9963_OK;
	L9963_RegisterUnion vbattdiv_reg = {0};
	L9963_RegisterUnion vsumbatt_reg = {0};

	/* Perform VBATT_DIV register read */
	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_VBATTDIV_ADDR,
									   &vbattdiv_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	/* Perform VSUM_BATT register read */
	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_VSUMBATT_ADDR,
									   &vsumbatt_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	/**
	 * vsum_batt is the digital sum of all cells voltage measurements. The more significant 18 bit
	 * of vsum_batt are in VSUMBATT register, the 2 bit less significant are in VBATTDIV register.
	 * VBATT_DIV is the direct series conversion.
	 */
	*vbatt_monitor = vbattdiv_reg.VBATTDIV.VBATT_DIV;
	*vbatt_sum = (vsumbatt_reg.VSUMBATT.vsum_batt19_2 << 2) | vbattdiv_reg.VBATTDIV.vsum_batt1_0;

	return status;
}

L9963_Status L9963_ReadGPIO(L9963_Handle* handle, uint8_t dev_id, L9963_GpiosEnum gpio,
							uint16_t* vgpio, uint8_t* data_ready) {

	L9963_Status status = L9963_OK;
	L9963_RegistersAddr addr;
	L9963_RegisterUnion gpiox_meas_reg = {0};

	switch (gpio) {
		case L9963_GPIO3:
			addr = L9963_GPIO3_MEAS_ADDR;
			break;
		case L9963_GPIO4:
			addr = L9963_GPIO4_MEAS_ADDR;
			break;
		case L9963_GPIO5:
			addr = L9963_GPIO5_MEAS_ADDR;
			break;
		case L9963_GPIO6:
			addr = L9963_GPIO6_MEAS_ADDR;
			break;
		case L9963_GPIO7:
			addr = L9963_GPIO7_MEAS_ADDR;
			break;
		case L9963_GPIO8:
			addr = L9963_GPIO8_MEAS_ADDR;
			break;
		case L9963_GPIO9:
			addr = L9963_GPIO9_MEAS_ADDR;
			break;
		default:
			addr = L9963_GPIO3_MEAS_ADDR;
			break;
	}

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, addr, &gpiox_meas_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	/* Here we use GPIO3_MEAS because all GPIOx_MEAS register structures are the same at bit level */
	*vgpio = gpiox_meas_reg.GPIO3_MEAS.GPIO3_MEAS;
	*data_ready = gpiox_meas_reg.GPIO3_MEAS.d_rdy_gpio3;

	return status;
}

L9963_Status L9963_EnableDisableCoulombCounting(L9963_Handle* handle, uint8_t dev_id,
												uint8_t enable) {

	L9963_Status status = L9963_OK;
	L9963_RegisterUnion csa_gpio_msk_reg;

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_CSA_GPIO_MSK_ADDR,
									   &csa_gpio_msk_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	csa_gpio_msk_reg.CSA_GPIO_MSK.CoulombCounter_en = enable & 1U;
	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_CSA_GPIO_MSK_ADDR,
									   &csa_gpio_msk_reg, 50);

	return status;
}

L9963_Status L9963_GetCoulombCounterSamples(L9963_Handle* handle, uint8_t dev_id,
											uint16_t* n_samples) {

	L9963_Status status = L9963_OK;
	L9963_RegisterUnion CoulCntrTime_reg;

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_CoulCntrTime_ADDR,
									   &CoulCntrTime_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	*n_samples = CoulCntrTime_reg.CoulCntrTime.CoulombCntTime;

	return status;
}

L9963_Status L9963_SetBalancingMode(L9963_Handle* handle, uint8_t dev_id,
									L9963_Balancing_Mode bal_mode) {

	L9963_Status status = L9963_OK;
	L9963_RegisterUnion Bal_2_reg;

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_Bal_2_ADDR, &Bal_2_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	Bal_2_reg.Bal_2.Balmode = bal_mode;
	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_Bal_2_ADDR, &Bal_2_reg, 50);

	return status;
}

L9963_Status L9963_SetBalancingCells(L9963_Handle* handle, uint8_t dev_id, uint16_t bal_mask) {
	L9963_Status status = L9963_OK;
	L9963_RegisterUnion BalCell14_7act_reg, BalCell6_1act_reg;

	/* Cells from 14 to 7 */
	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_BalCell14_7act_ADDR,
									   &BalCell14_7act_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	for (uint8_t i = 14; i >= 7; i--) {
		switch (i) {
			case L9963_CELL14:
			BalCell14_7act_reg.BalCell14_7act.BAL14 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			case L9963_CELL13:
			BalCell14_7act_reg.BalCell14_7act.BAL13 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			case L9963_CELL12:
			BalCell14_7act_reg.BalCell14_7act.BAL12 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			case L9963_CELL11:
			BalCell14_7act_reg.BalCell14_7act.BAL11 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			case L9963_CELL10:
			BalCell14_7act_reg.BalCell14_7act.BAL10 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			case L9963_CELL9:
			BalCell14_7act_reg.BalCell14_7act.BAL9 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			case L9963_CELL8:
			BalCell14_7act_reg.BalCell14_7act.BAL8 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			case L9963_CELL7:
			BalCell14_7act_reg.BalCell14_7act.BAL7 = bal_mask & (1 << i) ? 0b10 : 0b00;
			break;
			default:
			/* I think this case is impossible */
			break;
		}
	}

	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_BalCell14_7act_ADDR,
										&BalCell14_7act_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	/* Cells from 6 to 1 */
	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_BalCell6_1act_ADDR,
									   &BalCell6_1act_reg, 50);
	if (status != 0) {
		return status;
	}

	for (uint8_t i = 6; i >= 1; i--) {
		switch (i) {
			case L9963_CELL6:
				BalCell6_1act_reg.BalCell6_1act.BAL6 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			case L9963_CELL5:
				BalCell6_1act_reg.BalCell6_1act.BAL5 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			case L9963_CELL4:
				BalCell6_1act_reg.BalCell6_1act.BAL4 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			case L9963_CELL3:
				BalCell6_1act_reg.BalCell6_1act.BAL3 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			case L9963_CELL2:
				BalCell6_1act_reg.BalCell6_1act.BAL2 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			case L9963_CELL1:
				BalCell6_1act_reg.BalCell6_1act.BAL1 = bal_mask & (1 << i) ? 0b10 : 0b00;
				break;
			default:
				/* I think this case is impossible */
				break;
		}
	}

	status = L9963_Driver_RegisterWrite(&handle->drv_handle, dev_id, L9963_BalCell6_1act_ADDR,
										&BalCell6_1act_reg, 50);

	return status;
}

L9963_Status L9963_StartStopBalancing(L9963_Handle* handle, uint8_t dev_id, uint8_t start) {
	if (start != 0 && start != 1) {
		return L9963_NOT_OK;
	}

	L9963_Status status = L9963_OK;
	L9963_RegisterUnion Bal_1_reg;

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_Bal_1_ADDR, &Bal_1_reg, 50);
	if (status != L9963_OK) {
		return status;
	}

	if (start == 1) {
		Bal_1_reg.Bal_1.bal_start = 1;
		Bal_1_reg.Bal_1.bal_stop = 0;
	} else {
		Bal_1_reg.Bal_1.bal_start = 0;
		Bal_1_reg.Bal_1.bal_stop = 1;
	}

	status = L9963_Driver_RegisterRead(&handle->drv_handle, dev_id, L9963_Bal_1_ADDR, &Bal_1_reg, 50);

	return status;
}
