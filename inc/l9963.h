/**
 * @file l9963.h
 * @brief L9963 high level driver header
 *
 * @author Antonio Capone
 * @date 2025-02-17
 * @version 1.0
 */

#ifndef __L9963_H_
#define __L9963_H_

#include "l9963_driver.h"
#include "l9963_registers.h"
#include "l9963_status.h"
#include "l9963_platform.h"

#ifndef L9963_DEBUG
#define L9963_DEBUG 1
#endif  //L9963_DEBUG

#define L9963_T_WAKEUP_MS 2 /* Time needed for the wakeup procedure (ms) */
#define L9963_TOTAL_CELLS 14 /* All available device cells number */

typedef struct {
    L9963_Driver_Handle drv_handle;
} L9963_Handle;

typedef enum {
	L9963_CELL1 = 1,
	L9963_CELL2 = 2,
	L9963_CELL3 = 4,
	L9963_CELL4 = 8,
	L9963_CELL5 = 16,
	L9963_CELL6 = 32,
	L9963_CELL7 = 64,
	L9963_CELL8 = 128,
	L9963_CELL9 = 256,
	L9963_CELL10 = 512,
	L9963_CELL11 = 1024,
	L9963_CELL12 = 2048,
	L9963_CELL13 = 4096,
	L9963_CELL14 = 8192,
} L9963_CellsEnum; /* bitmap */

typedef enum {
	L9963_GPIO3 = 1,
	L9963_GPIO4 = 2,
	L9963_GPIO5 = 4,
	L9963_GPIO6 = 8,
	L9963_GPIO7 = 16,
	L9963_GPIO8 = 32,
	L9963_GPIO9 = 64,
} L9963_GpiosEnum;

typedef enum {
	L9963_GPIO_CONV = 1,
	L9963_BAL_TERM_CONV = 2,
	L9963_CELL_TERM_CONV = 4,
	L9963_GPIO_TERM_CONV = 8,
} L9963_StartConvertionOptEnum;

typedef enum {
	_32MS = 0b00,
	_256MS = 0b01,
	_1024MS = 0b10,
	_2048MS = 0b11
} L9963_CommTimeoutEnum;

typedef enum {
	L9963_FSM_Sleep 		= 0b0001,
	L9963_FSM_Init			= 0b0010,
	L9963_FSM_Normal 		= 0b0100,
	L9963_FSM_CyclicWakeUp 	= 0b1000
} L9963_FSM_Status;

/**
 * @brief Perform initialization of the high level driver.
 * @param[in] handle: pointer to device handler structure
 * @param[in] interface: structure that contains the user's platform dependent APIs implementations
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 */
L9963_Status L9963_Init(L9963_Handle *handle, L9963_Platform interface);

/**
 * @brief Perform the device address assignment (dev_id) in the DEV_GEN_CFG register.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 * @note For now, this function can be called only at the device startup
 */
L9963_Status L9963_SetDevID(L9963_Handle *handle, uint8_t dev_id);

/**
 * @brief Perform the SPI communication timeout.
 * @param[in] handle: pointer to device handler structure
 * @param[in] commTiemout: new value of communication timeout
 * @param[in] dev_id: 5-bit long device address
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 */
L9963_Status L9963_SetCommTimeout(L9963_Handle *handle, L9963_CommTimeoutEnum commTimeout, uint8_t dev_id);

/**
 * @brief Perform the FSMstatus read from the FSM register.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @param[out] status: the readed status
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 */
L9963_Status L9963_GetFsmStatus(L9963_Handle *handle, uint8_t dev_id, L9963_FSM_Status *status);

/**
 * @brief Perform the device software reset.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 * @note Do not use this function
 */
L9963_Status L9963_SoftwareReset(L9963_Handle *handle, uint8_t dev_id);

/**
 * @brief Moves the device's status to the SLEEP status.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 * @note Do not use this function
 */
L9963_Status L9963_GoToSleep(L9963_Handle *handle, uint8_t dev_id);

/**
 * @brief Enables the specified cells.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @param[in] en_cells_mask: 14-bit long binary mask that indicates cells (bit set) to enable
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 */
L9963_Status L9963_SetEnabledCells(L9963_Handle *handle, uint8_t dev_id, uint16_t en_cells_mask);

// L9963_Status L9963_enable_vref(L9963_Handle *handle, uint8_t device, uint8_t preserve_reg_value);

// L9963_Status L9963_start_conversion(L9963_Handle *handle,
// 									uint8_t device,
// 									uint8_t adc_filter_soc,
// 									uint8_t options);

// L9963_Status L9963_poll_conversion(L9963_Handle *handle, uint8_t device, uint8_t *conversion_done);

/**
 * @brief Reads the specified cell voltage binary code.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @param[in] cell: the desidered cell
 * @param[out] vcell: the readed voltage
 * @param[out] data_ready: 0 if data was already read once, 1 otherwise
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 */
L9963_Status L9963_ReadCellVoltage(L9963_Handle *handle, uint8_t dev_id, L9963_CellsEnum cell, uint16_t *vcell, uint8_t *data_ready);

/**
 * @brief Reads the measured battery voltage (via ADC) and the digital sum of all cell's voltages.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @param[out] vbatt_monitor: the readed battery terminal voltage
 * @param[out] vbatt_sum: the digital sum of all cell's voltages
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 */
L9963_Status L9963_ReadBatteryVoltage(L9963_Handle *handle, uint8_t dev_id, uint16_t *vbatt_monitor, uint32_t *vbatt_sum);

/**
 * @brief Reads the voltage across a desidered GPIO.
 * @param[in] handle: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device address
 * @param[in] gpio: gpio to read
 * @param[out] vgpio: the readed gpio voltage
 * @param[out] data_ready: 0 if data was already read once, 1 otherwise
 * @retval L9963_OK if no error occurs, L9963_NOT_OK otherwise
 * @note [Antonio Capone] I'm not sure, but i think that is necessary to set a GPIO as analog input in GPIOx_CONFIG to use this function. It will be tested.
 */
L9963_Status L9963_ReadGPIO(L9963_Handle *handle, uint8_t dev_id, L9963_GpiosEnum gpio, uint16_t *vgpio, uint8_t *data_ready);

#endif /* __L9963_H_ */