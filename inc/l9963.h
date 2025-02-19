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
L9963_Status L9963_setCommTimeout(L9963_Handle *handle, L9963_CommTimeoutEnum commTimeout, uint8_t dev_id);

// L9963_Status L9963_set_enabled_cells(L9963_Handle *handle, uint8_t device, uint16_t cells);

// L9963_Status L9963_enable_vref(L9963_Handle *handle, uint8_t device, uint8_t preserve_reg_value);

// L9963_Status L9963_start_conversion(L9963_Handle *handle,
// 									uint8_t device,
// 									uint8_t adc_filter_soc,
// 									uint8_t options);

// L9963_Status L9963_poll_conversion(L9963_Handle *handle, uint8_t device, uint8_t *conversion_done);

// L9963_Status L9963_read_cell_voltage(L9963_Handle *handle,
// 									uint8_t device,
// 									L9963_CellsEnum cell,
// 									uint16_t *vcell,
// 									uint8_t *data_ready);

// L9963_Status L9963_read_batt_voltage(L9963_Handle *handle,
// 									uint8_t device,
// 									uint16_t *vbatt_monitor,
// 									uint32_t *vbatt_sum);

// L9963_Status L9963_read_gpio_voltage(L9963_Handle *handle,
// 									uint8_t device,
// 									L9963_GpiosEnum gpio,
// 									uint16_t *vgpio,
// 									uint8_t *data_ready);

#endif /* __L9963_H_ */