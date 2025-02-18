/**
 * @file l9963.h
 * @brief L9963E low level driver header
 *
 * @author Antonio Capone
 * @date 2025-02-11
 * @version 1.0
 */

#ifndef __L9963_DRIVER_H_
#define __L9963_DRIVER_H_

#include "l9963_registers.h"
#include "l9963_status.h"
#include "l9963_platform.h"

#define L9963_Driver_BURST_0x78_LEN 18
#define L9963_Driver_BURST_0x7A_LEN 13
#define L9963_Driver_BURST_0x7B_LEN 14

typedef struct {
	L9963_Platform interface;
} L9963_Driver_Handle;

typedef union {
	uint64_t val;
	struct {
		uint64_t crc : 6, data : 18, gsw : 2, addr : 7, devid : 5, rw_burst : 1, pa : 1, : 24;
	} cmd;
} L9963_Driver_FrameUnion;

typedef enum {
	_0x78BurstCmd = 0x78,
	_0x7ABurstCmd = 0x7A,
	_0x7BBurstCmd = 0x7B
} L9963_Driver_BurstCmd;

typedef struct {
	struct {
		uint32_t VCell : 16, d_rdy_Vcell : 1, VCELL_EN : 1, : 14;
	} Frame1_14[14];
	struct {
		uint32_t vsum_batt19_2 : 18, : 14;
	} Frame15;
	struct {
		uint32_t VBATT_DIV : 16, vsum_batt1_0 : 2, : 14;
	} Frame16;
	struct {
		uint32_t eof_bal : 1, bal_on : 1, TimedBalTimer : 7, TimedBalacc : 1, VSUM_UV : 1,
			VSUM_OV : 1, DUTY_ON : 1, CONF_CYCLIC_EN : 1, OVR_LATCH : 1, SOC : 1,
			data_ready_vbattdiv : 1, data_ready_vsum : 1, : 14;
	} Frame17;
	struct {
		uint32_t CUR_INST_calib : 18, : 14;
	} Frame18;
} L9963_Driver_0x78Burst;

typedef struct {
	struct {
		uint32_t wu_cyc_wup : 1, wu_faulth : 1, wu_isoline : 1, wu_spi : 1, wu_gpio7 : 1,
			VCOM_UV : 1, VCOM_OV : 1, VREG_UV : 1, VTREF_OV : 1, VTREF_UV : 1, VDIG_OV : 1,
			VANA_OV : 1, Otchip : 1, sense_minus_open : 1, sense_plus_open : 1, TCYCLE_OVF : 1,
			VREG_OV : 1, OVR_LATCH : 1, : 14;
	} Frame1;
	struct {
		uint32_t GPIO6_OPEN : 1, GPIO7_OPEN : 1, GPIO8_OPEN : 1, GPIO9_OPEN : 1,
			GPIO3_fastchg_OT : 1, GPIO4_fastchg_OT : 1, GPIO5_fastchg_OT : 1, GPIO6_fastchg_OT : 1,
			GPIO7_fastchg_OT : 1, GPIO8_fastchg_OT : 1, GPIO9_fastchg_OT : 1, EoBtimeerror : 1,
			CoCouOvF : 1, TrimmCalOk : 1, loss_gndref : 1, loss_cgnd : 1, loss_dgnd : 1,
			loss_agnd : 1, : 14;
	} Frame2;
	struct {
		uint32_t BAL1_OPEN : 1, BAL2_OPEN : 1, BAL3_OPEN : 1, BAL4_OPEN : 1, BAL5_OPEN : 1,
			BAL6_OPEN : 1, BAL7_OPEN : 1, BAL8_OPEN : 1, BAL9_OPEN : 1, BAL10_OPEN : 1,
			BAL11_OPEN : 1, BAL12_OPEN : 1, BAL13_OPEN : 1, BAL14_OPEN : 1, EEPROM_DWNLD_DONE : 1,
			GPIO3_OPEN : 1, GPIO4_OPEN : 1, GPIO5_OPEN : 1, : 14;
	} Frame3;
	struct {
		uint32_t BAL1_SHORT : 1, BAL2_SHORT : 1, BAL3_SHORT : 1, BAL4_SHORT : 1, BAL5_SHORT : 1,
			BAL6_SHORT : 1, BAL7_SHORT : 1, BAL8_SHORT : 1, BAL9_SHORT : 1, BAL10_SHORT : 1,
			BAL11_SHORT : 1, BAL12_SHORT : 1, BAL13_SHORT : 1, BAL14_SHORT : 1,
			VTREF_COMP_BIST_FAIL : 1, VCOM_COMP_BIST_FAIL : 1, VREG_COMP_BIST_FAIL : 1,
			VBAT_COMP_BIST_FAIL : 1, : 14;
	} Frame4;
	struct {
		uint32_t CELL0_OPEN : 1, CELL1_OPEN : 1, CELL2_OPEN : 1, CELL3_OPEN : 1, CELL4_OPEN : 1,
			CELL5_OPEN : 1, CELL6_OPEN : 1, CELL7_OPEN : 1, CELL8_OPEN : 1, CELL9_OPEN : 1,
			CELL10_OPEN : 1, CELL11_OPEN : 1, CELL12_OPEN : 1, CELL13_OPEN : 1, CELL14_OPEN : 1,
			VBAT_OPEN : 1, HWSC_DONE : 1, EEPROM_CRC_ERR_CAL_FF : 1, : 14;
	} Frame5;
	struct {
		uint32_t VCELL1_UV : 1, VCELL2_UV : 1, VCELL3_UV : 1, VCELL4_UV : 1, VCELL5_UV : 1,
			VCELL6_UV : 1, VCELL7_UV : 1, VCELL8_UV : 1, VCELL9_UV : 1, VCELL10_UV : 1,
			VCELL11_UV : 1, VCELL12_UV : 1, VCELL13_UV : 1, VCELL14_UV : 1, RAM_CRC_ERR : 1,
			EEPROM_CRC_ERR_CAL_RAM : 1, Comm_timeout_flt : 1, EEPROM_CRC_ERR_SECT_0 : 1, : 14;
	} Frame6;
	struct {
		uint32_t VCELL1_OV : 1, VCELL2_OV : 1, VCELL3_OV : 1, VCELL4_OV : 1, VCELL5_OV : 1,
			VCELL6_OV : 1, VCELL7_OV : 1, VCELL8_OV : 1, VCELL9_OV : 1, VCELL10_OV : 1,
			VCELL11_OV : 1, VCELL12_OV : 1, VCELL13_OV : 1, VCELL14_OV : 1, VSUM_UV : 1,
			VBATTCRIT_UV : 1, VBATT_WRN_UV : 1, VBATT_WRN_OV : 1, : 14;
	} Frame7;
	struct {
		uint32_t GPIO3_UT : 1, GPIO4_UT : 1, GPIO5_UT : 1, GPIO6_UT : 1, GPIO7_UT : 1, GPIO8_UT : 1,
			GPIO9_UT : 1, GPIO3_OT : 1, GPIO4_OT : 1, GPIO5_OT : 1, GPIO6_OT : 1, GPIO7_OT : 1,
			GPIO8_OT : 1, GPIO9_OT : 1, VSUM_OV : 1, VBATTCRIT_OV : 1, eof_bal : 1,
			bal_on : 1, : 14;
	} Frame8;
	struct {
		uint32_t VCELL1_BAL_UV : 1, VCELL2_BAL_UV : 1, VCELL3_BAL_UV : 1, VCELL4_BAL_UV : 1,
			VCELL5_BAL_UV : 1, VCELL6_BAL_UV : 1, VCELL7_BAL_UV : 1, VCELL8_BAL_UV : 1,
			VCELL9_BAL_UV : 1, VCELL10_BAL_UV : 1, VCELL11_BAL_UV : 1, VCELL12_BAL_UV : 1,
			VCELL13_BAL_UV : 1, VCELL14_BAL_UV : 1, GPO3on : 1, GPO4on : 1, GPO5on : 1,
			GPO6on : 1, : 14;
	} Frame9;
	struct {
		uint32_t GPIO_BIST_FAIL : 7, GPO3short : 1, GPO4short : 1, GPO5short : 1, GPO6short : 1,
			GPO7short : 1, GPO8short : 1, GPO9short : 1, GPO7on : 1, GPO8on : 1, GPO9on : 1,
			Fault_L_line_status : 1, : 14;
	} Frame10;
	struct {
		uint32_t MUX_BIST_FAIL : 14, HeartBeat_En : 1, FaultH_EN : 1, FaultHline_fault : 1,
			HeartBeat_fault : 1, : 14;
	} Frame11;
	struct {
		uint32_t BIST_BAL_COMP_LS_FAIL : 7, BIST_BAL_COMP_HS_FAIL : 7, HeartBeatCycle : 3,
			curr_sense_ovc_sleep : 1, : 14;
	} Frame12;
	struct {
		uint32_t OPEN_BIST_FAIL : 14, clk_mon_init_done : 1, clk_mon_en : 1, OSCFail : 1,
			curr_sense_ovc_norm : 1, : 14;
	} Frame13;
} L9963_Driver_0x7ABurst;

typedef struct {
	struct {
		uint32_t CoulombCntTime : 16, CoCouOvF : 1, CoulombCounter_en : 1, : 14;
	} Frame1;
	struct {
		uint32_t CoulombCounter_msb : 16, sense_minus_open : 1, sense_plus_open : 1, : 14;
	} Frame2;
	struct {
		uint32_t CoulombCounter_lsb : 16, curr_sense_ovc_norm : 1, curr_sense_ovc_sleep : 1, : 14;
	} Frame3;
	struct {
		uint32_t CUR_INST_synch : 18, : 14;
	} Frame4;
	struct {
		uint32_t CUR_INST_calib : 18, : 14;
	} Frame5;
	struct {
		uint32_t GPIO_MEAS : 16, d_rdy_gpio : 1, GPIO_OT : 1, : 14;
	} Frame6_12[7];
	struct {
		uint32_t VTREF_MEAS : 16, d_rdy_vtref : 1, TrimmCalOk : 1, : 14;
	} Frame13;
	struct {
		uint32_t TempChip : 8, GPIO3_UT : 1, GPIO4_UT : 1, GPIO5_UT : 1, GPIO6_UT : 1, GPIO7_UT : 1,
			GPIO8_UT : 1, GPIO9_UT : 1, bal_on : 1, eof_bal : 1, OTchip : 1, : 14;
	} Frame14;
} L9963_Driver_0x7BBurst;

typedef union {
	uint32_t generics[L9963_Driver_BURST_0x78_LEN];
	L9963_Driver_0x78Burst _0x78;
	L9963_Driver_0x7ABurst _0x7A;
	L9963_Driver_0x7BBurst _0x7B;
} L9963_Driver_BurstUnion;

/**
 * @brief L9963 driver initializer
 * @param[in] dev: pointer to empty handler structure
 * @param[in] interface: platform dependent functions handler
 */
void L9963_Driver_Init(L9963_Driver_Handle *dev, L9963_Platform interface);

/**
 * @brief Enables SPI chip select pin
 * @param[in] dev: pointer to empty handler structure
 */
void L9963_Driver_Select(L9963_Driver_Handle *dev);

/**
 * @brief Disables SPI chip select pin
 * @param[in] dev: pointer to empty handler structure
 */
void L9963_Driver_Deselect(L9963_Driver_Handle *dev);

/**
 * @brief Platform dependent SPI trasmit wrapper
 * @param[in] dev: pointer to device handler structure
 * @param[in] data: pointer to data to be sent
 * @param[in] len: payload length
 * @param[in] timeout: trasmission timeout
 * @retval L9963_Driver_OK if no error occurs, L9963_Driver_NOT_OK otherwise
 */
L9963_Status L9963_Driver_SPI_Transmit(L9963_Driver_Handle *dev, uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief Platform dependent SPI trasmit wrapper
 * @param[in] dev: pointer to device handler structure
 * @param[out] data: pointer to read buffer
 * @param[in] len: payload length
 * @param[in] timeout: transmission timeout (ms)
 * @retval L9963_Driver_OK if no error occurs, L9963_Driver_NOT_OK otherwise
 */
L9963_Status L9963_Driver_SPI_Receive(L9963_Driver_Handle *dev, uint8_t *data, uint16_t len, uint32_t timeout);

/**
 * @brief Perform a write or a read request in "single access mode" (see datasheet)
 * @param[in] dev: pointer to device handler structure
 * @param[in] rw: 0 for read and 1 for write commands
 * @param[in] dev_id: 5-bit long device id
 * @param[in] reg_addr: register address
 * @param[in, out] data: pointer to data to be written if rw == 1, otherwise read buffer pointer
 * @param[in] timeout: transmission timeout (ms)
 */
static L9963_Status L9963_Driver_RegisterCmd(L9963_Driver_Handle *dev, uint8_t rw, uint8_t dev_id, L9963_RegistersAddr reg_addr, L9963_RegisterUnion *data, uint8_t timeout);

/**
 * @brief Perform a read request in "single access mode" (see datasheet)
 * @param[in] dev: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device id
 * @param[in] reg_addr: register address
 * @param[in, out] data: read buffer pointer
 * @param[in] timeout: transmission timeout (ms)
 */
L9963_Status L9963_Driver_RegisterRead(L9963_Driver_Handle *dev, uint8_t dev_id, L9963_RegistersAddr reg_addr, L9963_RegisterUnion *data, uint8_t timeout);

/**
 * @brief Perform a write request in "single access mode" (see datasheet)
 * @param[in] dev: pointer to device handler structure
 * @param[in] dev_id: 5-bit long device id
 * @param[in] reg_addr: register address
 * @param[in, out] data: pointer to data to be written
 * @param[in] timeout: transmission timeout (ms)
 */
L9963_Status L9963_Driver_RegisterWrite(L9963_Driver_Handle *dev, uint8_t dev_id, L9963_RegistersAddr reg_addr, L9963_RegisterUnion *data, uint8_t timeout);

/**
 * @brief Perform a synchronous SPI receive and automatically checks for CRC errors
 * @param[in] dev: pointer to device handler structure
 * @param[out] frame: pointer to received frame buffer
 * @param[in] dev_id: 5-bit long device id
 * @param[in] timeout: transmission timeout (ms)
 */
L9963_Status L9963_Driver_WaitReceive(L9963_Driver_Handle *dev, L9963_Driver_FrameUnion *frame, uint8_t dev_id, uint8_t timeout);

/**
 * @brief Sends a burst cmd to the ic and stores the response into a buffer
 * @param[in] dev: pointer to device handler
 * @param[in] dev_id: 5-bit long device id
 * @param[in] command: burst cmd type to send
 * @param[out] data: pointer to rx buffer, the dimension must me at least expected_frames_n * sizeof(your_burst_cmd)
 * @param[in] expected_frames_n: number of rx expected frames, this parameter is command-dependent (see datasheet)
 * @param[in] timeout: transmission timeout (ms)
 * @retval L9963_Driver_OK if no error occurs, L9963_Driver_NOT_OK otherwise
 */
L9963_Status L9963_Driver_SendBurst(L9963_Driver_Handle* dev, uint8_t dev_id, L9963_Driver_BurstCmd command, L9963_Driver_BurstUnion* data, uint8_t expected_frames_n, uint8_t timeout);

/**
 * @brief Converts a 40-bit frame endianess (from Big-Endian to Little-Endian and vice versa)
 * @param[in] in: pointer to frame to be converted
 * @param[out] out: pointer to the destination frame
 * @note The function assumes that both input and output buffers are at least 40-bit long.
 */
void L9963_Driver_FrameSwitchEndianess(uint8_t *in, uint8_t *out);

/**
 * @brief Builds a L9963 SPI valid frame from frame fields
 * @param[out] out: pointer to output frame buffer
 * @param[in] pa: 0 for slave responses and always 1 for master's commands
 * @param[in] rw: 0 for read and 1 for write commands
 * @param[in] devid: 5-bit long device id
 * @param[in] addr_command: 7-bit register address
 * @param[in] data: 18-bit data payload
 */
void L9963_Driver_BuildFrame(uint8_t *out, uint8_t pa, uint8_t rw, uint8_t devid, uint8_t addr_command, uint32_t data);

/**
 * @brief Computes the CRC calculation of the SPI frame
 * @param[in] InputWord: 40-bit frame (CRC 6-bit are assumed to be 0)
 * @retval CRC frame value
 */
static uint8_t L9963_Driver_ComputeCRC(uint64_t InputWord);

/**
 * @param[in] dev: pointer to device handler
 * @retval System current tick (ms)
 */
uint32_t L9963_Driver_GetTick(L9963_Driver_Handle *dev);

#endif /* __L9963_DRIVER_H_ */
