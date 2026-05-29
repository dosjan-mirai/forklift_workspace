/*
 * robstride.h
 *
 *  Created on: May 8, 2026
 *      Author: dosjan
 */
#include "main.h"
#ifndef INC_ROBSTRIDE_H_
#define INC_ROBSTRIDE_H_

#define ROBSTRIDE_CAN_HOST_ID          (0xFDU)
#define ROBSTRIDE_CAN_MOTOR_ID         (0x01U)
#define ROBSTRIDE_CTRL_PERIOD_MS       (10U)
#define ROBSTRIDE_PRINT_PERIOD_MS      (200U)

#define ROBSTRIDE_TORQUE_MIN_NM        (-14.0f)
#define ROBSTRIDE_TORQUE_MAX_NM        (14.0f)
#define ROBSTRIDE_PI                   (3.14159265359f)
#define ROBSTRIDE_POS_MIN_RAD          (-4.0f * ROBSTRIDE_PI)
#define ROBSTRIDE_POS_MAX_RAD          (4.0f * ROBSTRIDE_PI)
#define ROBSTRIDE_VEL_MIN_RAD_S        (-33.0f)
#define ROBSTRIDE_VEL_MAX_RAD_S        (33.0f)
#define ROBSTRIDE_KP_MIN               (0.0f)
#define ROBSTRIDE_KP_MAX               (500.0f)
#define ROBSTRIDE_KD_MIN               (0.0f)
#define ROBSTRIDE_KD_MAX               (5.0f)

#define ROBSTRIDE_CMD_MOTION_CTRL      (0x01U)
#define ROBSTRIDE_CMD_MOTOR_FEEDBACK   (0x02U)
#define ROBSTRIDE_CMD_ENABLE           (0x03U)
#define ROBSTRIDE_CMD_STOP             (0x04U)
#define ROBSTRIDE_CMD_PARAM_READ       (0x11U)
#define ROBSTRIDE_CMD_PARAM_WRITE      (0x12U)
#define ROBSTRIDE_CMD_ACTIVE_REPORT    (0x18U)

#define ROBSTRIDE_IDX_RUN_MODE         (0x7005U)
#define ROBSTRIDE_IDX_IQ_REF           (0x7006U)
#define ROBSTRIDE_IDX_SPD_REF          (0x700AU)
#define ROBSTRIDE_IDX_LIMIT_TRQ        (0x700BU)
#define ROBSTRIDE_IDX_LOC_REF          (0x7016U)
#define ROBSTRIDE_IDX_LIMIT_SPD        (0x7017U)
#define ROBSTRIDE_IDX_LIMIT_CUR        (0x7018U)
#define ROBSTRIDE_IDX_VBUS             (0x701CU)
#define ROBSTRIDE_IDX_MECH_POS         (0x7019U)
#define ROBSTRIDE_IDX_IQFBK            (0x701AU)
#define ROBSTRIDE_IDX_ACC_RAD          (0x7022U)
#define ROBSTRIDE_IDX_VEL_MAX          (0x7024U)
#define ROBSTRIDE_IDX_ACC_SET          (0x7025U)
#define ROBSTRIDE_IDX_REPORT_TIME      (0x7026U)

#define ROBSTRIDE_MODE_OPERATION       (0U)
#define ROBSTRIDE_MODE_PP_POS          (1U)
#define ROBSTRIDE_MODE_VELOCITY        (2U)
#define ROBSTRIDE_MODE_CURRENT         (3U)
#define ROBSTRIDE_MODE_CSP_POS         (5U)

#define LIVE_CMD_NONE                  (0U)
#define LIVE_CMD_RUN_ALL_TESTS         (15U)
#define LIVE_CMD_MAX_TORQUE            (16U)

#define ROBSTRIDE_MAX_TORQUE_IQ_A      (12.0f)
#define ROBSTRIDE_MAX_TORQUE_HOLD_MS   (30000U)


 HAL_StatusTypeDef RobStride_SendEnable(uint8_t host_can_id, uint8_t motor_can_id);
 HAL_StatusTypeDef RobStride_SendStop(uint8_t host_can_id, uint8_t motor_can_id, uint8_t clear_fault);
 HAL_StatusTypeDef RobStride_SendOpControl(float torque_nm, float target_pos_rad, float target_vel_rad_s, float kp, float kd);
 HAL_StatusTypeDef RobStride_WriteFloat(uint16_t index, float value);
 HAL_StatusTypeDef RobStride_WriteU8(uint16_t index, uint8_t value);
 HAL_StatusTypeDef RobStride_WriteU16(uint16_t index, uint16_t value);
 HAL_StatusTypeDef RobStride_ReadParam(uint16_t index);
 HAL_StatusTypeDef RobStride_EnableActiveReport(uint16_t report_ms);
 HAL_StatusTypeDef RobStride_DisableActiveReport(void);
 void RobStride_ProcessRx(void);
 void RobStride_ReadAllParams(void);
 void RobStride_HandleLiveCommand(void);
 void RobStride_RunAllModeTests(void);
 HAL_StatusTypeDef RobStride_PrepareMode(uint8_t run_mode);
 void RobStride_RunMaxTorque(void);
 void UartLog(const char *msg);
 void RobStride_LogCanDiag(const char *tag);
 const char *RobStride_CanLecToStr(uint32_t lec);


typedef struct
{
  float pos_rad;
  float vel_rad_s;
  float torque_nm;
  float temp_c;
  float vbus_v;
  float mech_pos_rad;
  float iq_fbk_a;
  uint8_t mode_status;
  uint8_t fault_bits;
  uint8_t run_mode;
  uint8_t type2_valid;
} RobStrideState;




#endif /* INC_ROBSTRIDE_H_ */
