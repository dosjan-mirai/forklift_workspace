///*
// * robstride.c
// *
// *  Created on: May 8, 2026
// *      Author: dosjan
// */
//
//#include "robstride.h"
//#include"main.h"
//
//
//
//extern UART_HandleTypeDef huart3;
//extern  RobStrideState g_state;
//
// void UartLog(const char *msg)
//{
//  (void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)strlen(msg), 100U);
//}
//
// const char *RobStride_CanLecToStr(uint32_t lec)
//{
//  switch (lec)
//  {
//    case 0U: return "NO_ERR";
//    case 1U: return "STUFF_ERR";
//    case 2U: return "FORM_ERR";
//    case 3U: return "ACK_ERR";
//    case 4U: return "BIT1_ERR";
//    case 5U: return "BIT0_ERR";
//    case 6U: return "CRC_ERR";
//    case 7U: return "SW_SET";
//    default: return "UNKNOWN";
//  }
//}
//
// void RobStride_LogCanDiag(const char *tag)
//{
//  char msg[180];
//  uint32_t esr = CAN1->ESR;
//  uint32_t msr = CAN1->MSR;
//  uint32_t tsr = CAN1->TSR;
//  uint32_t lec = (esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
//  uint32_t tec = (esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
//  uint32_t rec = (esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;
//  int len = snprintf(msg, sizeof(msg),
//                     "[CAN] %s LEC=%lu(%s) TEC=%lu REC=%lu BOFF=%lu EPVF=%lu EWGF=%lu INAK=%lu txok0=%lu txok1=%lu txok2=%lu\r\n",
//                     tag,
//                     (unsigned long)lec, RobStride_CanLecToStr(lec),
//                     (unsigned long)tec, (unsigned long)rec,
//                     (unsigned long)((esr & CAN_ESR_BOFF) ? 1U : 0U),
//                     (unsigned long)((esr & CAN_ESR_EPVF) ? 1U : 0U),
//                     (unsigned long)((esr & CAN_ESR_EWGF) ? 1U : 0U),
//                     (unsigned long)((msr & CAN_MSR_INAK) ? 1U : 0U),
//                     (unsigned long)((tsr & CAN_TSR_TXOK0) ? 1U : 0U),
//                     (unsigned long)((tsr & CAN_TSR_TXOK1) ? 1U : 0U),
//                     (unsigned long)((tsr & CAN_TSR_TXOK2) ? 1U : 0U));
//  if (len > 0)
//  {
//    (void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)len, 120U);
//  }
//}
//
// float clampf(float x, float xmin, float xmax)
//{
//  if (x < xmin)
//  {
//    return xmin;
//  }
//  if (x > xmax)
//  {
//    return xmax;
//  }
//  return x;
//}
//
// uint16_t float_to_u16(float x, float x_min, float x_max)
//{
//  const float clamped = clampf(x, x_min, x_max);
//  const float norm = (clamped - x_min) / (x_max - x_min);
//  return (uint16_t)(norm * 65535.0f);
//}
//
// float u16_to_float(uint16_t x, float x_min, float x_max)
//{
//  return (((float)x) * (x_max - x_min) / 65535.0f) + x_min;
//}
//
// uint32_t robstride_build_ext_id(uint8_t comm_type, uint16_t data_area2, uint8_t data_area1)
//{
//  return ((((uint32_t)comm_type) & 0x1FU) << 24U) |
//         ((((uint32_t)data_area2) & 0xFFFFU) << 8U) |
//         (((uint32_t)data_area1) & 0xFFU);
//}
//
// HAL_StatusTypeDef can1_send_ext(uint32_t ext_id, const uint8_t data[8])
//{
//  uint32_t mailbox = 0U;
//  uint32_t rqcp = 0U;
//  uint32_t txok = 0U;
//  const uint32_t tickstart = HAL_GetTick();
//
//  while ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0U)
//  {
//    if ((HAL_GetTick() - tickstart) > 10U)
//    {
//      RobStride_LogCanDiag("MAILBOX_TIMEOUT");
//      return HAL_TIMEOUT;
//    }
//  }
//
//  if ((CAN1->TSR & CAN_TSR_TME0) != 0U)
//  {
//    mailbox = 0U;
//    rqcp = CAN_TSR_RQCP0;
//    txok = CAN_TSR_TXOK0;
//  }
//  else if ((CAN1->TSR & CAN_TSR_TME1) != 0U)
//  {
//    mailbox = 1U;
//    rqcp = CAN_TSR_RQCP1;
//    txok = CAN_TSR_TXOK1;
//  }
//  else
//  {
//    mailbox = 2U;
//    rqcp = CAN_TSR_RQCP2;
//    txok = CAN_TSR_TXOK2;
//  }
//
//  CAN1->sTxMailBox[mailbox].TDTR = 8U;
//  CAN1->sTxMailBox[mailbox].TDLR = ((uint32_t)data[3] << 24U) |
//                                   ((uint32_t)data[2] << 16U) |
//                                   ((uint32_t)data[1] << 8U) |
//                                   ((uint32_t)data[0]);
//  CAN1->sTxMailBox[mailbox].TDHR = ((uint32_t)data[7] << 24U) |
//                                   ((uint32_t)data[6] << 16U) |
//                                   ((uint32_t)data[5] << 8U) |
//                                   ((uint32_t)data[4]);
//  CAN1->sTxMailBox[mailbox].TIR = (ext_id << 3U) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;
//
//  while ((CAN1->TSR & rqcp) == 0U)
//  {
//    if ((HAL_GetTick() - tickstart) > 10U)
//    {
//      RobStride_LogCanDiag("TXRQ_TIMEOUT");
//      return HAL_TIMEOUT;
//    }
//  }
//
//  {
//    const uint32_t tsr = CAN1->TSR;
//    CAN1->TSR = rqcp;
//    if ((tsr & txok) == 0U)
//    {
//      RobStride_LogCanDiag("TX_NOACK_OR_FAIL");
//      return HAL_ERROR;
//    }
//  }
//
//  return HAL_OK;
//}
//
// HAL_StatusTypeDef RobStride_SendEnable(uint8_t host_can_id, uint8_t motor_can_id)
//{
//  const uint16_t target = (((uint16_t)host_can_id) << 8U) | motor_can_id;
//  const uint8_t data[8] = {0};
//  return can1_send_ext(robstride_build_ext_id(0x03U, target, motor_can_id), data);
//}
//
// HAL_StatusTypeDef RobStride_SendStop(uint8_t host_can_id, uint8_t motor_can_id, uint8_t clear_fault)
//{
//  const uint16_t target = (((uint16_t)host_can_id) << 8U) | motor_can_id;
//  uint8_t data[8] = {0};
//  data[0] = clear_fault ? 1U : 0U;
//  return can1_send_ext(robstride_build_ext_id(0x04U, target, motor_can_id), data);
//}
//
// HAL_StatusTypeDef RobStride_SendOpControl(float torque_nm, float target_pos_rad, float target_vel_rad_s, float kp, float kd)
//{
//  const uint16_t torque_u16 = float_to_u16(torque_nm, ROBSTRIDE_TORQUE_MIN_NM, ROBSTRIDE_TORQUE_MAX_NM);
//  const uint16_t pos_u16 = float_to_u16(target_pos_rad, ROBSTRIDE_POS_MIN_RAD, ROBSTRIDE_POS_MAX_RAD);
//  const uint16_t vel_u16 = float_to_u16(target_vel_rad_s, ROBSTRIDE_VEL_MIN_RAD_S, ROBSTRIDE_VEL_MAX_RAD_S);
//  const uint16_t kp_u16 = float_to_u16(kp, ROBSTRIDE_KP_MIN, ROBSTRIDE_KP_MAX);
//  const uint16_t kd_u16 = float_to_u16(kd, ROBSTRIDE_KD_MIN, ROBSTRIDE_KD_MAX);
//  const uint8_t data[8] = {
//      (uint8_t)(pos_u16 >> 8U), (uint8_t)(pos_u16 & 0xFFU),
//      (uint8_t)(vel_u16 >> 8U), (uint8_t)(vel_u16 & 0xFFU),
//      (uint8_t)(kp_u16 >> 8U), (uint8_t)(kp_u16 & 0xFFU),
//      (uint8_t)(kd_u16 >> 8U), (uint8_t)(kd_u16 & 0xFFU)};
//
//  /* data_area1 must be target motor id for reliable operation mode control. */
//  return can1_send_ext(robstride_build_ext_id(0x01U, torque_u16, ROBSTRIDE_CAN_MOTOR_ID), data);
//}
//
// HAL_StatusTypeDef can1_recv_ext(uint32_t *ext_id, uint8_t data[8], uint8_t *dlc)
//{
//  uint32_t rir;
//  uint32_t rdtr;
//  uint32_t rdlr;
//  uint32_t rdhr;
//
//  if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0U)
//  {
//    return HAL_ERROR;
//  }
//
//  rir = CAN1->sFIFOMailBox[0].RIR;
//  rdtr = CAN1->sFIFOMailBox[0].RDTR;
//  rdlr = CAN1->sFIFOMailBox[0].RDLR;
//  rdhr = CAN1->sFIFOMailBox[0].RDHR;
//  CAN1->RF0R |= CAN_RF0R_RFOM0;
//
//  if ((rir & CAN_RI0R_IDE) == 0U)
//  {
//    return HAL_ERROR;
//  }
//
//  *ext_id = (rir >> 3U) & 0x1FFFFFFFU;
//  *dlc = (uint8_t)(rdtr & 0x0FU);
//  data[0] = (uint8_t)(rdlr);
//  data[1] = (uint8_t)(rdlr >> 8U);
//  data[2] = (uint8_t)(rdlr >> 16U);
//  data[3] = (uint8_t)(rdlr >> 24U);
//  data[4] = (uint8_t)(rdhr);
//  data[5] = (uint8_t)(rdhr >> 8U);
//  data[6] = (uint8_t)(rdhr >> 16U);
//  data[7] = (uint8_t)(rdhr >> 24U);
//  return HAL_OK;
//}
//
// HAL_StatusTypeDef RobStride_WriteFloat(uint16_t index, float value)
//{
//  uint8_t data[8] = {0};
//  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_WRITE,
//                                           ((uint16_t)ROBSTRIDE_CAN_HOST_ID) << 8U,
//                                           ROBSTRIDE_CAN_MOTOR_ID);
//  data[0] = (uint8_t)(index & 0xFFU);
//  data[1] = (uint8_t)(index >> 8U);
//  memcpy(&data[4], &value, sizeof(float));
//  return can1_send_ext(ext_id, data);
//}
//
// HAL_StatusTypeDef RobStride_WriteU8(uint16_t index, uint8_t value)
//{
//  uint8_t data[8] = {0};
//  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_WRITE,
//                                           ((uint16_t)ROBSTRIDE_CAN_HOST_ID) << 8U,
//                                           ROBSTRIDE_CAN_MOTOR_ID);
//  data[0] = (uint8_t)(index & 0xFFU);
//  data[1] = (uint8_t)(index >> 8U);
//  data[4] = value;
//  return can1_send_ext(ext_id, data);
//}
//
// HAL_StatusTypeDef RobStride_WriteU16(uint16_t index, uint16_t value)
//{
//  uint8_t data[8] = {0};
//  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_WRITE,
//                                           ((uint16_t)ROBSTRIDE_CAN_HOST_ID) << 8U,
//                                           ROBSTRIDE_CAN_MOTOR_ID);
//  data[0] = (uint8_t)(index & 0xFFU);
//  data[1] = (uint8_t)(index >> 8U);
//  data[4] = (uint8_t)(value & 0xFFU);
//  data[5] = (uint8_t)(value >> 8U);
//  return can1_send_ext(ext_id, data);
//}
//
// HAL_StatusTypeDef RobStride_ReadParam(uint16_t index)
//{
//  uint8_t data[8] = {0};
//  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_READ,
//                                           ((uint16_t)ROBSTRIDE_CAN_HOST_ID) << 8U,
//                                           ROBSTRIDE_CAN_MOTOR_ID);
//  data[0] = (uint8_t)(index & 0xFFU);
//  data[1] = (uint8_t)(index >> 8U);
//  return can1_send_ext(ext_id, data);
//}
//
// HAL_StatusTypeDef RobStride_EnableActiveReport(uint16_t report_ms)
//{
//  uint8_t data[8] = {1U, 2U, 3U, 4U, 5U, 6U, 1U, 0U};
//  uint16_t ep = (report_ms <= 10U) ? 1U : (uint16_t)(((report_ms - 10U) / 5U) + 1U);
//  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_ACTIVE_REPORT,
//                                           ((uint16_t)ROBSTRIDE_CAN_HOST_ID) << 8U,
//                                           ROBSTRIDE_CAN_MOTOR_ID);
//
//  if (can1_send_ext(ext_id, data) != HAL_OK)
//  {
//    return HAL_ERROR;
//  }
//  HAL_Delay(5);
//  return RobStride_WriteU16(ROBSTRIDE_IDX_REPORT_TIME, ep);
//}
//
// HAL_StatusTypeDef RobStride_DisableActiveReport(void)
//{
//  uint8_t data[8] = {1U, 2U, 3U, 4U, 5U, 6U, 0U, 0U};
//  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_ACTIVE_REPORT,
//                                           ((uint16_t)ROBSTRIDE_CAN_HOST_ID) << 8U,
//                                           ROBSTRIDE_CAN_MOTOR_ID);
//  return can1_send_ext(ext_id, data);
//}
//
// void RobStride_ProcessRx(void)
//{
//  uint8_t data[8];
//  uint8_t dlc;
//  uint8_t ct;
//  uint32_t ext_id;
//  uint16_t raw;
//  float fv;
//  int i;
//
//  for (i = 0; i < 16; i++)
//  {
//    if (can1_recv_ext(&ext_id, data, &dlc) != HAL_OK)
//    {
//      break;
//    }
//    if (dlc < 8U)
//    {
//      continue;
//    }
//
//    ct = (uint8_t)((ext_id >> 24U) & 0x1FU);
//    if ((ct == ROBSTRIDE_CMD_MOTOR_FEEDBACK) || (ct == ROBSTRIDE_CMD_ACTIVE_REPORT))
//    {
//      raw = ((uint16_t)data[0] << 8U) | data[1];
//      g_state.pos_rad = u16_to_float(raw, ROBSTRIDE_POS_MIN_RAD, ROBSTRIDE_POS_MAX_RAD);
//      raw = ((uint16_t)data[2] << 8U) | data[3];
//      g_state.vel_rad_s = u16_to_float(raw, ROBSTRIDE_VEL_MIN_RAD_S, ROBSTRIDE_VEL_MAX_RAD_S);
//      raw = ((uint16_t)data[4] << 8U) | data[5];
//      g_state.torque_nm = u16_to_float(raw, ROBSTRIDE_TORQUE_MIN_NM, ROBSTRIDE_TORQUE_MAX_NM);
//      raw = ((uint16_t)data[6] << 8U) | data[7];
//      g_state.temp_c = ((float)raw) / 10.0f;
//      g_state.mode_status = (uint8_t)((ext_id >> 22U) & 0x03U);
//      g_state.fault_bits = (uint8_t)((ext_id >> 16U) & 0xFFU);
//      g_state.type2_valid = 1U;
//    }
//    else if (ct == ROBSTRIDE_CMD_PARAM_READ)
//    {
//      uint16_t index = ((uint16_t)data[1] << 8U) | data[0];
//      memcpy(&fv, &data[4], sizeof(float));
//      switch (index)
//      {
//        case ROBSTRIDE_IDX_VBUS:
//          g_state.vbus_v = fv;
//          break;
//        case ROBSTRIDE_IDX_MECH_POS:
//          g_state.mech_pos_rad = fv;
//          break;
//        case ROBSTRIDE_IDX_IQFBK:
//          g_state.iq_fbk_a = fv;
//          break;
//        case ROBSTRIDE_IDX_RUN_MODE:
//          g_state.run_mode = data[4];
//          break;
//        default:
//          break;
//      }
//    }
//  }
//}
//
// void RobStride_ReadAllParams(void)
//{
//  (void)RobStride_ReadParam(ROBSTRIDE_IDX_MECH_POS);
//  HAL_Delay(8);
//  (void)RobStride_ReadParam(ROBSTRIDE_IDX_VBUS);
//  HAL_Delay(8);
//  (void)RobStride_ReadParam(ROBSTRIDE_IDX_IQFBK);
//  HAL_Delay(8);
//  (void)RobStride_ReadParam(ROBSTRIDE_IDX_RUN_MODE);
//}
//
//
//
// HAL_StatusTypeDef RobStride_PrepareMode(uint8_t run_mode)
//{
//  uint32_t t0 = HAL_GetTick();
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  HAL_Delay(40);
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 1U);
//  HAL_Delay(30);
//
//  if (RobStride_WriteU8(ROBSTRIDE_IDX_RUN_MODE, run_mode) != HAL_OK)
//  {
//    return HAL_ERROR;
//  }
//  HAL_Delay(25);
//
//  /* Verify run_mode feedback for up to 250ms. */
//  while ((HAL_GetTick() - t0) < 250U)
//  {
//    (void)RobStride_ReadParam(ROBSTRIDE_IDX_RUN_MODE);
//    HAL_Delay(8);
//    RobStride_ProcessRx();
//    if (g_state.run_mode == run_mode)
//    {
//      break;
//    }
//  }
//
//  if (RobStride_SendEnable(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID) != HAL_OK)
//  {
//    HAL_Delay(20);
//    if (RobStride_SendEnable(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID) != HAL_OK)
//    {
//      return HAL_ERROR;
//    }
//  }
//  HAL_Delay(80);
//  return HAL_OK;
//}
//
// void RobStride_RunMaxTorque(void)
//{
//  uint32_t t0;
//  char msg[96];
//  UartLog("[TEST] max_torque begin\r\n");
//
//  if (RobStride_PrepareMode(ROBSTRIDE_MODE_CURRENT) != HAL_OK)
//  {
//    UartLog("[TEST] max_torque prep fail\r\n");
//    return;
//  }
//
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LIMIT_CUR, ROBSTRIDE_MAX_TORQUE_IQ_A);
//  HAL_Delay(10);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LIMIT_TRQ, 14.0f);
//  HAL_Delay(10);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_IQ_REF, ROBSTRIDE_MAX_TORQUE_IQ_A);
//  (void)snprintf(msg, sizeof(msg), "[TEST] max_torque iq=%.1fA hold=%lums\r\n",
//                 (double)ROBSTRIDE_MAX_TORQUE_IQ_A,
//                 (unsigned long)ROBSTRIDE_MAX_TORQUE_HOLD_MS);
//  UartLog(msg);
//
//  t0 = HAL_GetTick();
//  while ((HAL_GetTick() - t0) < ROBSTRIDE_MAX_TORQUE_HOLD_MS)
//  {
//    RobStride_ProcessRx();
//    HAL_Delay(2);
//  }
//
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_IQ_REF, 0.0f);
//  HAL_Delay(20);
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  UartLog("[TEST] max_torque done\r\n");
//}
//
// void RobStride_RunAllModeTests(void)
//{
//  uint32_t i;
//  const float csp_targets[3] = {0.0f, 0.9f, -0.9f};
//  const float pp_targets[3] = {0.0f, 0.8f, -0.8f};
//  char msg[64];
//
//  UartLog("[TEST] all_modes begin\r\n");
//
//  /* Operation (MIT) mode */
//  if (RobStride_PrepareMode(ROBSTRIDE_MODE_OPERATION) != HAL_OK) UartLog("[TEST] op prep fail\r\n");
//  (void)snprintf(msg, sizeof(msg), "[TEST] op run=%u\r\n", g_state.run_mode);
//  UartLog(msg);
//  UartLog("[TEST] operation\r\n");
//  for (i = 0U; i < 50U; i++)
//  {
//    (void)RobStride_SendOpControl(0.0f, 0.0f, 1.5f, 0.0f, 1.5f);
//    HAL_Delay(20);
//  }
//  for (i = 0U; i < 70U; i++)
//  {
//    (void)RobStride_SendOpControl(0.0f, 1.0f, 0.0f, 20.0f, 1.8f);
//    HAL_Delay(20);
//  }
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  HAL_Delay(100);
//
//  /* Current mode */
//  if (RobStride_PrepareMode(ROBSTRIDE_MODE_CURRENT) != HAL_OK) UartLog("[TEST] cur prep fail\r\n");
//  (void)snprintf(msg, sizeof(msg), "[TEST] cur run=%u\r\n", g_state.run_mode);
//  UartLog(msg);
//  UartLog("[TEST] current\r\n");
//  for (i = 0U; i <= 12U; i++)
//  {
//    (void)RobStride_WriteFloat(ROBSTRIDE_IDX_IQ_REF, ((float)i) * 0.15f);
//    HAL_Delay(100);
//  }
//  for (i = 12U; i > 0U; i--)
//  {
//    (void)RobStride_WriteFloat(ROBSTRIDE_IDX_IQ_REF, ((float)(i - 1U)) * 0.15f);
//    HAL_Delay(100);
//  }
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  HAL_Delay(100);
//
//  /* Velocity mode */
//  if (RobStride_PrepareMode(ROBSTRIDE_MODE_VELOCITY) != HAL_OK) UartLog("[TEST] vel prep fail\r\n");
//  (void)snprintf(msg, sizeof(msg), "[TEST] vel run=%u\r\n", g_state.run_mode);
//  UartLog(msg);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LIMIT_CUR, 5.0f);
//  HAL_Delay(10);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_ACC_RAD, 8.0f);
//  HAL_Delay(10);
//  UartLog("[TEST] velocity\r\n");
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_SPD_REF, 8.0f);
//  HAL_Delay(1400);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_SPD_REF, -8.0f);
//  HAL_Delay(1400);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_SPD_REF, 0.0f);
//  HAL_Delay(400);
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  HAL_Delay(100);
//
//  /* CSP position mode */
//  if (RobStride_PrepareMode(ROBSTRIDE_MODE_CSP_POS) != HAL_OK) UartLog("[TEST] csp prep fail\r\n");
//  (void)snprintf(msg, sizeof(msg), "[TEST] csp run=%u\r\n", g_state.run_mode);
//  UartLog(msg);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LIMIT_SPD, 10.0f);
//  HAL_Delay(10);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LIMIT_TRQ, 12.0f);
//  HAL_Delay(10);
//  UartLog("[TEST] csp\r\n");
//  for (i = 0U; i < 3U; i++)
//  {
//    (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LOC_REF, csp_targets[i]);
//    HAL_Delay(1300);
//  }
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  HAL_Delay(100);
//
//  /* PP position mode */
//  if (RobStride_PrepareMode(ROBSTRIDE_MODE_PP_POS) != HAL_OK) UartLog("[TEST] pp prep fail\r\n");
//  (void)snprintf(msg, sizeof(msg), "[TEST] pp run=%u\r\n", g_state.run_mode);
//  UartLog(msg);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LIMIT_TRQ, 12.0f);
//  HAL_Delay(10);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_VEL_MAX, 6.0f);
//  HAL_Delay(10);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_ACC_SET, 10.0f);
//  HAL_Delay(10);
//  UartLog("[TEST] pp\r\n");
//  for (i = 0U; i < 3U; i++)
//  {
//    (void)RobStride_WriteFloat(ROBSTRIDE_IDX_LOC_REF, pp_targets[i]);
//    HAL_Delay(1700);
//  }
//  (void)RobStride_SendStop(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID, 0U);
//  HAL_Delay(100);
//
//  /* Back to safe current mode */
//  (void)RobStride_WriteU8(ROBSTRIDE_IDX_RUN_MODE, ROBSTRIDE_MODE_CURRENT);
//  HAL_Delay(20);
//  (void)RobStride_SendEnable(ROBSTRIDE_CAN_HOST_ID, ROBSTRIDE_CAN_MOTOR_ID);
//  HAL_Delay(50);
//  (void)RobStride_WriteFloat(ROBSTRIDE_IDX_IQ_REF, 0.0f);
//  UartLog("[TEST] all_modes done\r\n");
//}
//

/*
 * robstride.c
 *
 *  Created on: May 8, 2026
 *      Author: dosjan
 *
 *  Refactored: Multi-motor support via RobStrideMotor instance pattern.
 */

#include "robstride.h"
#include "main.h"

/* ─── Globals ────────────────────────────────────────────────────────────── */
extern UART_HandleTypeDef huart3;

RobStrideMotor g_motors[ROBSTRIDE_MAX_MOTORS];
uint8_t        g_motor_count = 0U;

/* ══════════════════════════════════════════════════════════════════════════
 *  Motor pool management
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief  Register a new motor instance.
 * @return 1 on success, 0 if pool is full.
 */
uint8_t RobStride_AddMotor(uint8_t host_id, uint8_t motor_id)
{
  if (g_motor_count >= ROBSTRIDE_MAX_MOTORS)
  {
    UartLog("[MOTOR] pool full, cannot add motor\r\n");
    return 0U;
  }
  g_motors[g_motor_count].host_id  = host_id;
  g_motors[g_motor_count].motor_id = motor_id;
  memset(&g_motors[g_motor_count].state, 0, sizeof(RobStrideState));
  g_motor_count++;
  return 1U;
}

/**
 * @brief  Look up a motor instance by its CAN motor_id.
 * @return Pointer to RobStrideMotor, or NULL if not found.
 */
RobStrideMotor *RobStride_FindMotor(uint8_t motor_id)
{
  uint8_t i;
  for (i = 0U; i < g_motor_count; i++)
  {
    if (g_motors[i].motor_id == motor_id)
    {
      return &g_motors[i];
    }
  }
  return NULL;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Diagnostics / logging
 * ══════════════════════════════════════════════════════════════════════════ */

void UartLog(const char *msg)
{
  (void)HAL_UART_Transmit(&huart3, (const uint8_t *)msg, (uint16_t)strlen(msg), 100U);
}

const char *RobStride_CanLecToStr(uint32_t lec)
{
  switch (lec)
  {
    case 0U: return "NO_ERR";
    case 1U: return "STUFF_ERR";
    case 2U: return "FORM_ERR";
    case 3U: return "ACK_ERR";
    case 4U: return "BIT1_ERR";
    case 5U: return "BIT0_ERR";
    case 6U: return "CRC_ERR";
    case 7U: return "SW_SET";
    default: return "UNKNOWN";
  }
}

void RobStride_LogCanDiag(const char *tag)
{
  char     msg[180];
  uint32_t esr = CAN1->ESR;
  uint32_t msr = CAN1->MSR;
  uint32_t tsr = CAN1->TSR;
  uint32_t lec = (esr & CAN_ESR_LEC_Msk) >> CAN_ESR_LEC_Pos;
  uint32_t tec = (esr & CAN_ESR_TEC_Msk) >> CAN_ESR_TEC_Pos;
  uint32_t rec = (esr & CAN_ESR_REC_Msk) >> CAN_ESR_REC_Pos;

  int len = snprintf(msg, sizeof(msg),
    "[CAN] %s LEC=%lu(%s) TEC=%lu REC=%lu "
    "BOFF=%lu EPVF=%lu EWGF=%lu INAK=%lu "
    "txok0=%lu txok1=%lu txok2=%lu\r\n",
    tag,
    (unsigned long)lec, RobStride_CanLecToStr(lec),
    (unsigned long)tec, (unsigned long)rec,
    (unsigned long)((esr & CAN_ESR_BOFF) ? 1U : 0U),
    (unsigned long)((esr & CAN_ESR_EPVF) ? 1U : 0U),
    (unsigned long)((esr & CAN_ESR_EWGF) ? 1U : 0U),
    (unsigned long)((msr & CAN_MSR_INAK) ? 1U : 0U),
    (unsigned long)((tsr & CAN_TSR_TXOK0) ? 1U : 0U),
    (unsigned long)((tsr & CAN_TSR_TXOK1) ? 1U : 0U),
    (unsigned long)((tsr & CAN_TSR_TXOK2) ? 1U : 0U));

  if (len > 0)
  {
    (void)HAL_UART_Transmit(&huart3, (uint8_t *)msg, (uint16_t)len, 120U);
  }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Math helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static float clampf(float x, float xmin, float xmax)
{
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

static uint16_t float_to_u16(float x, float x_min, float x_max)
{
  const float clamped = clampf(x, x_min, x_max);
  const float norm    = (clamped - x_min) / (x_max - x_min);
  return (uint16_t)(norm * 65535.0f);
}

static float u16_to_float(uint16_t x, float x_min, float x_max)
{
  return ((float)x * (x_max - x_min) / 65535.0f) + x_min;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  CAN frame helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static uint32_t robstride_build_ext_id(uint8_t comm_type,
                                        uint16_t data_area2,
                                        uint8_t  data_area1)
{
  return (((uint32_t)comm_type   & 0x1FU) << 24U) |
         (((uint32_t)data_area2  & 0xFFFFU) << 8U) |
          ((uint32_t)data_area1  & 0xFFU);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  CAN low-level TX / RX  (hardware-facing, shared by all motors)
 * ══════════════════════════════════════════════════════════════════════════ */

HAL_StatusTypeDef can1_send_ext(uint32_t ext_id, const uint8_t data[8])
{
  uint32_t mailbox   = 0U;
  uint32_t rqcp      = 0U;
  uint32_t txok      = 0U;
  const uint32_t tickstart = HAL_GetTick();

  /* Wait for a free TX mailbox */
  while ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0U)
  {
    if ((HAL_GetTick() - tickstart) > 10U)
    {
      RobStride_LogCanDiag("MAILBOX_TIMEOUT");
      return HAL_TIMEOUT;
    }
  }

  if      ((CAN1->TSR & CAN_TSR_TME0) != 0U) { mailbox = 0U; rqcp = CAN_TSR_RQCP0; txok = CAN_TSR_TXOK0; }
  else if ((CAN1->TSR & CAN_TSR_TME1) != 0U) { mailbox = 1U; rqcp = CAN_TSR_RQCP1; txok = CAN_TSR_TXOK1; }
  else                                         { mailbox = 2U; rqcp = CAN_TSR_RQCP2; txok = CAN_TSR_TXOK2; }

  CAN1->sTxMailBox[mailbox].TDTR = 8U;
  CAN1->sTxMailBox[mailbox].TDLR = ((uint32_t)data[3] << 24U) |
                                   ((uint32_t)data[2] << 16U) |
                                   ((uint32_t)data[1] <<  8U) |
                                    (uint32_t)data[0];
  CAN1->sTxMailBox[mailbox].TDHR = ((uint32_t)data[7] << 24U) |
                                   ((uint32_t)data[6] << 16U) |
                                   ((uint32_t)data[5] <<  8U) |
                                    (uint32_t)data[4];
  CAN1->sTxMailBox[mailbox].TIR  = (ext_id << 3U) | CAN_TI0R_IDE | CAN_TI0R_TXRQ;

  /* Wait for TX complete */
  while ((CAN1->TSR & rqcp) == 0U)
  {
    if ((HAL_GetTick() - tickstart) > 10U)
    {
      RobStride_LogCanDiag("TXRQ_TIMEOUT");
      return HAL_TIMEOUT;
    }
  }

  {
    const uint32_t tsr = CAN1->TSR;
    CAN1->TSR = rqcp;                  /* clear RQCP flag */
    if ((tsr & txok) == 0U)
    {
      RobStride_LogCanDiag("TX_NOACK_OR_FAIL");
      return HAL_ERROR;
    }
  }
  return HAL_OK;
}

HAL_StatusTypeDef can1_recv_ext(uint32_t *ext_id, uint8_t data[8], uint8_t *dlc)
{
  uint32_t rir, rdtr, rdlr, rdhr;

  if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0U)
  {
    return HAL_ERROR;   /* FIFO empty */
  }

  rir  = CAN1->sFIFOMailBox[0].RIR;
  rdtr = CAN1->sFIFOMailBox[0].RDTR;
  rdlr = CAN1->sFIFOMailBox[0].RDLR;
  rdhr = CAN1->sFIFOMailBox[0].RDHR;
  CAN1->RF0R |= CAN_RF0R_RFOM0;        /* release FIFO slot */

  if ((rir & CAN_RI0R_IDE) == 0U)
  {
    return HAL_ERROR;   /* not an extended frame */
  }

  *ext_id = (rir >> 3U) & 0x1FFFFFFFU;
  *dlc    = (uint8_t)(rdtr & 0x0FU);
  data[0] = (uint8_t)(rdlr);
  data[1] = (uint8_t)(rdlr >>  8U);
  data[2] = (uint8_t)(rdlr >> 16U);
  data[3] = (uint8_t)(rdlr >> 24U);
  data[4] = (uint8_t)(rdhr);
  data[5] = (uint8_t)(rdhr >>  8U);
  data[6] = (uint8_t)(rdhr >> 16U);
  data[7] = (uint8_t)(rdhr >> 24U);
  return HAL_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Motor commands  (all take RobStrideMotor*)
 * ══════════════════════════════════════════════════════════════════════════ */

HAL_StatusTypeDef RobStride_Enable(RobStrideMotor *m)
{
  const uint16_t target = ((uint16_t)m->host_id << 8U) | m->motor_id;
  const uint8_t  data[8] = {0};
  return can1_send_ext(robstride_build_ext_id(ROBSTRIDE_CMD_ENABLE, target, m->motor_id), data);
}

HAL_StatusTypeDef RobStride_Stop(RobStrideMotor *m, uint8_t clear_fault)
{
  const uint16_t target = ((uint16_t)m->host_id << 8U) | m->motor_id;
  uint8_t data[8] = {0};
  data[0] = clear_fault ? 1U : 0U;
  return can1_send_ext(robstride_build_ext_id(ROBSTRIDE_CMD_STOP, target, m->motor_id), data);
}

HAL_StatusTypeDef RobStride_OpControl(RobStrideMotor *m,
                                       float torque_nm,
                                       float target_pos_rad,
                                       float target_vel_rad_s,
                                       float kp, float kd)
{
  const uint16_t torque_u16 = float_to_u16(torque_nm,        ROBSTRIDE_TORQUE_MIN_NM, ROBSTRIDE_TORQUE_MAX_NM);
  const uint16_t pos_u16    = float_to_u16(target_pos_rad,   ROBSTRIDE_POS_MIN_RAD,   ROBSTRIDE_POS_MAX_RAD);
  const uint16_t vel_u16    = float_to_u16(target_vel_rad_s, ROBSTRIDE_VEL_MIN_RAD_S, ROBSTRIDE_VEL_MAX_RAD_S);
  const uint16_t kp_u16     = float_to_u16(kp,               ROBSTRIDE_KP_MIN,        ROBSTRIDE_KP_MAX);
  const uint16_t kd_u16     = float_to_u16(kd,               ROBSTRIDE_KD_MIN,        ROBSTRIDE_KD_MAX);

  const uint8_t data[8] = {
    (uint8_t)(pos_u16 >> 8U), (uint8_t)(pos_u16 & 0xFFU),
    (uint8_t)(vel_u16 >> 8U), (uint8_t)(vel_u16 & 0xFFU),
    (uint8_t)(kp_u16  >> 8U), (uint8_t)(kp_u16  & 0xFFU),
    (uint8_t)(kd_u16  >> 8U), (uint8_t)(kd_u16  & 0xFFU)
  };

  return can1_send_ext(
    robstride_build_ext_id(ROBSTRIDE_CMD_MOTION_CTRL, torque_u16, m->motor_id),
    data);
}

HAL_StatusTypeDef RobStride_WriteFloat(RobStrideMotor *m, uint16_t index, float value)
{
  uint8_t data[8] = {0};
  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_WRITE,
                                            (uint16_t)m->host_id << 8U,
                                            m->motor_id);
  data[0] = (uint8_t)(index & 0xFFU);
  data[1] = (uint8_t)(index >> 8U);
  memcpy(&data[4], &value, sizeof(float));
  return can1_send_ext(ext_id, data);
}

HAL_StatusTypeDef RobStride_WriteU8(RobStrideMotor *m, uint16_t index, uint8_t value)
{
  uint8_t data[8] = {0};
  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_WRITE,
                                            (uint16_t)m->host_id << 8U,
                                            m->motor_id);
  data[0] = (uint8_t)(index & 0xFFU);
  data[1] = (uint8_t)(index >> 8U);
  data[4] = value;
  return can1_send_ext(ext_id, data);
}

HAL_StatusTypeDef RobStride_WriteU16(RobStrideMotor *m, uint16_t index, uint16_t value)
{
  uint8_t data[8] = {0};
  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_WRITE,
                                            (uint16_t)m->host_id << 8U,
                                            m->motor_id);
  data[0] = (uint8_t)(index & 0xFFU);
  data[1] = (uint8_t)(index >> 8U);
  data[4] = (uint8_t)(value & 0xFFU);
  data[5] = (uint8_t)(value >> 8U);
  return can1_send_ext(ext_id, data);
}

HAL_StatusTypeDef RobStride_ReadParam(RobStrideMotor *m, uint16_t index)
{
  uint8_t data[8] = {0};
  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_PARAM_READ,
                                            (uint16_t)m->host_id << 8U,
                                            m->motor_id);
  data[0] = (uint8_t)(index & 0xFFU);
  data[1] = (uint8_t)(index >> 8U);
  return can1_send_ext(ext_id, data);
}

HAL_StatusTypeDef RobStride_EnableActiveReport(RobStrideMotor *m, uint16_t report_ms)
{
  uint8_t  data[8] = {1U, 2U, 3U, 4U, 5U, 6U, 1U, 0U};
  uint16_t ep = (report_ms <= 10U) ? 1U : (uint16_t)(((report_ms - 10U) / 5U) + 1U);
  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_ACTIVE_REPORT,
                                            (uint16_t)m->host_id << 8U,
                                            m->motor_id);
  if (can1_send_ext(ext_id, data) != HAL_OK) return HAL_ERROR;
  HAL_Delay(5U);
  return RobStride_WriteU16(m, ROBSTRIDE_IDX_REPORT_TIME, ep);
}

HAL_StatusTypeDef RobStride_DisableActiveReport(RobStrideMotor *m)
{
  uint8_t  data[8] = {1U, 2U, 3U, 4U, 5U, 6U, 0U, 0U};
  uint32_t ext_id = robstride_build_ext_id(ROBSTRIDE_CMD_ACTIVE_REPORT,
                                            (uint16_t)m->host_id << 8U,
                                            m->motor_id);
  return can1_send_ext(ext_id, data);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  RX processing  — dispatches to the correct motor by sender ID
 * ══════════════════════════════════════════════════════════════════════════ */

void RobStride_ProcessRx(void)
{
  uint8_t        data[8];
  uint8_t        dlc;
  uint32_t       ext_id;
  int            n;

  for (n = 0; n < 16; n++)
  {
    if (can1_recv_ext(&ext_id, data, &dlc) != HAL_OK) break;
    if (dlc < 8U) continue;

    uint8_t ct        = (uint8_t)((ext_id >> 24U) & 0x1FU);
    uint8_t sender_id = (uint8_t)((ext_id >> 8U) & 0xFFU);  /* data_area2 low byte = motor_id */

    RobStrideMotor *m = RobStride_FindMotor(sender_id);
    if (m == NULL) continue;   /* unknown motor, discard */

    RobStrideState *st = &m->state;

    if ((ct == ROBSTRIDE_CMD_MOTOR_FEEDBACK) || (ct == ROBSTRIDE_CMD_ACTIVE_REPORT))
    {
      uint16_t raw;
      raw            = ((uint16_t)data[0] << 8U) | data[1];
      st->pos_rad    = u16_to_float(raw, ROBSTRIDE_POS_MIN_RAD,   ROBSTRIDE_POS_MAX_RAD);
      raw            = ((uint16_t)data[2] << 8U) | data[3];
      st->vel_rad_s  = u16_to_float(raw, ROBSTRIDE_VEL_MIN_RAD_S, ROBSTRIDE_VEL_MAX_RAD_S);
      raw            = ((uint16_t)data[4] << 8U) | data[5];
      st->torque_nm  = u16_to_float(raw, ROBSTRIDE_TORQUE_MIN_NM, ROBSTRIDE_TORQUE_MAX_NM);
      raw            = ((uint16_t)data[6] << 8U) | data[7];
      st->temp_c     = ((float)raw) / 10.0f;
      st->mode_status = (uint8_t)((ext_id >> 22U) & 0x03U);
      st->fault_bits  = (uint8_t)((ext_id >> 16U) & 0xFFU);
      st->type2_valid = 1U;
    }
    else if (ct == ROBSTRIDE_CMD_PARAM_READ)
    {
      uint16_t index = ((uint16_t)data[1] << 8U) | data[0];
      float    fv;
      memcpy(&fv, &data[4], sizeof(float));

      switch (index)
      {
        case ROBSTRIDE_IDX_VBUS:     st->vbus_v      = fv;      break;
        case ROBSTRIDE_IDX_MECH_POS: st->mech_pos_rad = fv;     break;
        case ROBSTRIDE_IDX_IQFBK:   st->iq_fbk_a    = fv;      break;
        case ROBSTRIDE_IDX_RUN_MODE: st->run_mode    = data[4]; break;
        default: break;
      }
    }
  }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Bulk parameter read
 * ══════════════════════════════════════════════════════════════════════════ */

void RobStride_ReadAllParams(RobStrideMotor *m)
{
  (void)RobStride_ReadParam(m, ROBSTRIDE_IDX_MECH_POS);  HAL_Delay(8U);
  (void)RobStride_ReadParam(m, ROBSTRIDE_IDX_VBUS);      HAL_Delay(8U);
  (void)RobStride_ReadParam(m, ROBSTRIDE_IDX_IQFBK);     HAL_Delay(8U);
  (void)RobStride_ReadParam(m, ROBSTRIDE_IDX_RUN_MODE);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Mode preparation helper
 * ══════════════════════════════════════════════════════════════════════════ */

HAL_StatusTypeDef RobStride_PrepareMode(RobStrideMotor *m, uint8_t run_mode)
{
  uint32_t t0 = HAL_GetTick();

  (void)RobStride_Stop(m, 0U);  HAL_Delay(40U);
  (void)RobStride_Stop(m, 1U);  HAL_Delay(30U);

  if (RobStride_WriteU8(m, ROBSTRIDE_IDX_RUN_MODE, run_mode) != HAL_OK)
  {
    return HAL_ERROR;
  }
  HAL_Delay(25U);

  /* Poll until the motor confirms the new run_mode (timeout 250 ms) */
  while ((HAL_GetTick() - t0) < 250U)
  {
    (void)RobStride_ReadParam(m, ROBSTRIDE_IDX_RUN_MODE);
    HAL_Delay(8U);
    RobStride_ProcessRx();
    if (m->state.run_mode == run_mode) break;
  }

  /* Enable — retry once on failure */
  if (RobStride_Enable(m) != HAL_OK)
  {
    HAL_Delay(20U);
    if (RobStride_Enable(m) != HAL_OK) return HAL_ERROR;
  }
  HAL_Delay(80U);
  return HAL_OK;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Test routines
 * ══════════════════════════════════════════════════════════════════════════ */

void RobStride_RunMaxTorque(RobStrideMotor *m)
{
  char     msg[96];
  uint32_t t0;

  UartLog("[TEST] max_torque begin\r\n");

  if (RobStride_PrepareMode(m, ROBSTRIDE_MODE_CURRENT) != HAL_OK)
  {
    UartLog("[TEST] max_torque prep fail\r\n");
    return;
  }

  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LIMIT_CUR, ROBSTRIDE_MAX_TORQUE_IQ_A);
  HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LIMIT_TRQ, 14.0f);
  HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_IQ_REF, ROBSTRIDE_MAX_TORQUE_IQ_A);

  (void)snprintf(msg, sizeof(msg),
    "[TEST] max_torque motor=%u iq=%.1fA hold=%lums\r\n",
    m->motor_id,
    (double)ROBSTRIDE_MAX_TORQUE_IQ_A,
    (unsigned long)ROBSTRIDE_MAX_TORQUE_HOLD_MS);
  UartLog(msg);

  t0 = HAL_GetTick();
  while ((HAL_GetTick() - t0) < ROBSTRIDE_MAX_TORQUE_HOLD_MS)
  {
    RobStride_ProcessRx();
    HAL_Delay(2U);
  }

  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_IQ_REF, 0.0f);
  HAL_Delay(20U);
  (void)RobStride_Stop(m, 0U);
  UartLog("[TEST] max_torque done\r\n");
}

void RobStride_RunAllModeTests(RobStrideMotor *m)
{
  uint32_t    i;
  char        msg[80];
  const float csp_targets[3] = {0.0f,  0.9f, -0.9f};
  const float pp_targets[3]  = {0.0f,  0.8f, -0.8f};

  (void)snprintf(msg, sizeof(msg), "[TEST] all_modes begin motor=%u\r\n", m->motor_id);
  UartLog(msg);

  /* ── Operation (MIT) mode ─────────────────────────────────────────────── */
  if (RobStride_PrepareMode(m, ROBSTRIDE_MODE_OPERATION) != HAL_OK)
    UartLog("[TEST] op prep fail\r\n");
  (void)snprintf(msg, sizeof(msg), "[TEST] op run=%u\r\n", m->state.run_mode);
  UartLog(msg);
  for (i = 0U; i < 50U; i++)
  {
    (void)RobStride_OpControl(m, 0.0f, 0.0f, 1.5f, 0.0f, 1.5f);
    HAL_Delay(20U);
  }
  for (i = 0U; i < 70U; i++)
  {
    (void)RobStride_OpControl(m, 0.0f, 1.0f, 0.0f, 20.0f, 1.8f);
    HAL_Delay(20U);
  }
  (void)RobStride_Stop(m, 0U);
  HAL_Delay(100U);

  /* ── Current mode ────────────────────────────────────────────────────── */
  if (RobStride_PrepareMode(m, ROBSTRIDE_MODE_CURRENT) != HAL_OK)
    UartLog("[TEST] cur prep fail\r\n");
  (void)snprintf(msg, sizeof(msg), "[TEST] cur run=%u\r\n", m->state.run_mode);
  UartLog(msg);
  for (i = 0U; i <= 12U; i++)
  {
    (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_IQ_REF, (float)i * 0.15f);
    HAL_Delay(100U);
  }
  for (i = 12U; i > 0U; i--)
  {
    (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_IQ_REF, (float)(i - 1U) * 0.15f);
    HAL_Delay(100U);
  }
  (void)RobStride_Stop(m, 0U);
  HAL_Delay(100U);

  /* ── Velocity mode ───────────────────────────────────────────────────── */
  if (RobStride_PrepareMode(m, ROBSTRIDE_MODE_VELOCITY) != HAL_OK)
    UartLog("[TEST] vel prep fail\r\n");
  (void)snprintf(msg, sizeof(msg), "[TEST] vel run=%u\r\n", m->state.run_mode);
  UartLog(msg);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LIMIT_CUR, 5.0f);  HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_ACC_RAD,   8.0f);  HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_SPD_REF,   8.0f);  HAL_Delay(1400U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_SPD_REF,  -8.0f);  HAL_Delay(1400U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_SPD_REF,   0.0f);  HAL_Delay(400U);
  (void)RobStride_Stop(m, 0U);
  HAL_Delay(100U);

  /* ── CSP position mode ───────────────────────────────────────────────── */
  if (RobStride_PrepareMode(m, ROBSTRIDE_MODE_CSP_POS) != HAL_OK)
    UartLog("[TEST] csp prep fail\r\n");
  (void)snprintf(msg, sizeof(msg), "[TEST] csp run=%u\r\n", m->state.run_mode);
  UartLog(msg);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LIMIT_SPD, 10.0f); HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LIMIT_TRQ, 12.0f); HAL_Delay(10U);
  for (i = 0U; i < 3U; i++)
  {
    (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LOC_REF, csp_targets[i]);
    HAL_Delay(1300U);
  }
  (void)RobStride_Stop(m, 0U);
  HAL_Delay(100U);

  /* ── PP position mode ────────────────────────────────────────────────── */
  if (RobStride_PrepareMode(m, ROBSTRIDE_MODE_PP_POS) != HAL_OK)
    UartLog("[TEST] pp prep fail\r\n");
  (void)snprintf(msg, sizeof(msg), "[TEST] pp run=%u\r\n", m->state.run_mode);
  UartLog(msg);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LIMIT_TRQ, 12.0f); HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_VEL_MAX,    6.0f); HAL_Delay(10U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_ACC_SET,   10.0f); HAL_Delay(10U);
  for (i = 0U; i < 3U; i++)
  {
    (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_LOC_REF, pp_targets[i]);
    HAL_Delay(1700U);
  }
  (void)RobStride_Stop(m, 0U);
  HAL_Delay(100U);

  /* ── Return to safe current mode ─────────────────────────────────────── */
  (void)RobStride_WriteU8   (m, ROBSTRIDE_IDX_RUN_MODE, ROBSTRIDE_MODE_CURRENT);
  HAL_Delay(20U);
  (void)RobStride_Enable    (m);
  HAL_Delay(50U);
  (void)RobStride_WriteFloat(m, ROBSTRIDE_IDX_IQ_REF, 0.0f);

  (void)snprintf(msg, sizeof(msg), "[TEST] all_modes done motor=%u\r\n", m->motor_id);
  UartLog(msg);
}
