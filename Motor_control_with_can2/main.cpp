
// #include <Arduino.h>       // Serial, delay, millis — PlatformIO-д заавал хэрэгтэй
// #include <string.h>        // memset, memcpy
// #include <math.h>          // fabs
// #include "driver/twai.h"

// // ──────────────────────────────────────────────
// //  FORWARD DECLARATIONS
// // ──────────────────────────────────────────────
// void demo_OperationMode();
// void demo_CurrentMode();
// void demo_VelocityMode();
// void demo_PositionCSP();
// void demo_PositionPP();

// // ──────────────────────────────────────────────
// //  PIN CONFIG
// // ──────────────────────────────────────────────
// #define TWAI_TX_PIN  GPIO_NUM_21
// #define TWAI_RX_PIN  GPIO_NUM_22

// // ──────────────────────────────────────────────
// //  MOTOR CONFIG
// // ──────────────────────────────────────────────
// #define MOTOR_ID     0x01   // Motor CAN ID (default 1)
// #define HOST_ID      0xFD   // Host CAN ID  (default 0xFD = 253)

// // ──────────────────────────────────────────────
// //  PARAMETER LIMITS (from manual)
// // ──────────────────────────────────────────────
// #define P_MIN   -12.57f     // rad
// #define P_MAX    12.57f     // rad
// #define V_MIN   -33.0f      // rad/s
// #define V_MAX    33.0f      // rad/s
// #define KP_MIN   0.0f
// #define KP_MAX   500.0f
// #define KD_MIN   0.0f
// #define KD_MAX   5.0f
// #define T_MIN   -14.0f      // N·m
// #define T_MAX    14.0f      // N·m

// // ──────────────────────────────────────────────
// //  COMMUNICATION TYPE IDs
// // ──────────────────────────────────────────────
// #define CMD_GET_ID          0x00
// #define CMD_MOTION_CTRL     0x01   // Operation control mode
// #define CMD_MOTOR_FEEDBACK  0x02
// #define CMD_ENABLE          0x03
// #define CMD_STOP            0x04
// #define CMD_SET_ZERO        0x06
// #define CMD_SET_CAN_ID      0x07
// #define CMD_PARAM_READ      0x11
// #define CMD_PARAM_WRITE     0x12   // Lost on power off (use with type 0x16 to save)
// #define CMD_FAULT_FEEDBACK  0x15
// #define CMD_SAVE_PARAMS     0x16
// #define CMD_ACTIVE_REPORT   0x18
// #define CMD_SET_PROTOCOL    0x19

// // ──────────────────────────────────────────────
// //  PARAMETER INDEX (0x7000 series)
// // ──────────────────────────────────────────────
// #define IDX_RUN_MODE    0x7005  // uint8: 0=MIT/Op, 1=PP pos, 2=Velocity, 3=Current, 5=CSP pos
// #define IDX_IQ_REF      0x7006  // float: current cmd (A), -16~16
// #define IDX_SPD_REF     0x700A  // float: speed cmd (rad/s), -33~33
// #define IDX_LIMIT_TRQ   0x700B  // float: torque limit (N·m), 0~14
// #define IDX_CUR_KP      0x7010  // float
// #define IDX_CUR_KI      0x7011  // float
// #define IDX_CUR_FILT    0x7014  // float: 0~1
// #define IDX_LOC_REF     0x7016  // float: position cmd (rad)
// #define IDX_LIMIT_SPD   0x7017  // float: CSP speed limit (rad/s)
// #define IDX_LIMIT_CUR   0x7018  // float: velocity/pos current limit (A)
// #define IDX_MECH_POS    0x7019  // float: READ ONLY
// #define IDX_IQF         0x701A  // float: READ ONLY
// #define IDX_MECH_VEL    0x701B  // float: READ ONLY
// #define IDX_VBUS        0x701C  // float: READ ONLY
// #define IDX_LOC_KP      0x701E  // float: default 40
// #define IDX_SPD_KP      0x701F  // float: default 6
// #define IDX_SPD_KI      0x7020  // float: default 0.02
// #define IDX_SPD_FILT    0x7021  // float: default 0.1
// #define IDX_ACC_RAD     0x7022  // float: velocity mode accel (rad/s²)
// #define IDX_VEL_MAX     0x7024  // float: PP mode max speed (rad/s)
// #define IDX_ACC_SET     0x7025  // float: PP mode accel (rad/s²)
// #define IDX_REPORT_TIME 0x7026  // uint16: report interval
// #define IDX_CAN_TIMEOUT 0x7028  // uint32: 20000 = 1s
// #define IDX_ZERO_STA    0x7029  // uint8: 0 = 0~2π, 1 = -π~π
// #define IDX_DAMPER      0x702A  // uint8: damping switch
// #define IDX_ADD_OFFSET  0x702B  // float: zero offset

// // ──────────────────────────────────────────────
// //  RUN MODE VALUES
// // ──────────────────────────────────────────────
// #define MODE_OPERATION  0   // MIT / Operation control
// #define MODE_PP_POS     1   // Profile Position
// #define MODE_VELOCITY   2   // Velocity
// #define MODE_CURRENT    3   // Current (Iq)
// #define MODE_CSP_POS    5   // Cyclic Synchronous Position

// // ──────────────────────────────────────────────
// //  MOTOR FEEDBACK STRUCT
// // ──────────────────────────────────────────────
// struct MotorFeedback {
//   uint8_t  motorId;
//   float    position;   // rad
//   float    velocity;   // rad/s
//   float    torque;     // N·m
//   float    tempCelsius;
//   uint8_t  modeStatus; // 0=Reset, 1=Cali, 2=Run
//   uint8_t  faultBits;  // bit flags from feedback frame
//   bool     valid;
// };

// MotorFeedback g_feedback = {0};

// // ──────────────────────────────────────────────
// //  HELPER: float <-> uint mapping
// // ──────────────────────────────────────────────
// uint16_t floatToUint16(float x, float xMin, float xMax) {
//   if (x > xMax) x = xMax;
//   if (x < xMin) x = xMin;
//   return (uint16_t)((x - xMin) / (xMax - xMin) * 65535.0f);
// }
// float uint16ToFloat(uint16_t x, float xMin, float xMax) {
//   return xMin + (float)x / 65535.0f * (xMax - xMin);
// }

// // ──────────────────────────────────────────────
// //  BUILD 29-BIT EXTENDED CAN ID
// //  Bit 28~24: comm type (5 bits)
// //  Bit 23~8 : data area 2 (16 bits) — carries host_id in bit15~8
// //  Bit  7~0 : destination motor CAN ID
// // ──────────────────────────────────────────────
// uint32_t buildExtId(uint8_t commType, uint16_t dataArea2, uint8_t destId) {
//   return ((uint32_t)(commType & 0x1F) << 24)
//        | ((uint32_t)(dataArea2 & 0xFFFF) << 8)
//        | ((uint32_t)(destId & 0xFF));
// }
// // ──────────────────────────────────────────────
// //  SEND CAN FRAME
// // ──────────────────────────────────────────────
// bool canSend(uint32_t extId, uint8_t* data, uint8_t len) {
//   twai_message_t msg;
//   msg.extd       = 1;           // Extended frame (29-bit)
//   msg.rtr        = 0;
//   msg.ss         = 0;
//   msg.self       = 0;
//   msg.dlc_non_comp = 0;
//   msg.identifier = extId;
//   msg.data_length_code = len;
//   memset(msg.data, 0, 8);
//   if (data) memcpy(msg.data, data, len);
//   esp_err_t result = twai_transmit(&msg, pdMS_TO_TICKS(10));
//   return (result == ESP_OK);
// }
// // ──────────────────────────────────────────────
// //  PARSE FEEDBACK FRAME (Type 2 / Type 24)
// //
// //  29-bit ID layout (manual section 4.1.2):
// //    bit28~24 : comm type (5 bits)  must be 0x02 or 0x18
// //    bit23~22 : mode  (0=Reset, 1=Cali, 2=Run)
// //    bit21~16 : fault flags
// //    bit15~8  : motor CAN ID
// //    bit7~0   : host CAN ID
// // ──────────────────────────────────────────────
// void parseFeedback(twai_message_t* msg) {
//   if (!msg->extd) return;
//   uint32_t id      = msg->identifier;
//   uint8_t commType = (id >> 24) & 0x1F;
//   if (commType != CMD_MOTOR_FEEDBACK && commType != 0x18) return;
//   uint16_t rawPos  = ((uint16_t)msg->data[0] << 8) | msg->data[1];
//   uint16_t rawVel  = ((uint16_t)msg->data[2] << 8) | msg->data[3];
//   uint16_t rawTrq  = ((uint16_t)msg->data[4] << 8) | msg->data[5];
//   uint16_t rawTemp = ((uint16_t)msg->data[6] << 8) | msg->data[7];
//   g_feedback.position    = uint16ToFloat(rawPos, P_MIN, P_MAX);
//   g_feedback.velocity    = uint16ToFloat(rawVel, V_MIN, V_MAX);
//   g_feedback.torque      = uint16ToFloat(rawTrq, T_MIN, T_MAX);
//   g_feedback.tempCelsius = (float)rawTemp / 10.0f;
//   uint8_t statusByte    = (id >> 16) & 0xFF;
//   g_feedback.modeStatus = (statusByte >> 6) & 0x03;
//   g_feedback.faultBits  = statusByte & 0x3F;
//   g_feedback.motorId    = (id >> 8) & 0xFF;
//   g_feedback.valid      = true;
// }

// // ──────────────────────────────────────────────
// //  RECEIVE (non-blocking, call in loop)
// // ──────────────────────────────────────────────
// bool canReceive() {
//   twai_message_t msg;
//   if (twai_receive(&msg, 0) == ESP_OK) {
//     parseFeedback(&msg);
//     return true;
//   }
//   return false;
// }

// // ══════════════════════════════════════════════
// //  MOTOR COMMANDS
// // ══════════════════════════════════════════════

// // ── Enable motor (Type 3) ──────────────────────
// void motorEnable(uint8_t id) {
//   uint32_t extId = buildExtId(CMD_ENABLE, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   canSend(extId, data, 8);
//   Serial.println("[CMD] Motor ENABLE sent");
// }

// // ── Stop motor (Type 4) ───────────────────────
// void motorStop(uint8_t id) {
//   uint32_t extId = buildExtId(CMD_STOP, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   canSend(extId, data, 8);
//   Serial.println("[CMD] Motor STOP sent");
// }

// // ── Clear fault: set data[0]=1 in stop frame ──
// void motorClearFault(uint8_t id) {
//   uint32_t extId = buildExtId(CMD_STOP, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
//   canSend(extId, data, 8);
//   Serial.println("[CMD] Fault CLEAR sent");
// }

// // ── Set mechanical zero (Type 6) ─────────────
// void motorSetZero(uint8_t id) {
//   uint32_t extId = buildExtId(CMD_SET_ZERO, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
//   canSend(extId, data, 8);
//   Serial.println("[CMD] Set ZERO sent");
// }

// // ── Save parameters to flash (Type 22) ───────
// void motorSaveParams(uint8_t id) {
//   uint32_t extId = buildExtId(CMD_SAVE_PARAMS, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
//   canSend(extId, data, 8);
//   Serial.println("[CMD] SAVE params sent");
// }

// // ──────────────────────────────────────────────
// //  WRITE SINGLE PARAMETER (Type 18, 0x12)
// //  Byte0~1: index (little-endian)
// //  Byte4~7: value (float or uint, little-endian)
// // ──────────────────────────────────────────────
// void motorWriteFloat(uint8_t id, uint16_t index, float value) {
//   uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);   // index little-endian
//   memcpy(&data[4], &value, 4);   // float little-endian
//   canSend(extId, data, 8);
// }

// void motorWriteUint8(uint8_t id, uint16_t index, uint8_t value) {
//   uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   data[4] = value;
//   canSend(extId, data, 8);
// }

// void motorWriteUint32(uint8_t id, uint16_t index, uint32_t value) {
//   uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   memcpy(&data[4], &value, 4);
//   canSend(extId, data, 8);
// }

// // ──────────────────────────────────────────────
// //  SET RUN MODE (must be done before enable)
// // ──────────────────────────────────────────────
// void motorSetMode(uint8_t id, uint8_t mode) {
//   motorWriteUint8(id, IDX_RUN_MODE, mode);
//   delay(5);
//   Serial.printf("[CMD] Mode set to %d\n", mode);
// }

// // ══════════════════════════════════════════════
// //  MODE 0: OPERATION / MIT CONTROL (Type 1)
// //  Sends: position, velocity, Kp, Kd, torque_ff
// // ══════════════════════════════════════════════
// void motorOperationControl(uint8_t id,
//                             float torque_ff,
//                             float position,
//                             float velocity,
//                             float kp,
//                             float kd) {
//   // Torque goes in the 29-bit ID data area (bits 23~8)
//   uint16_t tq_raw = floatToUint16(torque_ff, T_MIN, T_MAX);
//   uint32_t extId  = buildExtId(CMD_MOTION_CTRL,
//                                ((uint16_t)HOST_ID << 8) | (tq_raw >> 8),
//                                id);
//   // Actually per manual: Byte2 of data area 2 = torque high byte
//   // Let's follow the manual exactly:
//   // bit23~8 = data area 2, where Byte2(bit23~16)=torque high, rest=host
//   // Simplified: put torque uint16 into bits 23~8
//   uint32_t extIdCorrect = buildExtId(CMD_MOTION_CTRL, tq_raw, id);

//   uint8_t data[8];
//   uint16_t pos_raw = floatToUint16(position, P_MIN, P_MAX);
//   uint16_t vel_raw = floatToUint16(velocity, V_MIN, V_MAX);
//   uint16_t kp_raw  = floatToUint16(kp, KP_MIN, KP_MAX);
//   uint16_t kd_raw  = floatToUint16(kd, KD_MIN, KD_MAX);

//   data[0] = pos_raw >> 8;
//   data[1] = pos_raw & 0xFF;
//   data[2] = vel_raw >> 8;
//   data[3] = vel_raw & 0xFF;
//   data[4] = kp_raw  >> 8;
//   data[5] = kp_raw  & 0xFF;
//   data[6] = kd_raw  >> 8;
//   data[7] = kd_raw  & 0xFF;

//   canSend(extIdCorrect, data, 8);
// }

// // ══════════════════════════════════════════════
// //  MODE 3: CURRENT MODE
// //  Set Iq reference current in Amps
// // ══════════════════════════════════════════════
// void motorSetCurrent(uint8_t id, float iq_amps) {
//   motorWriteFloat(id, IDX_IQ_REF, iq_amps);
// }

// // ══════════════════════════════════════════════
// //  MODE 2: VELOCITY MODE
// // ══════════════════════════════════════════════
// void motorSetVelocity(uint8_t id, float vel_rad_s) {
//   motorWriteFloat(id, IDX_SPD_REF, vel_rad_s);
// }

// void motorSetVelocityAccel(uint8_t id, float accel_rad_s2) {
//   motorWriteFloat(id, IDX_ACC_RAD, accel_rad_s2);
// }

// void motorSetCurrentLimit(uint8_t id, float cur_amps) {
//   motorWriteFloat(id, IDX_LIMIT_CUR, cur_amps);
// }

// // ══════════════════════════════════════════════
// //  MODE 5: POSITION MODE (CSP)
// //  Cyclic Synchronous Position
// // ══════════════════════════════════════════════
// void motorSetPositionCSP(uint8_t id, float pos_rad) {
//   motorWriteFloat(id, IDX_LOC_REF, pos_rad);
// }

// void motorSetSpeedLimitCSP(uint8_t id, float spd_limit_rad_s) {
//   motorWriteFloat(id, IDX_LIMIT_SPD, spd_limit_rad_s);
// }

// // ══════════════════════════════════════════════
// //  MODE 1: POSITION MODE (PP - Profile Position)
// //  Set speed, accel, then position
// // ══════════════════════════════════════════════
// void motorSetPositionPP(uint8_t id, float pos_rad, float spd, float accel) {
//   motorWriteFloat(id, IDX_VEL_MAX, spd);
//   delay(2);
//   motorWriteFloat(id, IDX_ACC_SET, accel);
//   delay(2);
//   motorWriteFloat(id, IDX_LOC_REF, pos_rad);
// }

// // ──────────────────────────────────────────────
// //  PRINT FEEDBACK
// // ──────────────────────────────────────────────
// void printFeedback() {
//   if (!g_feedback.valid) return;
//   Serial.printf("[FB] ID:%d | Pos:%.3f rad | Vel:%.3f rad/s | Trq:%.3f Nm | Temp:%.1f°C | Mode:%d | Fault:0x%02X\n",
//     g_feedback.motorId,
//     g_feedback.position,
//     g_feedback.velocity,
//     g_feedback.torque,
//     g_feedback.tempCelsius,
//     g_feedback.modeStatus,
//     g_feedback.faultBits);
// }

// // ══════════════════════════════════════════════
// //  SETUP
// // ══════════════════════════════════════════════
// void setup() {
//   Serial.begin(115200);
//   delay(500);
//   Serial.println("=== RobStride RS00 - ESP32 TWAI Demo ===");

//   // Configure TWAI driver
//   twai_general_config_t g_config = {
//     .mode           = TWAI_MODE_NORMAL,
//     .tx_io          = TWAI_TX_PIN,
//     .rx_io          = TWAI_RX_PIN,
//     .clkout_io      = TWAI_IO_UNUSED,
//     .bus_off_io     = TWAI_IO_UNUSED,
//     .tx_queue_len   = 10,
//     .rx_queue_len   = 10,
//     .alerts_enabled = TWAI_ALERT_NONE,
//     .clkout_divider = 0
//   };

//   // 1 Mbps timing for 80MHz APB clock
//   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();

//   // Accept all frames
//   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//   if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
//     Serial.println("[ERROR] TWAI driver install failed!");
//     while (1) delay(1000);
//   }
//   if (twai_start() != ESP_OK) {
//     Serial.println("[ERROR] TWAI start failed!");
//     while (1) delay(1000);
//   }
//   Serial.println("[OK] TWAI started at 1 Mbps");

//   delay(200);

//   // ── DEMO: Choose one mode below and uncomment ──

//   //demo_OperationMode();
//    demo_CurrentMode();
//   // demo_VelocityMode();
//   // demo_PositionCSP();
//   // demo_PositionPP();
// }

// // ══════════════════════════════════════════════
// //  DEMO FUNCTIONS
// // ══════════════════════════════════════════════

// // ── DEMO 1: Operation (MIT) Control Mode ─────
// void demo_OperationMode() {
//   Serial.println("\n--- Demo: Operation Control Mode ---");

//   // Motor defaults to operation mode on power-on, no mode switch needed
//   motorEnable(MOTOR_ID);
//   delay(100);

//   // Run at 1 rad/s: set Kd=1, v_set=1, others=0
//   Serial.println("Running at 1 rad/s (Kd=1, v=1)...");
//   for (int i = 0; i < 50; i++) {
//     motorOperationControl(MOTOR_ID,
//       0.0f,   // torque feedforward (N·m)
//       0.0f,   // target position (rad)
//       1.0f,   // target velocity (rad/s)
//       0.0f,   // Kp
//       1.0f    // Kd
//     );
//     canReceive();
//     printFeedback();
//     delay(20);
//   }

//   // Hold position at 3 rad: Kp=10, Kd=1
//   Serial.println("Holding position at 3 rad...");
//   for (int i = 0; i < 100; i++) {
//     motorOperationControl(MOTOR_ID, 0.0f, 3.0f, 0.0f, 10.0f, 1.0f);
//     canReceive();
//     printFeedback();
//     delay(20);
//   }

//   motorStop(MOTOR_ID);
// }
// // ── DEMO 2: Current Mode ──────────────────────
// void demo_CurrentMode() {
//   Serial.println("\n--- Demo: Current Mode ---");
//   motorSetMode(MOTOR_ID, MODE_CURRENT);
//   delay(20);
//   motorEnable(MOTOR_ID);
//   delay(100);
//   // Ramp current from 0 to 2A
//   for (float iq = 0.0f; iq <= 2.0f; iq += 0.1f) {
//     motorSetCurrent(MOTOR_ID, iq);
//     canReceive();
//     printFeedback();
//     delay(50);
//   }
//   // Hold 2A for 1 second
//   for (int i = 0; i < 20; i++) {
//     motorSetCurrent(MOTOR_ID, 2.0f);
//     canReceive();
//     printFeedback();
//     delay(50);
//   }
//   // Ramp back to 0
//   for (float iq = 2.0f; iq >= 0.0f; iq -= 0.1f) {
//     motorSetCurrent(MOTOR_ID, iq);
//     delay(50);
//   }
//   motorStop(MOTOR_ID);
// }

// // ── DEMO 3: Velocity Mode ─────────────────────
// void demo_VelocityMode() {
//   Serial.println("\n--- Demo: Velocity Mode ---");
//   motorSetMode(MOTOR_ID, MODE_VELOCITY);
//   delay(20);
//   // Set limits before enabling
//   motorSetCurrentLimit(MOTOR_ID, 10.0f);    // 10A max
//   motorSetVelocityAccel(MOTOR_ID, 5.0f);    // 5 rad/s²
//   delay(20);
//   motorEnable(MOTOR_ID);
//   delay(100);
//   // Speed up to 5 rad/s
//   Serial.println("Setting speed: 5 rad/s");
//   motorSetVelocity(MOTOR_ID, 5.0f);
//   for (int i = 0; i < 100; i++) {
//     canReceive();
//     printFeedback();
//     delay(20);
//   }
//   // Reverse
//   Serial.println("Reversing: -3 rad/s");
//   motorSetVelocity(MOTOR_ID, -3.0f);
//   for (int i = 0; i < 100; i++) {
//     canReceive();
//     printFeedback();
//     delay(20);
//   }

//   // Stop
//   motorSetVelocity(MOTOR_ID, 0.0f);
//   delay(500);
//   motorStop(MOTOR_ID);
// }

// // ── DEMO 4: Position Mode CSP ─────────────────
// void demo_PositionCSP() {
//   Serial.println("\n--- Demo: Position Mode CSP ---");

//   motorSetMode(MOTOR_ID, MODE_CSP_POS);
//   delay(20);

//   motorSetSpeedLimitCSP(MOTOR_ID, 5.0f);   // max 5 rad/s
//   motorWriteFloat(MOTOR_ID, IDX_LIMIT_TRQ, 10.0f); // 10 N·m torque limit
//   delay(20);

//   motorEnable(MOTOR_ID);
//   delay(100);

//   float targets[] = {0.0f, 1.57f, 3.14f, 1.57f, 0.0f};  // 0, 90°, 180°, 90°, 0°
//   for (float target : targets) {
//     Serial.printf("Moving to %.2f rad (%.0f°)\n", target, target * 57.3f);
//     motorSetPositionCSP(MOTOR_ID, target);

//     // Wait for motor to reach position
//     unsigned long t0 = millis();
//     while (millis() - t0 < 3000) {
//       canReceive();
//       if (g_feedback.valid) {
//         printFeedback();
//         if (fabs(g_feedback.position - target) < 0.05f) {
//           Serial.println("  -> Reached target!");
//           break;
//         }
//       }
//       delay(20);
//     }
//     delay(500);
//   }

//   motorStop(MOTOR_ID);
// }

// // ── DEMO 5: Position Mode PP ──────────────────
// void demo_PositionPP() {
//   Serial.println("\n--- Demo: Position Mode PP ---");
//   motorSetMode(MOTOR_ID, MODE_PP_POS);
//   delay(20);
//   // Set torque limit
//   motorWriteFloat(MOTOR_ID, IDX_LIMIT_TRQ, 10.0f);
//   delay(10);
//   motorEnable(MOTOR_ID);
//   delay(100);
//   // Move with speed=3 rad/s, accel=5 rad/s²
//   float positions[] = {0.0f, 3.14f, -3.14f, 0.0f};
//   for (float pos : positions) {
//     Serial.printf("PP Move -> %.2f rad\n", pos);
//     motorSetPositionPP(MOTOR_ID, pos, 3.0f, 5.0f);
//     unsigned long t0 = millis();
//     while (millis() - t0 < 5000) {
//       canReceive();
//       if (g_feedback.valid) {
//         printFeedback();
//         if (fabs(g_feedback.position - pos) < 0.08f) {
//           Serial.println("  -> Reached!");
//           break;
//         }
//       }
//       delay(20);
//     }
//     delay(1000);
//   }

//   motorStop(MOTOR_ID);
// }

// // ══════════════════════════════════════════════
// //  MAIN LOOP
// // ══════════════════════════════════════════════
// void loop() {
//   // Continuously receive feedback
//   canReceive();
//   // Print feedback every 200ms
//   static unsigned long lastPrint = 0;
//   if (millis() - lastPrint > 200) {
//     printFeedback();
//     lastPrint = millis();
//   }
//   // ── Serial command handler (simple) ──────────
//   if (Serial.available()) {
//     char c = Serial.read();
//     switch (c) {
//       case 'e': motorEnable(MOTOR_ID);  break;
//       case 's': motorStop(MOTOR_ID);    break;
//       case 'z': motorSetZero(MOTOR_ID); break;
//       case 'c': motorClearFault(MOTOR_ID); break;
//       case '1': demo_OperationMode();   break;
//       case '2': demo_CurrentMode();     break;
//       case '3': demo_VelocityMode();    break;
//       case '4': demo_PositionCSP();     break;
//       case '5': demo_PositionPP();      break;
//       case 'h':
//         Serial.println("Commands: e=Enable  s=Stop  z=SetZero  c=ClearFault");
//         Serial.println("          1=OpMode  2=Current  3=Velocity  4=CSP  5=PP");
//         break;
//     }
//   }
// }

/*








*/
// #include <Arduino.h>
// #include "driver/twai.h"
// #include <string.h>

// #define TWAI_TX_PIN  GPIO_NUM_5
// #define TWAI_RX_PIN  GPIO_NUM_18
// #define MOTOR_ID     0x01
// #define HOST_ID      0xFD

// uint32_t buildExtId(uint8_t commType, uint16_t dataArea2, uint8_t destId) {
//   return ((uint32_t)(commType & 0x1F) << 24)
//        | ((uint32_t)(dataArea2)       << 8)
//        | ((uint32_t)(destId));
// }

// bool canSend(uint32_t extId, uint8_t* data, uint8_t len) {
//   twai_message_t msg;
//   msg.extd             = 1;
//   msg.rtr              = 0;
//   msg.ss               = 0;
//   msg.self             = 0;
//   msg.dlc_non_comp     = 0;
//   msg.identifier       = extId;
//   msg.data_length_code = len;
//   memset(msg.data, 0, 8);
//   if (data) memcpy(msg.data, data, len);
//   esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(10));
//   if (r != ESP_OK) {
//     Serial.printf("[TX ERROR] esp_err=0x%X\n", r);
//     return false;
//   }
//   return true;
// }

// // ─── Ирсэн бүх фреймийг задлан хэвлэх ──
// void printRawFrame(twai_message_t* msg) {
//   uint32_t id = msg->identifier;
  
//   // 29-бит ID задлах
//   uint8_t  commType  = (id >> 24) & 0x1F;
//   uint16_t dataArea2 = (id >>  8) & 0xFFFF;
//   uint8_t  destId    = (id      ) & 0xFF;
//   uint8_t  motorId   = (dataArea2 >> 8) & 0xFF;
//   uint8_t  hostId    = (dataArea2     ) & 0xFF;

//   Serial.println("─────────────────────────────");
//   Serial.printf("RAW ID   : 0x%08X\n", id);
//   Serial.printf("CommType : 0x%02X (%d)\n", commType, commType);
//   Serial.printf("MotorID  : 0x%02X\n", motorId);
//   Serial.printf("HostID   : 0x%02X\n", hostId);
//   Serial.printf("DLC      : %d\n", msg->data_length_code);
//   Serial.printf("DATA     : ");
//   for (int i = 0; i < msg->data_length_code; i++) {
//     Serial.printf("%02X ", msg->data[i]);
//   }
//   Serial.println();

//   // CommType тайлбар
//   switch(commType) {
//     case 0x02: Serial.println("→ Type 2: Motor Feedback"); break;
//     case 0x11: Serial.println("→ Type 17: Param Read Reply"); break;
//     case 0x12: Serial.println("→ Type 18: Param Write Reply"); break;
//     case 0x18: Serial.println("→ Type 24: Active Report"); break;
//     default:   Serial.printf ("→ Type %d: Тодорхойгүй\n", commType);
//   }

//   // Type 2 бол задлах
//   if (commType == 0x02) {
//     uint16_t rawPos  = ((uint16_t)msg->data[0] << 8) | msg->data[1];
//     uint16_t rawVel  = ((uint16_t)msg->data[2] << 8) | msg->data[3];
//     uint16_t rawTrq  = ((uint16_t)msg->data[4] << 8) | msg->data[5];
//     uint16_t rawTemp = ((uint16_t)msg->data[6] << 8) | msg->data[7];

//     float pos  = -12.57f + (float)rawPos  / 65535.0f * 25.14f;
//     float vel  = -33.0f  + (float)rawVel  / 65535.0f * 66.0f;
//     float trq  = -14.0f  + (float)rawTrq  / 65535.0f * 28.0f;
//     float temp = rawTemp / 10.0f;

//     // ID-аас төлөв
//     uint8_t status = (id >> 22) & 0x03;
//     uint8_t fault  = (id >> 16) & 0x3F;

//     Serial.printf("  Pos  : %.4f рад (%.2f°)\n", pos, pos*57.2958f);
//     Serial.printf("  Vel  : %.4f рад/с\n", vel);
//     Serial.printf("  Torq : %.4f N·м\n", trq);
//     Serial.printf("  Temp : %.1f°C\n", temp);
//     Serial.printf("  Mode : %d (0=Reset,1=Cali,2=Run)\n", status);
//     Serial.printf("  Fault: 0x%02X\n", fault);
//   }

//   // Type 17 хариу бол задлах
//   if (commType == 0x11) {
//     uint16_t index;
//     memcpy(&index, &msg->data[0], 2);
//     Serial.printf("  Index: 0x%04X\n", index);
//     Serial.printf("  Val bytes: %02X %02X %02X %02X\n",
//       msg->data[4], msg->data[5], msg->data[6], msg->data[7]);
//     float fval;
//     memcpy(&fval, &msg->data[4], 4);
//     Serial.printf("  As float : %.6f\n", fval);
//     int16_t ival;
//     memcpy(&ival, &msg->data[4], 2);
//     Serial.printf("  As int16 : %d\n", ival);
//   }

//   Serial.println("─────────────────────────────");
// }

// // ─── Type 17 унших ──────────────────────
// void readParam(uint8_t motorId, uint16_t index) {
//   uint32_t extId = buildExtId(0x11,
//                               (uint16_t)HOST_ID << 8,
//                               motorId);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   Serial.printf("[TX] Type17 Read → index=0x%04X, extId=0x%08X\n",
//                 index, extId);
//   canSend(extId, data, 8);
// }

// // ─── Хариу хүлээх + хэвлэх ──────────────
// bool waitAndPrint(uint32_t timeout_ms = 100) {
//   twai_message_t msg;
//   esp_err_t r = twai_receive(&msg, pdMS_TO_TICKS(timeout_ms));
//   if (r == ESP_OK) {
//     printRawFrame(&msg);
//     return true;
//   } else if (r == ESP_ERR_TIMEOUT) {
//     Serial.println("[RX] Timeout — хариу ирсэнгүй!");
//   } else {
//     Serial.printf("[RX ERROR] 0x%X\n", r);
//   }
//   return false;
// }

// void setup() {
//   Serial.begin(115200);
//   delay(500);
//   Serial.println("\n=== ДИБАГ ГОРИМ ===");

//   twai_general_config_t g_config = {
//     .mode           = TWAI_MODE_NORMAL,
//     .tx_io          = TWAI_TX_PIN,
//     .rx_io          = TWAI_RX_PIN,
//     .clkout_io      = TWAI_IO_UNUSED,
//     .bus_off_io     = TWAI_IO_UNUSED,
//     .tx_queue_len   = 10,
//     .rx_queue_len   = 10,
//     .alerts_enabled = TWAI_ALERT_NONE,
//     .clkout_divider = 0
//   };
//   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
//   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//   if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
//     Serial.println("[ERROR] TWAI install failed");
//     while(1);
//   }
//   if (twai_start() != ESP_OK) {
//     Serial.println("[ERROR] TWAI start failed");
//     while(1);
//   }
//   Serial.println("[OK] TWAI 1Mbps бэлэн");
//   delay(300);

//   // ── 1. mechPos унших
//   Serial.println("\n[TEST 1] mechPos (0x3016) унших:");
//   readParam(MOTOR_ID, 0x3016);
//   delay(10);
//   waitAndPrint(100);

//   // ── 2. Vbus унших
//   Serial.println("\n[TEST 2] Vbus (0x701C) унших:");
//   readParam(MOTOR_ID, 0x701C);
//   delay(10);
//   waitAndPrint(100);

//   // ── 3. Буфер дахь бусад фрейм шалгах
//   Serial.println("\n[TEST 3] Буфер дахь бүх фрейм:");
//   for (int i = 0; i < 5; i++) {
//     twai_message_t msg;
//     if (twai_receive(&msg, pdMS_TO_TICKS(20)) == ESP_OK) {
//       printRawFrame(&msg);
//     }
//   }
// }

// void loop() {
//   // Бүх ирж буй фреймийг хэвлэ
//   twai_message_t msg;
//   if (twai_receive(&msg, pdMS_TO_TICKS(10)) == ESP_OK) {
//     printRawFrame(&msg);
//   }

//   if (Serial.available()) {
//     char c = Serial.read();
//     switch(c) {
//       case '1':
//         Serial.println("\n>> mechPos унших:");
//         readParam(MOTOR_ID, 0x3016);
//         break;
//       case '2':
//         Serial.println("\n>> Vbus унших:");
//         readParam(MOTOR_ID, 0x701C);
//         break;
//       case '3':
//         Serial.println("\n>> encoderRaw унших:");
//         readParam(MOTOR_ID, 0x3004);
//         break;
//       case '4':
//         Serial.println("\n>> modPos унших:");
//         readParam(MOTOR_ID, 0x3015);
//         break;
//     }
//   }
// }

/*



*/
// #include <Arduino.h>
// #include "driver/twai.h"
// #include <string.h>

// #define TWAI_TX_PIN  GPIO_NUM_5
// #define TWAI_RX_PIN  GPIO_NUM_18
// #define MOTOR_ID     0x01
// #define HOST_ID      0xFD

// // ── CAN ID бүтээх ──────────────────────
// uint32_t buildExtId(uint8_t commType, 
//                     uint16_t dataArea2, 
//                     uint8_t destId) {
//   return ((uint32_t)(commType & 0x1F) << 24)
//        | ((uint32_t)(dataArea2)       << 8)
//        | ((uint32_t)(destId));
// }

// // ── CAN илгээх ──────────────────────────
// bool canSend(uint32_t extId, uint8_t* data, uint8_t len) {
//   twai_message_t msg;
//   msg.extd             = 1;
//   msg.rtr              = 0;
//   msg.ss               = 0;
//   msg.self             = 0;
//   msg.dlc_non_comp     = 0;
//   msg.identifier       = extId;
//   msg.data_length_code = len;
//   memset(msg.data, 0, 8);
//   if (data) memcpy(msg.data, data, len);
//   return (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);
// }

// // ── Type 18: Параметр бичих ─────────────
// void writeFloat(uint8_t id, uint16_t index, float val) {
//   uint32_t extId = buildExtId(0x12, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   memcpy(&data[4], &val,   4);
//   canSend(extId, data, 8);
// }

// void writeUint8(uint8_t id, uint16_t index, uint8_t val) {
//   uint32_t extId = buildExtId(0x12, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   data[4] = val;
//   canSend(extId, data, 8);
// }

// void writeUint16(uint8_t id, uint16_t index, uint16_t val) {
//   uint32_t extId = buildExtId(0x12, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   memcpy(&data[4], &val,   2);
//   canSend(extId, data, 8);
// }

// // ── Type 17: Параметр унших ─────────────
// void readParam(uint8_t id, uint16_t index) {
//   uint32_t extId = buildExtId(0x11, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   canSend(extId, data, 8);
// }

// // ── Мотор асаах/зогсоох ─────────────────
// void motorEnable(uint8_t id) {
//   uint32_t extId = buildExtId(0x03, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   canSend(extId, data, 8);
// }

// void motorStop(uint8_t id) {
//   uint32_t extId = buildExtId(0x04, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   canSend(extId, data, 8);
// }

// // ── Type 2 feedback задлах ──────────────
// // Мануал 4.1.2: feedback frame бүтэц
// struct MotorState {
//   float   pos;      // рад
//   float   vel;      // рад/с
//   float   torque;   // N·м
//   float   temp;     // °C
//   uint8_t mode;     // 0=Reset, 1=Cali, 2=Run
//   uint8_t fault;    // алдааны бит
//   float   vbus;     // V (Type17-аас)
// };

// MotorState g_state = {0};

// bool parseType2(twai_message_t* msg) {
//   uint8_t ct = (msg->identifier >> 24) & 0x1F;
//   if (ct != 0x02 && ct != 0x18) return false;

//   uint16_t rawPos  = ((uint16_t)msg->data[0] << 8) | msg->data[1];
//   uint16_t rawVel  = ((uint16_t)msg->data[2] << 8) | msg->data[3];
//   uint16_t rawTrq  = ((uint16_t)msg->data[4] << 8) | msg->data[5];
//   uint16_t rawTemp = ((uint16_t)msg->data[6] << 8) | msg->data[7];

//   g_state.pos    = -12.57f + (float)rawPos  / 65535.0f * 25.14f;
//   g_state.vel    = -33.0f  + (float)rawVel  / 65535.0f * 66.0f;
//   g_state.torque = -14.0f  + (float)rawTrq  / 65535.0f * 28.0f;
//   g_state.temp   = rawTemp / 10.0f;

//   // ID-аас горим ба алдаа
//   uint32_t id    = msg->identifier;
//   g_state.mode   = (id >> 22) & 0x03;
//   g_state.fault  = (id >> 16) & 0x3F;
//   return true;
// }

// bool parseType17(twai_message_t* msg) {
//   uint8_t ct = (msg->identifier >> 24) & 0x1F;
//   if (ct != 0x11) return false;

//   uint16_t index;
//   memcpy(&index, &msg->data[0], 2);

//   // Vbus (0x701C) — энэ ажиллаж байгааг баталсан
//   if (index == 0x701C) {
//     float val;
//     memcpy(&val, &msg->data[4], 4);
//     g_state.vbus = val;
//     return true;
//   }
//   return false;
// }

// // ── Ирсэн фреймийг боловсруулах ─────────
// void processRx() {
//   twai_message_t msg;
//   while (twai_receive(&msg, 0) == ESP_OK) {
//     uint8_t ct = (msg.identifier >> 24) & 0x1F;

//     if (ct == 0x02 || ct == 0x18) {
//       parseType2(&msg);
//     } else if (ct == 0x11) {
//       parseType17(&msg);
//     }
//   }
// }

// // ── Мэдээлэл хэвлэх ────────────────────
// void printState() {
//   Serial.println("┌─────────────────────────────┐");
//   Serial.printf ("│ Pos:    %8.4f рад  %6.1f°│\n",
//                   g_state.pos, g_state.pos * 57.2958f);
//   Serial.printf ("│ Vel:    %8.4f рад/с        │\n", g_state.vel);
//   Serial.printf ("│ Torque: %8.4f N·м          │\n", g_state.torque);
//   Serial.printf ("│ Temp:   %8.1f °C           │\n", g_state.temp);
//   Serial.printf ("│ Vbus:   %8.2f V             │\n", g_state.vbus);

//   const char* modeStr[] = {"Reset", "Cali ", "Run  "};
//   Serial.printf ("│ Mode:   %s                   │\n",
//                   g_state.mode < 3 ? modeStr[g_state.mode] : "?????");

//   if (g_state.fault) {
//     Serial.printf("│ FAULT:  0x%02X ← АЛДАА!       │\n", g_state.fault);
//     if (g_state.fault & 0x01) Serial.println("│   bit0: Хэт халсан            │");
//     if (g_state.fault & 0x02) Serial.println("│   bit1: Driver chip алдаа     │");
//     if (g_state.fault & 0x04) Serial.println("│   bit2: Доогуур хүчдэл        │");
//     if (g_state.fault & 0x08) Serial.println("│   bit3: Хэт өндөр хүчдэл      │");
//     if (g_state.fault & 0x20) Serial.println("│   bit5: C фазын гүйдэл        │");
//     if (g_state.fault & 0x80) Serial.println("│   bit7: Encoder тохируулагдаагүй│");
//   } else {
//     Serial.println ("│ Fault:  Байхгүй ✓             │");
//   }
//   Serial.println("└─────────────────────────────┘");
// }

// // ── Active Report асаах (Type 24) ────────
// // Мануал 4.1.11: мотор өөрөө 10мс тутам илгээнэ
// void enableActiveReport(uint8_t id, uint16_t interval_ms = 10) {
//   uint32_t extId = buildExtId(0x18, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01};
//   // data[6] = 0x01 → Active report асаах
//   canSend(extId, data, 8);
//   delay(5);

//   // Интервал тохируулах: index 0x7026
//   // 1=10ms, 2=15ms, 3=20ms ...
//   uint16_t epVal = (interval_ms - 10) / 5 + 1;
//   if (epVal < 1) epVal = 1;
//   writeUint16(id, 0x7026, epVal);

//   Serial.printf("[OK] Active report асаалаа (%dмс)\n", interval_ms);
// }

// // ── Active Report унтраах ────────────────
// void disableActiveReport(uint8_t id) {
//   uint32_t extId = buildExtId(0x18, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00};
//   // data[6] = 0x00 → унтраах
//   canSend(extId, data, 8);
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);
//   Serial.println("\n=== RS00 МОТОР ТЕСТ ===");

//   twai_general_config_t g_config = {
//     .mode           = TWAI_MODE_NORMAL,
//     .tx_io          = TWAI_TX_PIN,
//     .rx_io          = TWAI_RX_PIN,
//     .clkout_io      = TWAI_IO_UNUSED,
//     .bus_off_io     = TWAI_IO_UNUSED,
//     .tx_queue_len   = 20,
//     .rx_queue_len   = 20,
//     .alerts_enabled = TWAI_ALERT_NONE,
//     .clkout_divider = 0
//   };
//   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
//   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//   twai_driver_install(&g_config, &t_config, &f_config);
//   twai_start();
//   Serial.println("[OK] TWAI 1Mbps");

//   delay(300);

//   // ── Vbus унших (энэ ажиллаж байгааг баталсан)
//   Serial.println("\n[1] Vbus унших...");
//   readParam(MOTOR_ID, 0x701C);
//   delay(20);
//   processRx();
//   Serial.printf("Vbus = %.2f V\n", g_state.vbus);

//   // ── Active report асаах
//   Serial.println("\n[2] Active report асааж байна...");
//   enableActiveReport(MOTOR_ID, 10);
//   delay(100);

//   // ── Хэд хэдэн фрейм хүлээн авч шалгах
//   Serial.println("\n[3] Мотор feedback хүлээж байна...");
//   for (int i = 0; i < 5; i++) {
//     twai_message_t msg;
//     if (twai_receive(&msg, pdMS_TO_TICKS(200)) == ESP_OK) {
//       uint8_t ct = (msg.identifier >> 24) & 0x1F;
//       Serial.printf("Фрейм %d: Type=%d ", i+1, ct);
//       if (ct == 0x02 || ct == 0x18) {
//         parseType2(&msg);
//         Serial.printf("Pos=%.3f Vel=%.3f Mode=%d\n",
//                       g_state.pos, g_state.vel, g_state.mode);
//       } else {
//         Serial.printf("ID=0x%08X\n", msg.identifier);
//       }
//     } else {
//       Serial.printf("Фрейм %d: Timeout\n", i+1);
//     }
//   }

//   Serial.println("\n[4] Эхний байдал:");
//   printState();
//   Serial.println("\nКоманд: e=Enable s=Stop r=Report p=Print");
// }

// void loop() {
//   // Ирсэн бүх фреймийг боловсруулах
//   processRx();

//   // 500мс тутам хэвлэх
//   static uint32_t lastPrint = 0;
//   if (millis() - lastPrint > 500) {
//     printState();
//     // Vbus байнга шинэчлэх
//     readParam(MOTOR_ID, 0x701C);
//     lastPrint = millis();
//   }

//   // Serial командууд
//   if (Serial.available()) {
//     char c = Serial.read();
//     switch (c) {
//       case 'e':
//         Serial.println("\n>> Мотор АСААЛАА");
//         motorEnable(MOTOR_ID);
//         break;

//       case 's':
//         Serial.println("\n>> Мотор ЗОГСООЛОО");
//         motorStop(MOTOR_ID);
//         break;

//       case 'r':
//         Serial.println("\n>> Active report асааж байна");
//         enableActiveReport(MOTOR_ID, 10);
//         break;

//       case 'x':
//         Serial.println("\n>> Active report унтраалаа");
//         disableActiveReport(MOTOR_ID);
//         break;

//       case 'p':
//         printState();
//         break;

//       case 'v':
//         readParam(MOTOR_ID, 0x701C);
//         delay(20);
//         processRx();
//         Serial.printf("Vbus = %.2f V\n", g_state.vbus);
//         break;

//       case 'z':
//         // Тэг байрлал тохируулах
//         Serial.println("\n>> Тэг байрлал тохируулж байна");
//         {
//           uint32_t extId = buildExtId(0x06,
//                                       (uint16_t)HOST_ID << 8,
//                                       MOTOR_ID);
//           uint8_t data[8] = {1,0,0,0,0,0,0,0};
//           canSend(extId, data, 8);
//         }
//         break;
//     }
//   }
// }



// #include <Arduino.h>
// #include "driver/twai.h"
// #include <string.h>
// #include <math.h>

// #define TWAI_TX_PIN  GPIO_NUM_5
// #define TWAI_RX_PIN  GPIO_NUM_18

// #define MOTOR_ID     0x01
// #define HOST_ID      0xFD

// // -------------------------
// // Parameter Index
// // -------------------------
// #define IDX_MECH_POS   0x7019   // float, working
// #define IDX_VBUS       0x701C   // float, working
// #define IDX_REPORT_MS  0x7026   // uint16, working for active report interval

// // These 2 may need sniff verification depending on firmware
// #define VEL_MODE_INDEX    0x7005   // assumed: run_mode
// #define VEL_TARGET_INDEX  0x700A   // assumed: target velocity (rad/s)

// // -------------------------
// // Motor state
// // -------------------------
// struct MotorState {
//   // Type2 feedback
//   float pos;       // rad
//   float vel;       // rad/s
//   float torque;    // N.m
//   float temp;      // C
//   uint8_t mode;    // 0=Reset, 1=Cali, 2=Run
//   uint8_t fault;   // fault bits

//   // Type17 parameters
//   float mechPos;   // rad
//   float vbus;      // V
// };

// MotorState g_state = {0};

// // -------------------------
// // Helpers
// // -------------------------
// float radToDeg(float rad) {
//   return rad * 57.2957795f;
// }

// float wrapTo2Pi(float rad) {
//   float x = fmodf(rad, 2.0f * (float)M_PI);
//   if (x < 0) x += 2.0f * (float)M_PI;
//   return x;
// }

// float totalTurns() {
//   return g_state.mechPos / (2.0f * (float)M_PI);
// }

// // -------------------------
// // CAN ID builder
// // -------------------------
// uint32_t buildExtId(uint8_t commType, uint16_t dataArea2, uint8_t destId) {
//   return ((uint32_t)(commType & 0x1F) << 24)
//        | ((uint32_t)(dataArea2) << 8)
//        | ((uint32_t)(destId));
// }

// // -------------------------
// // CAN send
// // -------------------------
// bool canSend(uint32_t extId, const uint8_t* data, uint8_t len) {
//   twai_message_t msg = {};
//   msg.extd = 1;
//   msg.rtr = 0;
//   msg.ss = 0;
//   msg.self = 0;
//   msg.dlc_non_comp = 0;
//   msg.identifier = extId;
//   msg.data_length_code = len;

//   memset(msg.data, 0, sizeof(msg.data));
//   if (data && len > 0) {
//     memcpy(msg.data, data, len > 8 ? 8 : len);
//   }

//   return (twai_transmit(&msg, pdMS_TO_TICKS(100)) == ESP_OK);
// }

// // -------------------------
// // Type17 read parameter
// // -------------------------
// void readParam(uint8_t id, uint16_t index) {
//   uint32_t extId = buildExtId(0x11, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   canSend(extId, data, 8);
// }

// // -------------------------
// // Type18 write parameter
// // -------------------------
// void writeFloat(uint8_t id, uint16_t index, float val) {
//   uint32_t extId = buildExtId(0x12, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   memcpy(&data[4], &val, 4);
//   canSend(extId, data, 8);
// }

// void writeUint8(uint8_t id, uint16_t index, uint8_t val) {
//   uint32_t extId = buildExtId(0x12, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   data[4] = val;
//   canSend(extId, data, 8);
// }

// void writeUint16(uint8_t id, uint16_t index, uint16_t val) {
//   uint32_t extId = buildExtId(0x12, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   memcpy(&data[0], &index, 2);
//   memcpy(&data[4], &val, 2);
//   canSend(extId, data, 8);
// }

// // -------------------------
// // Basic motor commands
// // -------------------------
// void motorEnable(uint8_t id) {
//   uint32_t extId = buildExtId(0x03, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   canSend(extId, data, 8);
// }

// void motorStop(uint8_t id) {
//   uint32_t extId = buildExtId(0x04, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0};
//   canSend(extId, data, 8);
// }

// void motorSetZero(uint8_t id) {
//   uint32_t extId = buildExtId(0x06, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
//   canSend(extId, data, 8);
// }

// // -------------------------
// // Active report control
// // -------------------------
// void enableActiveReport(uint8_t id, uint16_t interval_ms = 10) {
//   uint32_t extId = buildExtId(0x18, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x01};
//   canSend(extId, data, 8);
//   delay(5);

//   uint16_t epVal = 1;
//   if (interval_ms > 10) {
//     epVal = (uint16_t)((interval_ms - 10) / 5 + 1);
//   }

//   writeUint16(id, IDX_REPORT_MS, epVal);
//   Serial.printf("[OK] Active report enabled (%u ms)\n", interval_ms);
// }

// void disableActiveReport(uint8_t id) {
//   uint32_t extId = buildExtId(0x18, (uint16_t)HOST_ID << 8, id);
//   uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x00};
//   canSend(extId, data, 8);
//   Serial.println("[OK] Active report disabled");
// }

// // -------------------------
// // Velocity test helpers
// // -------------------------
// void setVelocityMode(uint8_t id) {
//   writeUint8(id, VEL_MODE_INDEX, 2);
//   delay(10);
// }

// void setVelocityTarget(uint8_t id, float vel_rad_s) {
//   writeFloat(id, VEL_TARGET_INDEX, vel_rad_s);
// }

// // -------------------------
// // Parse Type2 feedback
// // -------------------------
// bool parseType2(twai_message_t* msg) {
//   uint8_t ct = (msg->identifier >> 24) & 0x1F;
//   if (ct != 0x02 && ct != 0x18) return false;

//   uint16_t rawPos  = ((uint16_t)msg->data[0] << 8) | msg->data[1];
//   uint16_t rawVel  = ((uint16_t)msg->data[2] << 8) | msg->data[3];
//   uint16_t rawTrq  = ((uint16_t)msg->data[4] << 8) | msg->data[5];
//   uint16_t rawTemp = ((uint16_t)msg->data[6] << 8) | msg->data[7];

//   // working mapping from your existing code
//   g_state.pos    = -12.57f + ((float)rawPos / 65535.0f) * 25.14f;
//   g_state.vel    = -33.0f  + ((float)rawVel / 65535.0f) * 66.0f;
//   g_state.torque = -14.0f  + ((float)rawTrq / 65535.0f) * 28.0f;
//   g_state.temp   = (float)rawTemp * 0.1f;

//   uint32_t id = msg->identifier;
//   g_state.mode  = (id >> 22) & 0x03;
//   g_state.fault = (id >> 16) & 0xFF;

//   return true;
// }

// // -------------------------
// // Parse Type17 response
// // -------------------------
// bool parseType17(twai_message_t* msg) {
//   uint8_t ct = (msg->identifier >> 24) & 0x1F;
//   if (ct != 0x11) return false;

//   uint16_t index = 0;
//   memcpy(&index, &msg->data[0], 2);

//   switch (index) {
//     case IDX_MECH_POS: {
//       float val = 0.0f;
//       memcpy(&val, &msg->data[4], 4);
//       g_state.mechPos = val;
//       return true;
//     }

//     case IDX_VBUS: {
//       float val = 0.0f;
//       memcpy(&val, &msg->data[4], 4);
//       g_state.vbus = val;
//       return true;
//     }

//     default:
//       return false;
//   }
// }

// // -------------------------
// // RX processing
// // -------------------------
// void processRx() {
//   twai_message_t msg;
//   while (twai_receive(&msg, 0) == ESP_OK) {
//     uint8_t ct = (msg.identifier >> 24) & 0x1F;

//     if (ct == 0x02 || ct == 0x18) {
//       parseType2(&msg);
//     } else if (ct == 0x11) {
//       parseType17(&msg);
//     }
//   }
// }

// // -------------------------
// // Parameter reads
// // -------------------------
// void readPositionParams(uint8_t id) {
//   readParam(id, IDX_MECH_POS);
//   delay(5);
// }

// void readAllParams(uint8_t id) {
//   readParam(id, IDX_MECH_POS);
//   delay(5);
//   readParam(id, IDX_VBUS);
//   delay(5);
// }

// // -------------------------
// // Printing
// // -------------------------
// void printFaults(uint8_t fault) {
//   if (fault == 0) {
//     Serial.println("Fault: none");
//     return;
//   }

//   Serial.printf("Fault: 0x%02X\n", fault);

//   if (fault & 0x01) Serial.println("  bit0: over temperature");
//   if (fault & 0x02) Serial.println("  bit1: driver chip fault");
//   if (fault & 0x04) Serial.println("  bit2: under voltage");
//   if (fault & 0x08) Serial.println("  bit3: over voltage");
//   if (fault & 0x10) Serial.println("  bit4: B phase current fault");
//   if (fault & 0x20) Serial.println("  bit5: C phase current fault");
//   if (fault & 0x40) Serial.println("  bit6: reserved/unknown");
//   if (fault & 0x80) Serial.println("  bit7: encoder uncalibrated");
// }

// void printState() {
//   Serial.println();
//   Serial.println("========== RS00 STATE ==========");
//   Serial.printf("Type2 Pos     : %.6f rad (%.2f deg)\n", g_state.pos, radToDeg(g_state.pos));
//   Serial.printf("Type2 Vel     : %.6f rad/s\n", g_state.vel);
//   Serial.printf("Type2 Torque  : %.6f N.m\n", g_state.torque);
//   Serial.printf("Type2 Temp    : %.1f C\n", g_state.temp);

//   Serial.printf("mechPos       : %.6f rad (%.2f deg)\n", g_state.mechPos, radToDeg(g_state.mechPos));
//   Serial.printf("mechPos 0..2π : %.6f rad (%.2f deg)\n",
//                 wrapTo2Pi(g_state.mechPos),
//                 radToDeg(wrapTo2Pi(g_state.mechPos)));
//   Serial.printf("turns         : %.4f\n", totalTurns());
//   Serial.printf("Vbus          : %.2f V\n", g_state.vbus);

//   const char* modeStr = "Unknown";
//   if (g_state.mode == 0) modeStr = "Reset";
//   else if (g_state.mode == 1) modeStr = "Cali";
//   else if (g_state.mode == 2) modeStr = "Run";

//   Serial.printf("Mode          : %s (%u)\n", modeStr, g_state.mode);
//   printFaults(g_state.fault);
//   Serial.println("================================");
// }

// void readAndPrintPositionOnce() {
//   readPositionParams(MOTOR_ID);
//   delay(30);
//   processRx();

//   Serial.println();
//   Serial.println("---- POSITION READ ----");
//   Serial.printf("mechPos : %.6f rad (%.2f deg)\n", g_state.mechPos, radToDeg(g_state.mechPos));
//   Serial.printf("turns   : %.4f\n", totalTurns());
//   Serial.println("-----------------------");
// }

// // -------------------------
// // Setup
// // -------------------------
// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   Serial.println();
//   Serial.println("=== RS00 FULL POSITION + SLOW VELOCITY TEST ===");

//   twai_general_config_t g_config = {
//     .mode = TWAI_MODE_NORMAL,
//     .tx_io = TWAI_TX_PIN,
//     .rx_io = TWAI_RX_PIN,
//     .clkout_io = TWAI_IO_UNUSED,
//     .bus_off_io = TWAI_IO_UNUSED,
//     .tx_queue_len = 20,
//     .rx_queue_len = 50,
//     .alerts_enabled = TWAI_ALERT_NONE,
//     .clkout_divider = 0
//   };

//   twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
//   twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

//   esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
//   if (err != ESP_OK) {
//     Serial.printf("[ERR] twai_driver_install failed: %d\n", err);
//     while (1) delay(1000);
//   }

//   err = twai_start();
//   if (err != ESP_OK) {
//     Serial.printf("[ERR] twai_start failed: %d\n", err);
//     while (1) delay(1000);
//   }

//   Serial.println("[OK] TWAI started at 1 Mbps");
//   delay(300);

//   Serial.println("[1] Read Vbus...");
//   readParam(MOTOR_ID, IDX_VBUS);
//   delay(20);
//   processRx();
//   Serial.printf("Vbus = %.2f V\n", g_state.vbus);

//   Serial.println("[2] Enable active report...");
//   enableActiveReport(MOTOR_ID, 10);
//   delay(100);

//   Serial.println("[3] Read mechPos...");
//   readAndPrintPositionOnce();

//   Serial.println();
//   Serial.println("Commands:");
//   Serial.println("  e = enable motor");
//   Serial.println("  s = motor stop");
//   Serial.println("  z = set zero");
//   Serial.println("  r = enable active report");
//   Serial.println("  x = disable active report");
//   Serial.println("  p = print full state");
//   Serial.println("  m = read mechPos once");
//   Serial.println("  v = read Vbus");
//   Serial.println("  1 = set velocity mode");
//   Serial.println("  2 = +0.10 rad/s");
//   Serial.println("  3 = -0.10 rad/s");
//   Serial.println("  0 = 0 rad/s");
// }

// // -------------------------
// // Loop
// // -------------------------
// void loop() {
//   processRx();

//   static uint32_t lastPoll = 0;
//   static uint32_t lastPrint = 0;

//   if (millis() - lastPoll >= 500) {
//     readAllParams(MOTOR_ID);
//     lastPoll = millis();
//   }

//   if (millis() - lastPrint >= 1000) {
//     delay(20);
//     processRx();
//     printState();
//     lastPrint = millis();
//   }

//   if (Serial.available()) {
//     char c = Serial.read();

//     switch (c) {
//       case 'e':
//         Serial.println(">> Enable motor");
//         motorEnable(MOTOR_ID);
//         break;

//       case 's':
//         Serial.println(">> Stop motor");
//         motorStop(MOTOR_ID);
//         break;

//       case 'z':
//         Serial.println(">> Set zero");
//         motorSetZero(MOTOR_ID);
//         break;

//       case 'r':
//         Serial.println(">> Enable active report");
//         enableActiveReport(MOTOR_ID, 10);
//         break;

//       case 'x':
//         Serial.println(">> Disable active report");
//         disableActiveReport(MOTOR_ID);
//         break;

//       case 'p':
//         delay(20);
//         processRx();
//         printState();
//         break;

//       case 'm':
//         Serial.println(">> Read mechPos");
//         readAndPrintPositionOnce();
//         break;

//       case 'v':
//         Serial.println(">> Read Vbus");
//         readParam(MOTOR_ID, IDX_VBUS);
//         delay(20);
//         processRx();
//         Serial.printf("Vbus = %.2f V\n", g_state.vbus);
//         break;

//       case '1':
//         Serial.println(">> Set velocity mode");
//         setVelocityMode(MOTOR_ID);
//         break;

//       case '2':
//         Serial.println(">> Velocity +0.10 rad/s");
//         setVelocityTarget(MOTOR_ID, 0.10f);
//         break;

//       case '3':
//         Serial.println(">> Velocity -0.10 rad/s");
//         setVelocityTarget(MOTOR_ID, -0.10f);
//         break;

//       case '0':
//         Serial.println(">> Velocity 0.00 rad/s");
//         setVelocityTarget(MOTOR_ID, 0.0f);
//         break;
//     }
//   }
// }

#include <Arduino.h>
#include "driver/twai.h"
#include <string.h>
#include <math.h>

// =====================================================
// PIN / BUS CONFIG
// =====================================================
#define TWAI_TX_PIN  GPIO_NUM_5
#define TWAI_RX_PIN  GPIO_NUM_18

#define MOTOR_ID     0x01
#define HOST_ID      0xFD

// =====================================================
// LIMITS / SCALING
// =====================================================
#define P_MIN   -12.57f
#define P_MAX    12.57f
#define V_MIN   -33.0f
#define V_MAX    33.0f
#define KP_MIN   0.0f
#define KP_MAX   500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN   -14.0f
#define T_MAX    14.0f

#define POLE_PAIRS  14

// =====================================================
// COMM TYPES
// =====================================================
#define CMD_GET_ID          0x00
#define CMD_MOTION_CTRL     0x01
#define CMD_MOTOR_FEEDBACK  0x02
#define CMD_ENABLE          0x03
#define CMD_STOP            0x04
#define CMD_SET_ZERO        0x06
#define CMD_SET_CAN_ID      0x07
#define CMD_PARAM_READ      0x11
#define CMD_PARAM_WRITE     0x12
#define CMD_FAULT_FEEDBACK  0x15
#define CMD_SAVE_PARAMS     0x16
#define CMD_ACTIVE_REPORT   0x18
#define CMD_SET_PROTOCOL    0x19

// =====================================================
// PARAM INDEX
// =====================================================
#define IDX_RUN_MODE      0x7005
#define IDX_IQ_REF        0x7006
#define IDX_SPD_REF       0x700A
#define IDX_LIMIT_TRQ     0x700B
#define IDX_LOC_REF       0x7016
#define IDX_LIMIT_SPD     0x7017
#define IDX_LIMIT_CUR     0x7018
#define IDX_MECH_POS      0x7019
#define IDX_IQFBK         0x701A   // Iq feedback (actual current)
#define IDX_VBUS          0x701C
#define IDX_LOC_KP        0x701E
#define IDX_SPD_KP        0x701F
#define IDX_SPD_KI        0x7020
#define IDX_SPD_FILT      0x7021
#define IDX_ACC_RAD       0x7022
#define IDX_VEL_MAX       0x7024
#define IDX_ACC_SET       0x7025
#define IDX_REPORT_TIME   0x7026
#define IDX_CAN_TIMEOUT   0x7028
#define IDX_ZERO_STA      0x7029
#define IDX_DAMPER        0x702A
#define IDX_ADD_OFFSET    0x702B

// =====================================================
// RUN MODES
// =====================================================
#define MODE_OPERATION  0
#define MODE_PP_POS     1
#define MODE_VELOCITY   2
#define MODE_CURRENT    3
#define MODE_CSP_POS    5

// =====================================================
// STATE
// =====================================================
struct MotorState {
  // Type2 feedback
  float pos;
  float vel;
  float torque;
  float temp;
  uint8_t mode;
  uint8_t fault;

  // Params
  float mechPos;
  float vbus;
  float iqFbk;
  float locKp;
  float spdKp;
  float spdKi;
  float limitTrq;
  float limitSpd;
  float limitCur;
  uint8_t runMode;
  uint8_t zeroSta;

  bool type2Valid;
};

MotorState g_state = {};

// velocity increment for q/w keys
static float g_velTarget = 0.0f;

// =====================================================
// HELPERS
// =====================================================
float radToDeg(float r)  { return r * 57.2957795f; }
float degToRad(float d)  { return d * 0.01745329f; }

float wrapTo2Pi(float r) {
  float x = fmodf(r, 2.0f * (float)M_PI);
  if (x < 0) x += 2.0f * (float)M_PI;
  return x;
}

float totalTurns() { return g_state.mechPos / (2.0f * (float)M_PI); }

uint16_t floatToUint16(float x, float mn, float mx) {
  if (x > mx) x = mx;
  if (x < mn) x = mn;
  return (uint16_t)((x - mn) / (mx - mn) * 65535.0f);
}
float uint16ToFloat(uint16_t x, float mn, float mx) {
  return mn + (float)x / 65535.0f * (mx - mn);
}

// =====================================================
// TWAI RECOVERY
// =====================================================
void recoverTWAI() {
  twai_stop();
  delay(100);
  twai_start();
  Serial.println("[TWAI] Recovered from Bus-Off");
}

// =====================================================
// CAN SEND
// =====================================================
uint32_t buildExtId(uint8_t ct, uint16_t da2, uint8_t dst) {
  return ((uint32_t)(ct & 0x1F) << 24)
       | ((uint32_t)(da2 & 0xFFFF) << 8)
       | ((uint32_t)(dst & 0xFF));
}

bool canSend(uint32_t extId, const uint8_t* data, uint8_t len) {
  twai_message_t msg = {};
  msg.extd = 1;
  msg.identifier = extId;
  msg.data_length_code = len;
  memset(msg.data, 0, 8);
  if (data && len > 0) memcpy(msg.data, data, len > 8 ? 8 : len);

  esp_err_t r = twai_transmit(&msg, pdMS_TO_TICKS(50));
  if (r != ESP_OK) {
    Serial.printf("[TX ERR] 0x%X\n", r);
    if (r == ESP_ERR_INVALID_STATE) recoverTWAI();
    return false;
  }
  return true;
}

// =====================================================
// PARAM READ / WRITE
// =====================================================
void readParam(uint8_t id, uint16_t idx) {
  uint32_t extId = buildExtId(CMD_PARAM_READ, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  memcpy(&d[0], &idx, 2);
  canSend(extId, d, 8);
}

void writeFloat(uint8_t id, uint16_t idx, float v) {
  uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  memcpy(&d[0], &idx, 2);
  memcpy(&d[4], &v, 4);
  canSend(extId, d, 8);
}

void writeUint8(uint8_t id, uint16_t idx, uint8_t v) {
  uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  memcpy(&d[0], &idx, 2);
  d[4] = v;
  canSend(extId, d, 8);
}

void writeUint16(uint8_t id, uint16_t idx, uint16_t v) {
  uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  memcpy(&d[0], &idx, 2);
  memcpy(&d[4], &v, 2);
  canSend(extId, d, 8);
}

void writeUint32(uint8_t id, uint16_t idx, uint32_t v) {
  uint32_t extId = buildExtId(CMD_PARAM_WRITE, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  memcpy(&d[0], &idx, 2);
  memcpy(&d[4], &v, 4);
  canSend(extId, d, 8);
}

// =====================================================
// BASIC COMMANDS
// =====================================================
void motorEnable(uint8_t id) {
  uint32_t extId = buildExtId(CMD_ENABLE, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  canSend(extId, d, 8);
  Serial.println("[CMD] Enable");
}

void motorStop(uint8_t id) {
  uint32_t extId = buildExtId(CMD_STOP, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {};
  canSend(extId, d, 8);
  Serial.println("[CMD] Stop");
}

void motorClearFault(uint8_t id) {
  uint32_t extId = buildExtId(CMD_STOP, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {1};
  canSend(extId, d, 8);
  Serial.println("[CMD] Clear fault");
}

void motorSetZero(uint8_t id) {
  uint32_t extId = buildExtId(CMD_SET_ZERO, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {1};
  canSend(extId, d, 8);
  Serial.println("[CMD] Set zero");
}

void motorSaveParams(uint8_t id) {
  uint32_t extId = buildExtId(CMD_SAVE_PARAMS, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {1,2,3,4,5,6,7,8};
  canSend(extId, d, 8);
  Serial.println("[CMD] Save params");
}

// =====================================================
// ACTIVE REPORT
// =====================================================
void enableActiveReport(uint8_t id, uint16_t ms = 10) {
  uint32_t extId = buildExtId(CMD_ACTIVE_REPORT, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x01};
  canSend(extId, d, 8);
  delay(5);
  uint16_t ep = (ms <= 10) ? 1 : (uint16_t)((ms - 10) / 5 + 1);
  writeUint16(id, IDX_REPORT_TIME, ep);
  Serial.printf("[CMD] Active report ON (%u ms)\n", ms);
}

void disableActiveReport(uint8_t id) {
  uint32_t extId = buildExtId(CMD_ACTIVE_REPORT, (uint16_t)HOST_ID << 8, id);
  uint8_t d[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x00};
  canSend(extId, d, 8);
  Serial.println("[CMD] Active report OFF");
}

// =====================================================
// CONTROL HELPERS
// =====================================================
void motorSetMode(uint8_t id, uint8_t mode) {
  writeUint8(id, IDX_RUN_MODE, mode);
  delay(5);
}

void motorSetCurrent(uint8_t id, float iq)    { writeFloat(id, IDX_IQ_REF,    iq); }
void motorSetVelocity(uint8_t id, float v)    { writeFloat(id, IDX_SPD_REF,   v);  }
void motorSetPositionCSP(uint8_t id, float p) { writeFloat(id, IDX_LOC_REF,   p);  }
void motorSetCurrentLimit(uint8_t id, float c){ writeFloat(id, IDX_LIMIT_CUR, c);  }
void motorSetVelocityAccel(uint8_t id, float a){ writeFloat(id, IDX_ACC_RAD,  a);  }
void motorSetSpeedLimitCSP(uint8_t id, float s){ writeFloat(id, IDX_LIMIT_SPD,s);  }

void motorSetPositionPP(uint8_t id, float pos, float spd, float acc) {
  writeFloat(id, IDX_VEL_MAX, spd); delay(2);
  writeFloat(id, IDX_ACC_SET, acc); delay(2);
  writeFloat(id, IDX_LOC_REF, pos);
}

void motorOperationControl(uint8_t id,
                           float torque_ff, float position,
                           float velocity,  float kp, float kd) {
  uint16_t tq_raw = floatToUint16(torque_ff, T_MIN, T_MAX);
  uint32_t extId  = buildExtId(CMD_MOTION_CTRL, tq_raw, id);
  uint8_t d[8];
  uint16_t pos_r = floatToUint16(position, P_MIN, P_MAX);
  uint16_t vel_r = floatToUint16(velocity, V_MIN, V_MAX);
  uint16_t kp_r  = floatToUint16(kp, KP_MIN, KP_MAX);
  uint16_t kd_r  = floatToUint16(kd, KD_MIN, KD_MAX);
  d[0]=pos_r>>8; d[1]=pos_r&0xFF;
  d[2]=vel_r>>8; d[3]=vel_r&0xFF;
  d[4]=kp_r >>8; d[5]=kp_r &0xFF;
  d[6]=kd_r >>8; d[7]=kd_r &0xFF;
  canSend(extId, d, 8);
}

// =====================================================
// PARSE RX
// =====================================================
bool parseType2(twai_message_t* msg) {
  uint8_t ct = (msg->identifier >> 24) & 0x1F;
  if (ct != CMD_MOTOR_FEEDBACK && ct != CMD_ACTIVE_REPORT) return false;

  uint16_t rPos  = ((uint16_t)msg->data[0]<<8)|msg->data[1];
  uint16_t rVel  = ((uint16_t)msg->data[2]<<8)|msg->data[3];
  uint16_t rTrq  = ((uint16_t)msg->data[4]<<8)|msg->data[5];
  uint16_t rTemp = ((uint16_t)msg->data[6]<<8)|msg->data[7];

  g_state.pos    = uint16ToFloat(rPos,  P_MIN, P_MAX);
  g_state.vel    = uint16ToFloat(rVel,  V_MIN, V_MAX);
  g_state.torque = uint16ToFloat(rTrq,  T_MIN, T_MAX);
  g_state.temp   = (float)rTemp / 10.0f;
  g_state.mode   = (msg->identifier >> 22) & 0x03;
  g_state.fault  = (msg->identifier >> 16) & 0xFF;
  g_state.type2Valid = true;
  return true;
}

bool parseType17(twai_message_t* msg) {
  uint8_t ct = (msg->identifier >> 24) & 0x1F;
  if (ct != CMD_PARAM_READ) return false;

  uint16_t idx = 0;
  memcpy(&idx, &msg->data[0], 2);
  float fv = 0.0f; memcpy(&fv, &msg->data[4], 4);
  uint8_t u8v = msg->data[4];

  switch (idx) {
    case IDX_MECH_POS:   g_state.mechPos  = fv;  break;
    case IDX_VBUS:       g_state.vbus     = fv;  break;
    case IDX_IQFBK:      g_state.iqFbk    = fv;  break;
    case IDX_LOC_KP:     g_state.locKp    = fv;  break;
    case IDX_SPD_KP:     g_state.spdKp    = fv;  break;
    case IDX_SPD_KI:     g_state.spdKi    = fv;  break;
    case IDX_LIMIT_TRQ:  g_state.limitTrq = fv;  break;
    case IDX_LIMIT_SPD:  g_state.limitSpd = fv;  break;
    case IDX_LIMIT_CUR:  g_state.limitCur = fv;  break;
    case IDX_RUN_MODE:   g_state.runMode  = u8v; break;
    case IDX_ZERO_STA:   g_state.zeroSta  = u8v; break;
    default: return false;
  }
  return true;
}

void processRx() {
  twai_message_t msg;
  while (twai_receive(&msg, 0) == ESP_OK) {
    uint8_t ct = (msg.identifier >> 24) & 0x1F;
    if (ct == CMD_MOTOR_FEEDBACK || ct == CMD_ACTIVE_REPORT)
      parseType2(&msg);
    else if (ct == CMD_PARAM_READ)
      parseType17(&msg);
  }
}

// =====================================================
// READ ALL PARAMS
// =====================================================
void readAllParams(uint8_t id) {
  readParam(id, IDX_MECH_POS);   delay(8);
  readParam(id, IDX_VBUS);       delay(8);
  readParam(id, IDX_IQFBK);      delay(8);
  readParam(id, IDX_LOC_KP);     delay(8);
  readParam(id, IDX_SPD_KP);     delay(8);
  readParam(id, IDX_SPD_KI);     delay(8);
  readParam(id, IDX_LIMIT_TRQ);  delay(8);
  readParam(id, IDX_LIMIT_SPD);  delay(8);
  readParam(id, IDX_LIMIT_CUR);  delay(8);
  readParam(id, IDX_RUN_MODE);   delay(8);
  readParam(id, IDX_ZERO_STA);   delay(8);
  processRx();
}

// =====================================================
// PRINT
// =====================================================
void printFaults(uint8_t fault) {
  if (!fault) { Serial.println("Fault         : none"); return; }
  Serial.printf("Fault         : 0x%02X\n", fault);
  if (fault & 0x01) Serial.println("  [!] Over temperature");
  if (fault & 0x02) Serial.println("  [!] Driver chip fault");
  if (fault & 0x04) Serial.println("  [!] Under voltage");
  if (fault & 0x08) Serial.println("  [!] Over voltage");
  if (fault & 0x10) Serial.println("  [!] B phase current fault");
  if (fault & 0x20) Serial.println("  [!] C phase current fault");
  if (fault & 0x40) Serial.println("  [!] Reserved");
  if (fault & 0x80) Serial.println("  [!] Encoder uncalibrated");
}

const char* modeStr(uint8_t m) {
  if (m == 0) return "Reset";
  if (m == 1) return "Cali";
  if (m == 2) return "Run";
  return "Unknown";
}

const char* runModeStr(uint8_t m) {
  if (m == MODE_OPERATION) return "Operation/MIT";
  if (m == MODE_PP_POS)    return "PP Position";
  if (m == MODE_VELOCITY)  return "Velocity";
  if (m == MODE_CURRENT)   return "Current";
  if (m == MODE_CSP_POS)   return "CSP Position";
  return "Unknown";
}

void printFullState() {
  // Encoder derived
  float mechDeg     = radToDeg(g_state.mechPos);
  float wrapped     = wrapTo2Pi(g_state.mechPos);
  float wrappedDeg  = radToDeg(wrapped);
  float turns       = totalTurns();
  float elecAngle   = fmodf(g_state.mechPos * POLE_PAIRS, 2.0f*(float)M_PI);
  if (elecAngle < 0) elecAngle += 2.0f*(float)M_PI;

  Serial.println();
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║         RS00  FULL STATE              ║");
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  [ ENCODER / POSITION ]               ║");
  Serial.printf( "║  mechPos       : %8.4f rad  %7.2f°  ║\n", g_state.mechPos, mechDeg);
  Serial.printf( "║  mechPos 0-2pi : %8.4f rad  %7.2f°  ║\n", wrapped, wrappedDeg);
  Serial.printf( "║  Total turns   : %8.4f rev             ║\n", turns);
  Serial.printf( "║  Elec angle    : %8.4f rad  %7.2f°  ║\n", elecAngle, radToDeg(elecAngle));
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  [ TYPE2 FEEDBACK ]                   ║");
  Serial.printf( "║  Pos           : %8.4f rad  %7.2f°  ║\n", g_state.pos, radToDeg(g_state.pos));
  Serial.printf( "║  Velocity      : %8.4f rad/s           ║\n", g_state.vel);
  Serial.printf( "║  Torque        : %8.4f N.m             ║\n", g_state.torque);
  Serial.printf( "║  Temperature   : %8.1f C               ║\n", g_state.temp);
  Serial.printf( "║  Iq feedback   : %8.4f A               ║\n", g_state.iqFbk);
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  [ BUS / POWER ]                      ║");
  Serial.printf( "║  Vbus          : %8.2f V               ║\n", g_state.vbus);
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  [ CONTROL PARAMS ]                   ║");
  Serial.printf( "║  Run mode      : %s (%u)        ║\n", runModeStr(g_state.runMode), g_state.runMode);
  Serial.printf( "║  Limit torque  : %8.2f N.m             ║\n", g_state.limitTrq);
  Serial.printf( "║  Limit speed   : %8.2f rad/s           ║\n", g_state.limitSpd);
  Serial.printf( "║  Limit current : %8.2f A               ║\n", g_state.limitCur);
  Serial.printf( "║  Loc Kp        : %8.2f                 ║\n", g_state.locKp);
  Serial.printf( "║  Spd Kp        : %8.2f                 ║\n", g_state.spdKp);
  Serial.printf( "║  Spd Ki        : %8.2f                 ║\n", g_state.spdKi);
  Serial.println("╠══════════════════════════════════════╣");
  Serial.println("║  [ STATUS ]                           ║");
  Serial.printf( "║  Mode          : %s (%u)              ║\n", modeStr(g_state.mode), g_state.mode);
  Serial.printf( "║  Zero status   : %u                    ║\n", g_state.zeroSta);
  printFaults(g_state.fault);
  Serial.println("╚══════════════════════════════════════╝");
}

// =====================================================
// DEMOS
// =====================================================
void demo_OperationMode() {
  Serial.println("\n>>> DEMO: Operation / MIT Mode");
  motorEnable(MOTOR_ID);
  delay(100);

  Serial.println("  Phase 1: vel=1 rad/s, kd=1 (spin freely)");
  for (int i = 0; i < 50; i++) {
    motorOperationControl(MOTOR_ID, 0.0f, 0.0f, 1.0f, 0.0f, 1.0f);
    processRx();
    delay(20);
  }

  Serial.println("  Phase 2: pos=3.0 rad, kp=10, kd=1");
  for (int i = 0; i < 100; i++) {
    motorOperationControl(MOTOR_ID, 0.0f, 3.0f, 0.0f, 10.0f, 1.0f);
    processRx();
    if (i % 20 == 0)
      Serial.printf("    pos=%.3f  vel=%.3f\n", g_state.pos, g_state.vel);
    delay(20);
  }

  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

void demo_CurrentMode() {
  Serial.println("\n>>> DEMO: Current Mode");
  motorSetMode(MOTOR_ID, MODE_CURRENT);
  delay(20);
  motorEnable(MOTOR_ID);
  delay(100);

  Serial.println("  Ramp up 0→2A");
  for (float iq = 0.0f; iq <= 2.0f; iq += 0.1f) {
    motorSetCurrent(MOTOR_ID, iq);
    processRx();
    Serial.printf("    Iq_cmd=%.1f  Iq_fbk=%.3f  vel=%.3f\n",
                  iq, g_state.iqFbk, g_state.vel);
    delay(80);
  }

  Serial.println("  Hold 2A for 1s");
  for (int i = 0; i < 20; i++) {
    motorSetCurrent(MOTOR_ID, 2.0f);
    processRx();
    delay(50);
  }

  Serial.println("  Ramp down 2→0A");
  for (float iq = 2.0f; iq >= 0.0f; iq -= 0.1f) {
    motorSetCurrent(MOTOR_ID, iq);
    processRx();
    delay(80);
  }

  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

void demo_VelocityMode() {
  Serial.println("\n>>> DEMO: Velocity Mode");
  motorSetMode(MOTOR_ID, MODE_VELOCITY);
  delay(20);
  motorSetCurrentLimit(MOTOR_ID, 10.0f);
  motorSetVelocityAccel(MOTOR_ID, 5.0f);
  delay(20);
  motorEnable(MOTOR_ID);
  delay(100);

  float speeds[] = {3.0f, -3.0f, 6.0f, -6.0f, 0.0f};
  for (float spd : speeds) {
    Serial.printf("  Target vel = %.1f rad/s\n", spd);
    motorSetVelocity(MOTOR_ID, spd);
    for (int i = 0; i < 60; i++) {
      processRx();
      if (i % 15 == 0)
        Serial.printf("    vel=%.3f  torque=%.3f  pos=%.3f\n",
                      g_state.vel, g_state.torque, g_state.pos);
      delay(20);
    }
  }

  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

void demo_PositionCSP() {
  Serial.println("\n>>> DEMO: Position CSP Mode");
  motorSetMode(MOTOR_ID, MODE_CSP_POS);
  delay(20);
  motorSetSpeedLimitCSP(MOTOR_ID, 5.0f);
  writeFloat(MOTOR_ID, IDX_LIMIT_TRQ, 10.0f);
  delay(20);
  motorEnable(MOTOR_ID);
  delay(100);

  float targets[] = {0.0f, 1.57f, 3.14f, 4.71f, 0.0f};
  for (float t : targets) {
    Serial.printf("  Move CSP -> %.2f rad (%.1f deg)\n", t, radToDeg(t));
    motorSetPositionCSP(MOTOR_ID, t);
    unsigned long t0 = millis();
    while (millis() - t0 < 3000) {
      processRx();
      delay(20);
    }
    Serial.printf("    arrived: pos=%.4f  err=%.4f\n",
                  g_state.pos, t - g_state.pos);
    delay(300);
  }

  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

void demo_PositionPP() {
  Serial.println("\n>>> DEMO: Position PP Mode");
  motorSetMode(MOTOR_ID, MODE_PP_POS);
  delay(20);
  writeFloat(MOTOR_ID, IDX_LIMIT_TRQ, 10.0f);
  delay(10);
  motorEnable(MOTOR_ID);
  delay(100);

  float positions[] = {0.0f, 3.14f, -3.14f, 1.57f, 0.0f};
  for (float p : positions) {
    Serial.printf("  Move PP -> %.2f rad\n", p);
    motorSetPositionPP(MOTOR_ID, p, 3.0f, 5.0f);
    unsigned long t0 = millis();
    while (millis() - t0 < 5000) {
      processRx();
      delay(20);
    }
    Serial.printf("    arrived: pos=%.4f  err=%.4f\n",
                  g_state.pos, p - g_state.pos);
    delay(500);
  }

  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

void demo_SinusoidalVelocity() {
  Serial.println("\n>>> DEMO: Sinusoidal Velocity");
  motorSetMode(MOTOR_ID, MODE_VELOCITY);
  delay(20);
  motorSetCurrentLimit(MOTOR_ID, 10.0f);
  motorSetVelocityAccel(MOTOR_ID, 20.0f);
  delay(20);
  motorEnable(MOTOR_ID);
  delay(100);

  float amp  = 5.0f;   // rad/s amplitude
  float freq = 0.5f;   // Hz
  unsigned long t0 = millis();
  unsigned long dur = 8000;

  while (millis() - t0 < dur) {
    float t   = (millis() - t0) / 1000.0f;
    float vel = amp * sinf(2.0f * (float)M_PI * freq * t);
    motorSetVelocity(MOTOR_ID, vel);
    processRx();
    Serial.printf("t=%.2f  cmd=%.3f  fbk=%.3f  pos=%.3f\n",
                  t, vel, g_state.vel, g_state.pos);
    delay(20);
  }

  motorSetVelocity(MOTOR_ID, 0.0f);
  delay(500);
  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

void demo_EncoderScan() {
  Serial.println("\n>>> DEMO: Encoder scan (rotate slowly, read all)");
  motorSetMode(MOTOR_ID, MODE_VELOCITY);
  delay(20);
  motorSetCurrentLimit(MOTOR_ID, 5.0f);
  motorSetVelocityAccel(MOTOR_ID, 2.0f);
  delay(20);
  motorEnable(MOTOR_ID);
  delay(100);
  motorSetVelocity(MOTOR_ID, 1.0f);

  for (int i = 0; i < 100; i++) {
    readAllParams(MOTOR_ID);
    processRx();

    float elec = fmodf(g_state.mechPos * POLE_PAIRS, 2.0f*(float)M_PI);
    if (elec < 0) elec += 2.0f*(float)M_PI;

    Serial.printf("[%3d] mechPos=%7.4f rad | wrap=%6.4f | turns=%6.3f | "
                  "elec=%6.4f | vel=%6.3f | Iq=%6.3f\n",
                  i,
                  g_state.mechPos, wrapTo2Pi(g_state.mechPos),
                  totalTurns(), elec,
                  g_state.vel, g_state.iqFbk);
    delay(50);
  }

  motorSetVelocity(MOTOR_ID, 0.0f);
  delay(300);
  motorStop(MOTOR_ID);
  Serial.println(">>> DEMO done");
}

// =====================================================
// PRINT MENU
// =====================================================
void printMenu() {
  Serial.println();
  Serial.println("========= COMMANDS =========");
  Serial.println(" e  = enable");
  Serial.println(" s  = stop");
  Serial.println(" c  = clear fault");
  Serial.println(" z  = set zero");
  Serial.println(" S  = save params");
  Serial.println(" a  = active report ON");
  Serial.println(" x  = active report OFF");
  Serial.println(" p  = print full state");
  Serial.println(" r  = read all params now");
  Serial.println(" h  = show this menu");
  Serial.println("--- Velocity jog ---");
  Serial.println(" q  = +0.5 rad/s");
  Serial.println(" w  = -0.5 rad/s");
  Serial.println(" 0  = stop velocity");
  Serial.println("--- Demos ---");
  Serial.println(" 1  = Operation/MIT demo");
  Serial.println(" 2  = Current mode demo");
  Serial.println(" 3  = Velocity mode demo");
  Serial.println(" 4  = CSP position demo");
  Serial.println(" 5  = PP  position demo");
  Serial.println(" 6  = Sinusoidal velocity demo");
  Serial.println(" 7  = Encoder scan demo");
  Serial.println("============================");
}

// =====================================================
// SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n=== RS00 PERFECT DEMO ===");

  twai_general_config_t g_cfg = {
    .mode           = TWAI_MODE_NORMAL,
    .tx_io          = TWAI_TX_PIN,
    .rx_io          = TWAI_RX_PIN,
    .clkout_io      = TWAI_IO_UNUSED,
    .bus_off_io     = TWAI_IO_UNUSED,
    .tx_queue_len   = 20,
    .rx_queue_len   = 50,
    .alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED,
    .clkout_divider = 0
  };
  twai_timing_config_t t_cfg = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_cfg, &t_cfg, &f_cfg) != ESP_OK) {
    Serial.println("[ERR] TWAI install failed"); while(1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[ERR] TWAI start failed");  while(1) delay(1000);
  }
  Serial.println("[OK] TWAI @ 1Mbps");
  delay(300);

  readParam(MOTOR_ID, IDX_VBUS); delay(30); processRx();
  Serial.printf("[INIT] Vbus = %.2f V\n", g_state.vbus);

  enableActiveReport(MOTOR_ID, 10);
  delay(100);

  readAllParams(MOTOR_ID);
  delay(50);
  processRx();
  printFullState();
  printMenu();
}

// =====================================================
// LOOP
// =====================================================
void loop() {
  // TWAI alert check
  uint32_t alerts = 0;
  if (twai_read_alerts(&alerts, 0) == ESP_OK) {
    if (alerts & TWAI_ALERT_BUS_OFF) {
      Serial.println("[ALERT] Bus-Off! Recovering...");
      recoverTWAI();
    }
  }

  processRx();

  static uint32_t lastPoll  = 0;
  static uint32_t lastPrint = 0;

  if (millis() - lastPoll >= 200) {
    readAllParams(MOTOR_ID);
    lastPoll = millis();
  }

  if (millis() - lastPrint >= 2000) {
    processRx();
    printFullState();
    lastPrint = millis();
  }

  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'e': motorEnable(MOTOR_ID); break;
      case 's': motorStop(MOTOR_ID);   break;
      case 'c': motorClearFault(MOTOR_ID); break;
      case 'z': motorSetZero(MOTOR_ID); break;
      case 'S': motorSaveParams(MOTOR_ID); break;
      case 'a': enableActiveReport(MOTOR_ID, 10); break;
      case 'x': disableActiveReport(MOTOR_ID); break;
      case 'h': printMenu(); break;

      case 'p':
        readAllParams(MOTOR_ID);
        delay(50); processRx();
        printFullState();
        break;

      case 'r':
        readAllParams(MOTOR_ID);
        delay(50); processRx();
        Serial.println("[OK] Params refreshed");
        break;

      case 'q':
        g_velTarget += 0.5f;
        motorSetMode(MOTOR_ID, MODE_VELOCITY);
        delay(10); motorEnable(MOTOR_ID); delay(20);
        motorSetCurrentLimit(MOTOR_ID, 5.0f);
        motorSetVelocityAccel(MOTOR_ID, 3.0f);
        delay(10);
        motorSetVelocity(MOTOR_ID, g_velTarget);
        Serial.printf("[JOG] vel = %.2f rad/s\n", g_velTarget);
        break;

      case 'w':
        g_velTarget -= 0.5f;
        motorSetMode(MOTOR_ID, MODE_VELOCITY);
        delay(10); motorEnable(MOTOR_ID); delay(20);
        motorSetCurrentLimit(MOTOR_ID, 5.0f);
        motorSetVelocityAccel(MOTOR_ID, 3.0f);
        delay(10);
        motorSetVelocity(MOTOR_ID, g_velTarget);
        Serial.printf("[JOG] vel = %.2f rad/s\n", g_velTarget);
        break;

      case '0':
        g_velTarget = 0.0f;
        motorSetVelocity(MOTOR_ID, 0.0f);
        Serial.println("[JOG] velocity stop");
        break;
                                          
      case '1': demo_OperationMode();       break;
      case '2': demo_CurrentMode();         break;
      case '3': demo_VelocityMode();        break;
      case '4': demo_PositionCSP();         break;
      case '5': demo_PositionPP();          break;
      case '6': demo_SinusoidalVelocity();  break;
      case '7': demo_EncoderScan();         break;
    }
  }
}