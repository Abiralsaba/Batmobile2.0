/*
  Serial commands:
    x
      Enable motors

    base,shoulder,elbow,roll,gripper1,gripper2,servo

  Example:
    x
    0,0,0,0,0,0,0

  Gripper command:
    -1 = one direction
     0 = stop/hold
     1 = other direction

  CAN1 on Teensy 4.0:
    CTX1 = pin 22
    CRX1 = pin 23
*/

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <Servo.h>

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// ===================== USER SETTINGS =====================
#define CAN_BITRATE 1000000

const float CONTROL_PERIOD_S    = 0.005f;   // 200 Hz
const float DT                  = CONTROL_PERIOD_S;
const float FEEDBACK_PRINT_S    = 0.050f;
const float MIT_FEEDBACK_WAIT_S = 0.8f;
const float COMMAND_TIMEOUT_S   = 20.0f;
const float GRIP_POS_SEND_S     = 0.040f;

const uint32_t SERIAL_BAUD = 115200;
const float JOG_SPEED = 1.0f;

// ===================== EXTRA SERVO =====================
#define SERVO_PIN 9

Servo FServo;

int servo_cmd = 0;
int last_servo_cmd = -1;

const int SERVO_ANGLE_0 = 0;
const int SERVO_ANGLE_1 = 60;

// ===================== ARM CONFIG =====================
#define NJ 4

const float P_MIN  = -120.50f;
const float P_MAX  =  120.50f;
const float V_MIN  =  -50.0f;
const float V_MAX  =   50.0f;
const float KP_MIN =    0.0f;
const float KP_MAX =  500.0f;
const float KD_MIN =    0.0f;
const float KD_MAX =    5.0f;
const float T_MIN  =  -18.0f;
const float T_MAX  =   18.0f;

const float FB_P_MIN = -120.50f;
const float FB_P_MAX =  120.50f;
const float FB_V_MIN =  -30.0f;
const float FB_V_MAX =   30.0f;
const float FB_T_MIN =  -18.0f;
const float FB_T_MAX =   18.0f;

// J0=base, J1=shoulder, J2=elbow, J3=roll
const uint8_t MIT_BASE_ID     = 0x01;
const uint8_t MIT_SHOULDER_ID = 0x02;
const uint8_t MIT_ELBOW_ID    = 0x03;
const uint8_t MIT_ROLL_ID     = 0x04;

const uint8_t MIT_JOINT_IDS[NJ] = {
  MIT_BASE_ID,
  MIT_SHOULDER_ID,
  MIT_ELBOW_ID,
  MIT_ROLL_ID
};

float J_SIGN[NJ] = { +1.0f, +1.0f, +1.0f, +1.0f};

float q_min[NJ] = { -6.14f, -2.0f, 0.1f, -4.14f};
float q_max[NJ] = { 6.14f,  20.0f,  15.0f,  4.14f};

float kp_j[NJ] = {120.0f, 250.0f, 250.0f, 60.0f};
float kd_j[NJ] = { 2.5f,   3.0f,   4.0f,  2.0f};

// ===================== GRIPPERS =====================
const uint8_t GRIP1_DEV_ID = 0x09;
const uint8_t GRIP2_DEV_ID = 0x0A;

const float GRIP_OPEN_RAD  = -2.00f;
const float GRIP_CLOSE_RAD = 4.00f;
const float GRIP_JOG_SPEED_RAD_S = 0.35f;

const bool GRIP_POS_UNITS_DEGx100 = true;
const uint8_t GDZ_CMD_SET_POS = 0xC2;

// ===================== MIT COMMANDS =====================
const uint8_t MIT_CMD_ENTER_MOTOR_MODE = 0xFC;
const uint8_t MIT_CMD_EXIT_MOTOR_MODE  = 0xFD;

// ===================== HELPERS =====================
float nowSeconds() {
  return millis() * 0.001f;
}

float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

float ease_out(float u) {
  u = clampf(u, 0.0f, 1.0f);
  return u * (2.0f - u);
}

uint32_t float_to_uint(float x, float x_min, float x_max, int bits) {
  x = clampf(x, x_min, x_max);
  float span = x_max - x_min;

  if (bits == 12) return (uint32_t)((x - x_min) * 4095.0f / span);
  if (bits == 16) return (uint32_t)((x - x_min) * 65535.0f / span);

  return 0;
}

float uint_to_float(uint32_t x_int, float x_min, float x_max, int bits) {
  float span = x_max - x_min;

  if (bits == 12) return float(x_int) * span / 4095.0f + x_min;
  if (bits == 16) return float(x_int) * span / 65535.0f + x_min;

  return x_min;
}

int32_t grip_pos_to_raw(float pos_rad) {
  if (GRIP_POS_UNITS_DEGx100) {
    float deg = pos_rad * 57.2957795f;
    return (int32_t)roundf(deg * 100.0f);
  } else {
    return (int32_t)roundf(pos_rad * 10000.0f);
  }
}

// ===================== TRAJECTORY =====================
struct Traj {
  float q0;
  float qf;
  float t;
  float T;
  bool active;
};

// ===================== ARM CONTROLLER =====================
class ArmController {
  public:
    bool synced = false;

    Traj traj[NJ];

    float q_goal[NJ] = {0};
    float q_des[NJ]  = {0};
    float q_cmd[NJ]  = {0};
    float q_fb[NJ]   = {0};

    float m_pos[16] = {0};
    float m_vel[16] = {0};
    float m_trq[16] = {0};
    bool  m_seen[16] = {false};

    int gripper1_cmd = 0;
    int gripper2_cmd = 0;

    //  float grip1_pos_cmd = GRIP_OPEN_RAD;
    //  float grip2_pos_cmd = GRIP_OPEN_RAD;
    float grip1_pos_cmd = 0;
    float grip2_pos_cmd = 0;

    float grip1_last_send_s = 0.0f;
    float grip2_last_send_s = 0.0f;
    float _grip_last_update_s = 0.0f;

    bool motors_enabled = false;
    bool timed_out = true;
    bool timeout_printed = false;

    float last_command_s = 0.0f;
    float last_control_s = 0.0f;
    float last_fb_print_s = 0.0f;

    ArmController() {
      float t = nowSeconds();
      last_command_s = t;
      last_control_s = t;
      last_fb_print_s = t;
    }

    // ---------- CAN SEND ----------
    void _send(uint16_t arb_id, const uint8_t *data, uint8_t len) {
      CAN_message_t msg;
      msg.id = arb_id;
      msg.len = len;
      msg.flags.extended = 0;

      for (int i = 0; i < len; i++) {
        msg.buf[i] = data[i];
      }

      Can0.write(msg);
    }

    void send_s32_5b(uint16_t arb_id, uint8_t cmd, int32_t val_s32) {
      uint32_t v = (uint32_t)val_s32;

      uint8_t data[5];
      data[0] = cmd;
      data[1] = v & 0xFF;
      data[2] = (v >> 8) & 0xFF;
      data[3] = (v >> 16) & 0xFF;
      data[4] = (v >> 24) & 0xFF;

      _send(arb_id, data, 5);
    }

    void send_both_s32(uint8_t dev, uint8_t cmd, int32_t val_s32) {
      send_s32_5b(dev, cmd, val_s32);
      send_s32_5b(0x100 | dev, cmd, val_s32);
    }

    // ---------- MIT SEND ----------
    void send_mit(float position, float velocity, float kp, float kd, float t_ff, uint8_t motor_id) {
      float p_des = clampf(position, P_MIN, P_MAX);
      float v_des = clampf(velocity, V_MIN, V_MAX);

      kp = clampf(kp, KP_MIN, KP_MAX);
      kd = clampf(kd, KD_MIN, KD_MAX);
      t_ff = clampf(t_ff, T_MIN, T_MAX);

      uint32_t p_int  = float_to_uint(p_des, P_MIN, P_MAX, 16);
      uint32_t v_int  = float_to_uint(v_des, V_MIN, V_MAX, 12);
      uint32_t kp_int = float_to_uint(kp,    KP_MIN, KP_MAX, 12);
      uint32_t kd_int = float_to_uint(kd,    KD_MIN, KD_MAX, 12);
      uint32_t t_int  = float_to_uint(t_ff,  T_MIN, T_MAX, 12);

      uint8_t data[8];

      data[0] = (p_int >> 8) & 0xFF;
      data[1] = p_int & 0xFF;
      data[2] = (v_int >> 4) & 0xFF;
      data[3] = ((v_int & 0xF) << 4) | ((kp_int >> 8) & 0xF);
      data[4] = kp_int & 0xFF;
      data[5] = (kd_int >> 4) & 0xFF;
      data[6] = ((kd_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
      data[7] = t_int & 0xFF;

      _send(motor_id, data, 8);
    }

    void enter_mit_motor_mode(uint8_t motor_id) {
      uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MIT_CMD_ENTER_MOTOR_MODE};
      _send(motor_id, data, 8);
    }

    void exit_mit_motor_mode(uint8_t motor_id) {
      uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, MIT_CMD_EXIT_MOTOR_MODE};
      _send(motor_id, data, 8);
    }

    void enter_all_mit_motor_mode() {
      for (int i = 0; i < NJ; i++) {
        enter_mit_motor_mode(MIT_JOINT_IDS[i]);
        delay(1);
      }
    }

    void exit_all_mit_motor_mode() {
      for (int i = 0; i < NJ; i++) {
        exit_mit_motor_mode(MIT_JOINT_IDS[i]);
        delay(1);
      }
    }

    // ---------- FEEDBACK ----------
    void decode_mit_reply(const CAN_message_t &msg) {
      uint16_t mid = msg.id & 0x7FF;
      uint8_t k = 0;

      if (mid == 0) {
        if (msg.len < 6) return;
        mid = msg.buf[0];
        k = 1;
      }

      if (0x101 <= mid && mid <= 0x106) {
        mid -= 0x100;
      }

      if (mid >= 16) return;
      if (msg.len < k + 5) return;

      uint32_t p_int = ((uint32_t)msg.buf[k + 0] << 8) | msg.buf[k + 1];
      uint32_t v_int = ((uint32_t)msg.buf[k + 2] << 4) | (msg.buf[k + 3] >> 4);
      uint32_t t_int = ((uint32_t)(msg.buf[k + 3] & 0xF) << 8) | msg.buf[k + 4];

      m_pos[mid] = uint_to_float(p_int, FB_P_MIN, FB_P_MAX, 16);
      m_vel[mid] = uint_to_float(v_int, FB_V_MIN, FB_V_MAX, 12);
      m_trq[mid] = uint_to_float(t_int, FB_T_MIN, FB_T_MAX, 12);

      m_seen[mid] = true;
    }

    void poll_can_feedback() {
      CAN_message_t msg;

      while (Can0.read(msg)) {
        decode_mit_reply(msg);
      }
    }

    void update_joint_feedback() {
      for (int j = 0; j < NJ; j++) {
        uint8_t mid = MIT_JOINT_IDS[j];
        q_fb[j] = m_pos[mid];
      }
    }

    bool feedback_ready() {
      for (int j = 0; j < NJ; j++) {
        if (!m_seen[MIT_JOINT_IDS[j]]) return false;
      }

      return true;
    }

    void clear_seen_flags_on_arming() {
      for (int j = 0; j < NJ; j++) {
        m_seen[MIT_JOINT_IDS[j]] = false;
      }
    }

    // ---------- TRAJECTORY ----------
    void plan_move(int j, float goal, float duration_s) {
      goal = clampf(goal, q_min[j], q_max[j]);

      traj[j].q0 = q_cmd[j];
      traj[j].qf = goal;
      traj[j].t = 0.0f;
      traj[j].T = duration_s < DT ? DT : duration_s;
      traj[j].active = true;

      q_goal[j] = goal;
    }

    void plan_move_all(float target[NJ], float duration_s) {
      for (int j = 0; j < NJ; j++) {
        plan_move(j, target[j], duration_s);
      }
    }

    void update_trajectory() {
      for (int j = 0; j < NJ; j++) {
        Traj &tr = traj[j];

        if (!tr.active) {
          q_des[j] = q_goal[j];
          continue;
        }

        tr.t += DT;

        float u = tr.t / tr.T;
        float s = ease_out(u);

        q_des[j] = tr.q0 + s * (tr.qf - tr.q0);

        if (u >= 1.0f) {
          tr.active = false;
          q_des[j] = tr.qf;
        }
      }
    }

    void emergency_hold() {
      for (int j = 0; j < NJ; j++) {
        q_goal[j] = q_cmd[j];
        q_des[j]  = q_cmd[j];
        traj[j].active = false;
      }
    }

    void sync_command_to_feedback_partial() {
      bool ok[NJ] = {false, false, false, false};

      for (int j = 0; j < NJ; j++) {
        uint8_t mid = MIT_JOINT_IDS[j];

        if (m_seen[mid]) {
          q_fb[j] = m_pos[mid];
          ok[j] = true;
        }
      }

      for (int j = 0; j < NJ; j++) {
        float v = ok[j] ? q_fb[j] : q_des[j];

        q_goal[j] = v;
        q_des[j]  = v;
        q_cmd[j]  = v;

        traj[j].active = false;
      }
    }

    void sync_from_feedback_blocking(float timeout_s = 1.0f) {
      float t0 = nowSeconds();

      while ((nowSeconds() - t0) < timeout_s) {
        poll_can_feedback();
        update_joint_feedback();

        if (feedback_ready()) break;

        delay(2);
      }

      sync_command_to_feedback_partial();
      synced = true;

      Serial.println("SYNC: MIT command state aligned to feedback where available");
    }

    // ---------- GRIPPERS ----------
    void gripper_jog_hold_update(float now_s) {
      if (!motors_enabled) return;

      if (_grip_last_update_s == 0.0f) {
        _grip_last_update_s = now_s;
      }

      float dt = now_s - _grip_last_update_s;
      _grip_last_update_s = now_s;

      if (dt > 0.05f) dt = 0.05f;

      if (gripper1_cmd < 0) {
        grip1_pos_cmd += GRIP_JOG_SPEED_RAD_S * dt;
      } else if (gripper1_cmd > 0) {
        grip1_pos_cmd -= GRIP_JOG_SPEED_RAD_S * dt;
      }

      if (gripper2_cmd < 0) {
        grip2_pos_cmd += GRIP_JOG_SPEED_RAD_S * dt;
      } else if (gripper2_cmd > 0) {
        grip2_pos_cmd -= GRIP_JOG_SPEED_RAD_S * dt;
      }

      grip1_pos_cmd = clampf(grip1_pos_cmd, GRIP_OPEN_RAD, GRIP_CLOSE_RAD);
      grip2_pos_cmd = clampf(grip2_pos_cmd, GRIP_OPEN_RAD, GRIP_CLOSE_RAD);

      if ((now_s - grip1_last_send_s) >= GRIP_POS_SEND_S) {
        grip1_last_send_s = now_s;
        send_both_s32(GRIP1_DEV_ID, GDZ_CMD_SET_POS, grip_pos_to_raw(grip1_pos_cmd));
      }

      if ((now_s - grip2_last_send_s) >= GRIP_POS_SEND_S) {
        grip2_last_send_s = now_s;
        send_both_s32(GRIP2_DEV_ID, GDZ_CMD_SET_POS, grip_pos_to_raw(grip2_pos_cmd));
      }
    }

    // ---------- COMMAND OUTPUT ----------
    void send_motor_commands() {
      float q0 = clampf(J_SIGN[0] * q_cmd[0], q_min[0], q_max[0]);
      float q1 = clampf(J_SIGN[1] * q_cmd[1], q_min[1], q_max[1]);
      float q2 = clampf(J_SIGN[2] * q_cmd[2], q_min[2], q_max[2]);
      float q3 = clampf(J_SIGN[3] * q_cmd[3], q_min[3], q_max[3]);

      send_mit(q0, 0.0f, kp_j[0], kd_j[0], 0.0f, MIT_BASE_ID);
      send_mit(q1, 0.0f, kp_j[1], kd_j[1], 0.0f, MIT_SHOULDER_ID);
      send_mit(q2, 0.0f, kp_j[2], kd_j[2], 0.0f, MIT_ELBOW_ID);
      send_mit(q3, 0.0f, kp_j[3], kd_j[3], 0.0f, MIT_ROLL_ID);
    }

    // ---------- ARMING ----------
    void arm() {
      if (!synced) {
        sync_from_feedback_blocking(1.0f);
      }

      last_command_s = nowSeconds();

      clear_seen_flags_on_arming();
      enter_all_mit_motor_mode();

      motors_enabled = true;

      gripper1_cmd = 0;
      gripper2_cmd = 0;

      grip1_last_send_s = 0.0f;
      grip2_last_send_s = 0.0f;
      _grip_last_update_s = 0.0f;

      float t0 = nowSeconds();

      while ((nowSeconds() - t0) < MIT_FEEDBACK_WAIT_S) {
        poll_can_feedback();
        update_joint_feedback();

        if (feedback_ready()) break;

        delay(1);
      }

      if (!feedback_ready()) {
        Serial.println("WARN: feedback not ready; arming in degraded mode");
      }

      sync_command_to_feedback_partial();

      timed_out = false;
      timeout_printed = false;

      Serial.println("MK (armed)");
    }

    void boot_safe_idle() {
      exit_all_mit_motor_mode();
    }

    void shutdown_safe_idle() {
      exit_all_mit_motor_mode();
    }

    void periodic_feedback_requests(float now_s) {
      (void)now_s;
    }

    void fixed_rate_control_tick(float now_s) {
      if (motors_enabled && ((now_s - last_command_s) > COMMAND_TIMEOUT_S)) {
        if (!timed_out) {
          timed_out = true;
          emergency_hold();
        }

        gripper1_cmd = 0;
        gripper2_cmd = 0;

        if (!timeout_printed) {
          Serial.println("TIMEOUT! Holding position.");
          timeout_printed = true;
        }
      }

      if (motors_enabled && synced) {
        if (!timed_out) {
          update_trajectory();

          for (int j = 0; j < NJ; j++) {
            q_cmd[j] = q_des[j];
          }
        }

        send_motor_commands();
      }
    }

    void debug_print(float now_s) {
      if ((now_s - last_fb_print_s) >= FEEDBACK_PRINT_S) {
        last_fb_print_s = now_s;

        Serial.print("FB J0..J3: ");

        for (int j = 0; j < NJ; j++) {
          Serial.print(q_fb[j], 3);
          if (j < NJ - 1) Serial.print(", ");
        }

        Serial.print(" | G1Cmd=");
        Serial.print(gripper1_cmd);
        Serial.print(" G1Pos=");
        Serial.print(grip1_pos_cmd, 3);

        Serial.print(" | G2Cmd=");
        Serial.print(gripper2_cmd);
        Serial.print(" G2Pos=");
        Serial.println(grip2_pos_cmd, 3);
      }
    }
};

// ===================== GLOBALS =====================
ArmController arm;

String serialLine;
float q_target[NJ] = {0};

bool enabled = false;

// ===================== SERIAL PARSER =====================
bool parse_serial_csv(const String &line_in, float target[NJ], int &grip1_cmd, int &grip2_cmd, int &servo_cmd) {
  String line = line_in;
  line.trim();

  if (line.length() == 0) return false;

  float values[7] = {0};
  int start = 0;

  for (int i = 0; i < 7; i++) {
    int comma = line.indexOf(',', start);
    String part;

    if (i < 6) {
      if (comma < 0) {
        Serial.println("Invalid format. Use: base,shoulder,elbow,roll,gripper1,gripper2,servo");
        return false;
      }

      part = line.substring(start, comma);
      start = comma + 1;
    } else {
      part = line.substring(start);
    }

    part.trim();
    values[i] = part.toFloat();
  }

  if (line.indexOf(',', start) >= 0) {
    Serial.println("Invalid format. Too many values.");
    return false;
  }

  for (int j = 0; j < NJ; j++) {
    target[j] = clampf(values[j], q_min[j], q_max[j]);
  }

  if (values[4] > 0.5f) grip1_cmd = 1;
  else if (values[4] < -0.5f) grip1_cmd = -1;
  else grip1_cmd = 0;

  if (values[5] > 0.5f) grip2_cmd = 1;
  else if (values[5] < -0.5f) grip2_cmd = -1;
  else grip2_cmd = 0;

  servo_cmd = values[6] > 0.5f ? 1 : 0;

  return true;
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);
  FServo.attach(SERVO_PIN);
  FServo.write(SERVO_ANGLE_0);
  Serial.println("Opening CAN: Teensy CAN1, 1 Mbps");

  Can0.begin();
  Can0.setBaudRate(CAN_BITRATE);
  Can0.setMaxMB(16);
  Can0.enableFIFO();
  Can0.enableFIFOInterrupt();

  delay(200);

  arm.boot_safe_idle();

  arm.sync_from_feedback_blocking(0.2f);

  Serial.println("Boot: MIT motors sent EXIT/IDLE command");
  Serial.println("Listening.");
  Serial.println("Send x to enable motors.");
  Serial.println("Serial CSV = base,shoulder,elbow,roll,gripper1,gripper2,servo");
  Serial.println("Example: 0,0,0,0,0,0,0");
}

// ===================== LOOP =====================
void loop() {
  float now_s = nowSeconds();

  // ---------- CAN FEEDBACK ----------
  arm.poll_can_feedback();
  arm.update_joint_feedback();

  arm.periodic_feedback_requests(now_s);

  // ---------- SERIAL INPUT ----------
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      serialLine.trim();

      if (serialLine.length() > 0) {
        int grip1_cmd = 0;
        int grip2_cmd = 0;
        int servo_cmd = 0;
        // Enable motors
        if (serialLine == "x" || serialLine == "X") {
          if (!enabled) {
            arm.arm();
            enabled = true;
            Serial.println("Motors ENABLED");
          }

          serialLine = "";
          continue;
        }

        // Motion command

        if (parse_serial_csv(serialLine, q_target, grip1_cmd, grip2_cmd, servo_cmd)) {
          if (!enabled) {
            Serial.println("Send x first to enable motors");
            serialLine = "";
            continue;
          }

          arm.gripper1_cmd = grip1_cmd;
          arm.gripper2_cmd = grip2_cmd;

          servo_cmd = servo_cmd;

          if (servo_cmd != last_servo_cmd) {
            if (servo_cmd == 0) {
              FServo.write(SERVO_ANGLE_0);
            } else {
              FServo.write(SERVO_ANGLE_1);
            }

            last_servo_cmd = servo_cmd;
          }

          arm.last_command_s = now_s;
          arm.timed_out = false;
          arm.timeout_printed = false;

          float max_delta = 0.0f;

          for (int j = 0; j < NJ; j++) {
            max_delta = max(max_delta, fabsf(q_target[j] - arm.q_des[j]));
          }

          float move_t = max(0.01f, max_delta / JOG_SPEED);

          arm.plan_move_all(q_target, move_t);

          Serial.print("RX: ");
          for (int j = 0; j < NJ; j++) {
            Serial.print(q_target[j], 3);
            Serial.print(",");
          }

          Serial.print(arm.gripper1_cmd);
          Serial.print(",");
          Serial.print(arm.gripper2_cmd);
          Serial.print(",");
          Serial.println(servo_cmd);
        }

        serialLine = "";
      }
    } else {
      serialLine += c;

      if (serialLine.length() > 120) {
        serialLine = "";
        Serial.println("Serial line too long. Cleared.");
      }
    }
  }

  // ---------- FIXED 200 Hz CONTROL ----------
  if ((now_s - arm.last_control_s) >= CONTROL_PERIOD_S) {
    arm.last_control_s += CONTROL_PERIOD_S;
    arm.fixed_rate_control_tick(now_s);
  }

  // ---------- GRIPPERS ----------
  arm.gripper_jog_hold_update(now_s);

  // ---------- DEBUG ----------
  arm.debug_print(now_s);
}