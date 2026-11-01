#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>
#include <Wire.h>
#include "config.h"

// =====================================================================================
//  Khai bao bien toan cuc (dinh nghia trong src/globals.cpp) va nguyen mau ham
// =====================================================================================

//Cong log: chi ghi ra Serial khi may tinh dang mo cong COM, con lai bo qua (khong chan vong lap).
struct DebugPort {
  void begin(long baud) { DEBUG_PORT.begin(baud); }
  void write(uint8_t c) { if (DEBUG_LINK_UP()) DEBUG_PORT.write(c); }
  template <typename T> void print(T v)   { if (DEBUG_LINK_UP()) DEBUG_PORT.print(v); }
  template <typename T> void println(T v) { if (DEBUG_LINK_UP()) DEBUG_PORT.println(v); }
  void println(void)                      { if (DEBUG_LINK_UP()) DEBUG_PORT.println(); }
};
extern DebugPort dbg;

extern TwoWire HWire;                       // I2C port 2 (PB10 = SCL, PB11 = SDA), 400 kHz

// ---------- He so PID ----------
extern float pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw;
extern int   pid_max_yaw;
extern float pid_p_gain_altitude, pid_i_gain_altitude, pid_d_gain_altitude;
extern int   pid_max_altitude;
extern float pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll;
extern int   pid_max_roll;
extern float pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch;
extern int   pid_max_pitch;

// ---------- Cau hinh dong co / cam bien ----------
extern int16_t manual_takeoff_throttle, motor_idle_speed;
extern float   declination;
extern uint8_t gyro_address, MS5611_address, compass_address;

// ---------- Trang thai chuong trinh ----------
extern uint8_t start;                       // 0 = tat, 1 = dang khoi dong, 2 = dong co chay
extern uint8_t error;                       // ma loi (xem debug.cpp)
extern uint8_t flight_mode;                 // 1 = thu cong, 2 = giu do cao
extern uint8_t takeoff_detected, manual_altitude_change;
extern uint8_t level_calibration_on, compass_calibration_on, heading_lock;

// ---------- Nhan RC (input capture TIM2) ----------
extern uint8_t channel_select_counter;
extern int32_t channel_1, channel_2, channel_3, channel_4, channel_5, channel_6;
extern int32_t channel_1_base, channel_2_base, pid_roll_setpoint_base, pid_pitch_setpoint_base;
extern int32_t measured_time, measured_time_start, receiver_watchdog;

// ---------- Dong co / ESC ----------
extern int16_t esc_1, esc_2, esc_3, esc_4;
extern int16_t throttle, takeoff_throttle, manual_throttle;

// ---------- MPU-6050 ----------
extern int16_t cal_int;
extern int16_t acc_x, acc_y, acc_z, temperature;
extern int16_t gyro_pitch, gyro_roll, gyro_yaw;
extern int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
extern int16_t acc_pitch_cal_value, acc_roll_cal_value;
extern int32_t acc_total_vector, acc_total_vector_at_start;
extern int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total;
extern int16_t acc_z_average_short[26], acc_z_average_long[51];
extern uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;
extern int32_t acc_alt_integrated;
extern float   angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
extern float   roll_level_adjust, pitch_level_adjust;

// ---------- PID ----------
extern float pid_error_temp;
extern float pid_i_mem_roll,  pid_roll_setpoint,  gyro_roll_input,  pid_output_roll,  pid_last_roll_d_error;
extern float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
extern float pid_i_mem_yaw,   pid_yaw_setpoint,   gyro_yaw_input,   pid_output_yaw,   pid_last_yaw_d_error;

// ---------- HMC5883L ----------
extern int16_t compass_x, compass_y, compass_z;
extern int16_t compass_cal_values[6];
extern float   compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
extern float   compass_scale_y, compass_scale_z;
extern int16_t compass_offset_x, compass_offset_y, compass_offset_z;
extern float   course_lock_heading, heading_lock_course_deviation;

// ---------- MS5611 + PID do cao ----------
extern uint16_t C[7];
extern uint8_t  barometer_counter, temperature_counter, average_temperature_mem_location;
extern int64_t  OFF, OFF_C2, SENS, SENS_C1, P;
extern uint32_t raw_pressure, raw_temperature, raw_temperature_rotating_memory[6], raw_average_temperature_total;
extern float    actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
extern float    ground_pressure;
extern int32_t  dT;
extern float    pid_error_gain_altitude;
extern float    pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude;
extern uint8_t  parachute_rotating_mem_location;
extern int32_t  parachute_buffer[35], parachute_throttle;
extern float    pressure_parachute_previous;
extern int32_t  pressure_rotating_mem[50], pressure_total_avarage;
extern uint8_t  pressure_rotating_mem_location;

// ---------- Thoi gian vong lap ----------
extern uint32_t loop_timer, loop_timer_capture;

// =====================================================================================
//  Doc I2C: doc 2 / 3 byte theo thu tu xac dinh (tranh phu thuoc thu tu danh gia bieu thuc)
// =====================================================================================
static inline int16_t i2c_read_i16(void) {
  uint8_t hi = HWire.read();
  uint8_t lo = HWire.read();
  return (int16_t)(((uint16_t)hi << 8) | lo);
}
static inline uint32_t i2c_read_u24(void) {
  uint32_t b2 = (uint8_t)HWire.read();
  uint32_t b1 = (uint8_t)HWire.read();
  uint32_t b0 = (uint8_t)HWire.read();
  return (b2 << 16) | (b1 << 8) | b0;
}

// ---------- Nguyen mau ham ----------
void timer_setup(void);                     // timer_setup.cpp
void handler_channel_1(void);               // input_capture_mode_handlers.cpp

void gyro_setup(void);                      // gyro.cpp
void gyro_signalen(void);
void calibrate_gyro(void);

void compass_setup(void);                   // compass.cpp
void compass_compute_calibration(void);
void read_compass(void);
float course_deviation(float course_b, float course_c);

void barometer_setup(void);                 // barometer.cpp
void read_barometer(void);

void vertical_acceleration_calculations(void);  // vertical_acceleration_calculations.cpp
void calculate_pid(void);                   // calculate_pid.cpp
void start_stop_takeoff(void);              // start_stop_takeoff.cpp

void calibrate_compass(void);               // calibration.cpp
void calibrate_level(void);

void debug_begin(void);                     // debug.cpp
void debug_halt(uint8_t code, const char *message);   // dung han khi loi nang luc khoi dong
void debug_report_waiting(uint8_t code, const char *message);
void debug_update(void);                    // goi moi vong lap, khong chan

#endif // GLOBALS_H
