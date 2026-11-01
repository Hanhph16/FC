#include "globals.h"

// =====================================================================================
//  Dinh nghia bien toan cuc
// =====================================================================================

TwoWire HWire(2, I2C_FAST_MODE);            // I2C port 2 o 400 kHz
DebugPort dbg;                              // cong log (Serial Monitor)

// ---------- He so PID (gia tri lay tu config.h) ----------
float pid_p_gain_yaw = PID_P_GAIN_YAW;
float pid_i_gain_yaw = PID_I_GAIN_YAW;
float pid_d_gain_yaw = PID_D_GAIN_YAW;
int   pid_max_yaw    = PID_MAX_YAW;

float pid_p_gain_altitude = PID_P_GAIN_ALTITUDE;
float pid_i_gain_altitude = PID_I_GAIN_ALTITUDE;
float pid_d_gain_altitude = PID_D_GAIN_ALTITUDE;
int   pid_max_altitude    = PID_MAX_ALTITUDE;

float pid_p_gain_roll = PID_P_GAIN_ROLL;
float pid_i_gain_roll = PID_I_GAIN_ROLL;
float pid_d_gain_roll = PID_D_GAIN_ROLL;
int   pid_max_roll    = PID_MAX_ROLL;

float pid_p_gain_pitch = PID_P_GAIN_ROLL;   // pitch dung chung he so voi roll
float pid_i_gain_pitch = PID_I_GAIN_ROLL;
float pid_d_gain_pitch = PID_D_GAIN_ROLL;
int   pid_max_pitch    = PID_MAX_ROLL;

// ---------- Cau hinh dong co / cam bien ----------
int16_t manual_takeoff_throttle = MANUAL_TAKEOFF_THROTTLE;
int16_t motor_idle_speed        = MOTOR_IDLE_SPEED;
float   declination             = COMPASS_DECLINATION;
uint8_t gyro_address            = MPU6050_ADDRESS;
uint8_t MS5611_address          = MS5611_ADDRESS;
uint8_t compass_address         = HMC5883L_ADDRESS;

// ---------- Trang thai ----------
uint8_t start;
uint8_t error;
uint8_t flight_mode;
uint8_t takeoff_detected, manual_altitude_change;
uint8_t level_calibration_on, compass_calibration_on, heading_lock;

// ---------- Nhan RC ----------
uint8_t channel_select_counter;
int32_t channel_1, channel_2, channel_3, channel_4, channel_5, channel_6;
int32_t channel_1_base, channel_2_base, pid_roll_setpoint_base, pid_pitch_setpoint_base;
int32_t measured_time, measured_time_start, receiver_watchdog;

// ---------- ESC ----------
int16_t esc_1, esc_2, esc_3, esc_4;
int16_t throttle, takeoff_throttle, manual_throttle;

// ---------- MPU-6050 ----------
int16_t cal_int;
int16_t acc_x, acc_y, acc_z, temperature;
int16_t gyro_pitch, gyro_roll, gyro_yaw;
int32_t gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;
int16_t acc_pitch_cal_value = ACC_PITCH_CAL_DEFAULT;
int16_t acc_roll_cal_value  = ACC_ROLL_CAL_DEFAULT;
int32_t acc_total_vector, acc_total_vector_at_start;
int32_t acc_z_average_short_total, acc_z_average_long_total, acc_z_average_total;
int16_t acc_z_average_short[26], acc_z_average_long[51];
uint8_t acc_z_average_short_rotating_mem_location, acc_z_average_long_rotating_mem_location;
int32_t acc_alt_integrated;
float   angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, angle_yaw;
float   roll_level_adjust, pitch_level_adjust;

// ---------- PID ----------
float pid_error_temp;
float pid_i_mem_roll,  pid_roll_setpoint,  gyro_roll_input,  pid_output_roll,  pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw,   pid_yaw_setpoint,   gyro_yaw_input,   pid_output_yaw,   pid_last_yaw_d_error;

// ---------- HMC5883L ----------
int16_t compass_x, compass_y, compass_z;
int16_t compass_cal_values[6] = COMPASS_CAL_DEFAULT;
float   compass_x_horizontal, compass_y_horizontal, actual_compass_heading;
float   compass_scale_y, compass_scale_z;
int16_t compass_offset_x, compass_offset_y, compass_offset_z;
float   course_lock_heading, heading_lock_course_deviation;

// ---------- MS5611 + PID do cao ----------
uint16_t C[7];
uint8_t  barometer_counter, temperature_counter, average_temperature_mem_location;
int64_t  OFF, OFF_C2, SENS, SENS_C1, P;
uint32_t raw_pressure, raw_temperature, raw_temperature_rotating_memory[6], raw_average_temperature_total;
float    actual_pressure, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float    ground_pressure;
int32_t  dT;
float    pid_error_gain_altitude;
float    pid_i_mem_altitude, pid_altitude_setpoint, pid_altitude_input, pid_output_altitude;
uint8_t  parachute_rotating_mem_location;
int32_t  parachute_buffer[35], parachute_throttle;
float    pressure_parachute_previous;
int32_t  pressure_rotating_mem[50], pressure_total_avarage;
uint8_t  pressure_rotating_mem_location;

// ---------- Thoi gian ----------
uint32_t loop_timer, loop_timer_capture;
