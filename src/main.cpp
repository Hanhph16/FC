#include <Arduino.h>
#include "globals.h"

void setup() {
  debug_begin();                                          //Serial Monitor (DEBUG_BAUD).
  DEBUG_SERIAL.println("=== YMFC-32 (ban rut gon) khoi dong ===");

  timer_setup();                                          //TIM2: doc receiver, TIM4: xuat PWM cho ESC.
  delay(50);                                              //Cho timer on dinh.

  HWire.begin();
  delay(150);                                             //Cho I2C on dinh.

  //Kiem tra cac cam bien co tra loi tren I2C khong.
  HWire.beginTransmission(gyro_address);
  if (HWire.endTransmission() != 0) debug_halt(1, "MPU6050 (0x68) khong phan hoi - kiem tra day SDA/SCL");
  DEBUG_SERIAL.println("[OK] MPU6050");

  HWire.beginTransmission(compass_address);
  if (HWire.endTransmission() != 0) debug_halt(2, "HMC5883L (0x1E) khong phan hoi - kiem tra day SDA/SCL");
  DEBUG_SERIAL.println("[OK] HMC5883L");

  HWire.beginTransmission(MS5611_address);
  if (HWire.endTransmission() != 0) debug_halt(3, "MS5611 (0x77) khong phan hoi - kiem tra day SDA/SCL");
  DEBUG_SERIAL.println("[OK] MS5611");

  gyro_setup();                                           //Cau hinh thanh ghi MPU6050.
  compass_setup();                                        //Cau hinh HMC5883L + tinh he so hieu chuan.
  read_compass();                                         //Doc huong ban dau.
  angle_yaw = actual_compass_heading;

  //Cho 5 giay truoc khi hieu chuan gyro (giu drone dung yen).
  DEBUG_SERIAL.println("Giu drone dung yen. Hieu chuan gyro sau 5 giay...");
  for (uint16_t i = 0; i < 1250; i++) {                   //1250 vong x 4ms = 5 giay.
    if (i % 250 == 0) {
      DEBUG_SERIAL.print("  ");
      DEBUG_SERIAL.println(5 - i / 250);
    }
    delay(4);
  }
  DEBUG_SERIAL.println("Dang hieu chuan gyro (~8 giay)...");
  cal_int = 0;
  calibrate_gyro();                                       //Tinh offset gyro.
  DEBUG_SERIAL.print("Offset gyro roll/pitch/yaw: ");
  DEBUG_SERIAL.print(gyro_roll_cal); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.print(gyro_pitch_cal); DEBUG_SERIAL.print(' ');
  DEBUG_SERIAL.println(gyro_yaw_cal);

  //Cho receiver hoat dong.
  while (channel_1 < 990 || channel_2 < 990 || channel_3 < 990 || channel_4 < 990) {
    debug_report_waiting(4, "Chua nhan tin hieu RC - bat tay cam / kiem tra day PPM (PA0)");
    delay(4);
  }
  error = 0;
  DEBUG_SERIAL.println("[OK] Nhan duoc tin hieu RC");

  barometer_setup();                                      //Doc he so MS5611 + lam nong bo loc ap suat.
  DEBUG_SERIAL.print("[OK] MS5611 PROM C1..C6: ");
  for (uint8_t i = 1; i <= 6; i++) { DEBUG_SERIAL.print(C[i]); DEBUG_SERIAL.print(' '); }
  DEBUG_SERIAL.println();

  //Nap san gia tri trung binh gia toc ke vao bo nho xoay.
  for (uint8_t i = 0; i <= 24; i++) acc_z_average_short[i] = acc_z;
  for (uint8_t i = 0; i <= 49; i++) acc_z_average_long[i] = acc_z;
  acc_z_average_short_total = acc_z * 25;
  acc_z_average_long_total = acc_z * 50;
  start = 0;

  if (motor_idle_speed < 1000) motor_idle_speed = 1000;   //Gioi han toc do cham toi thieu 1000us.
  if (motor_idle_speed > 1200) motor_idle_speed = 1200;   //Gioi han toc do cham toi da 1200us.

  DEBUG_SERIAL.println("San sang. Gat ga thap + yaw trai de khoi dong dong co.");
  loop_timer = micros();                                  //Dat bo dem cho vong lap dau.
}

///////////////////////////////////////////////////////////////////////////////////////////
//Vong lap chinh (4000us = 250Hz)
///////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  //Watchdog receiver: mat tin hieu ~3s (750 vong) khi dang bay -> gat ve giua + giu do cao.
  if (receiver_watchdog < 750 && start == 2) {
    receiver_watchdog++;
  }
  if (receiver_watchdog == 750 && start == 2) {
    channel_1 = 1500;
    channel_2 = 1500;
    channel_3 = 1500;
    channel_4 = 1500;
    channel_5 = 1500;                                     //Che do 2: giu do cao (khong co GPS nen khong co return-to-home).
    error = 8;                                            //Mat tin hieu tay cam.
  }

  //Mot so chuc nang chi dung khi dong co tat.
  if (start == 0) {
    //Hieu chuan la ban: ca 2 can day len goc phai tren.
    if (channel_1 > 1900 && channel_2 < 1100 && channel_3 > 1900 && channel_4 > 1900) calibrate_compass();

    //Can bang ngang: ca 2 can day len goc trai tren.
    if (channel_1 < 1100 && channel_2 < 1100 && channel_3 > 1900 && channel_4 < 1100) calibrate_level();

    receiver_watchdog = 0;
  }

  heading_lock = 0;
  if (channel_6 > 1200) {
    heading_lock = 1;                                     //Khoa huong (heading lock).
  }

  //Che do bay: 1 = thu cong, 2 = giu do cao (channel_5 >= 1200).
  //(Truoc day 1200-1600 = che do 2, 1600+ = che do 3 giu vi tri GPS; khong co GPS thi che do 3 hanh xu giong che do 2.)
  flight_mode = 1;
  if (channel_5 >= 1200 && channel_5 < 2100) flight_mode = 2;

  gyro_signalen();                                        //Doc gyro va gia toc ke.
  read_barometer();                                       //Doc va tinh ap suat.
  read_compass();                                         //Doc va tinh huong.

  //65.5 = 1 do/giay (xem datasheet MPU-6050).
  gyro_roll_input = (gyro_roll_input * 0.7) + (((float)gyro_roll / 65.5) * 0.3);    //Dau vao PID gyro la do/giay.
  gyro_pitch_input = (gyro_pitch_input * 0.7) + (((float)gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + (((float)gyro_yaw / 65.5) * 0.3);

  ////////////////////////////////////////////////////////////////////////////////////////
  //Tinh goc nghieng (IMU)
  ////////////////////////////////////////////////////////////////////////////////////////

  //Goc tu gyro. 0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += (float)gyro_pitch * 0.0000611;
  angle_roll += (float)gyro_roll * 0.0000611;
  angle_yaw += (float)gyro_yaw * 0.0000611;
  if (angle_yaw < 0) angle_yaw += 360;
  else if (angle_yaw >= 360) angle_yaw -= 360;

  //0.000001066 = 0.0000611 * (PI / 180). Khi IMU xoay yaw thi chuyen goc roll <-> pitch.
  angle_pitch -= angle_roll * sin((float)gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin((float)gyro_yaw * 0.000001066);

  //Hieu chinh troi yaw cua gyro bang la ban.
  angle_yaw -= course_deviation(angle_yaw, actual_compass_heading) / 1200.0;
  if (angle_yaw < 0) angle_yaw += 360;
  else if (angle_yaw >= 360) angle_yaw -= 360;

  //Goc tu gia toc ke.
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  if (abs(acc_y) < acc_total_vector) {                    //Tranh asin cho ra NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }

  //Bo loc bu: dung gia toc ke sua troi cua gyro.
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  pitch_level_adjust = angle_pitch * 15;                  //Hieu chinh goc pitch.
  roll_level_adjust = angle_roll * 15;                    //Hieu chinh goc roll.

  vertical_acceleration_calculations();                   //Gia toc theo phuong thang dung (phat hien cat canh).

  channel_1_base = channel_1;                             //Binh thuong channel_1 la dau vao setpoint roll.
  channel_2_base = channel_2;                             //Binh thuong channel_2 la dau vao setpoint pitch.
  //Khi heading_lock bat, setpoint roll/pitch phu thuoc huong luc khoi dong (course_lock_heading).
  if (heading_lock == 1) {
    heading_lock_course_deviation = course_deviation(angle_yaw, course_lock_heading);
    channel_1_base = 1500 + ((float)(channel_1 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_2 - 1500) * cos((heading_lock_course_deviation - 90) * 0.017453));
    channel_2_base = 1500 + ((float)(channel_2 - 1500) * cos(heading_lock_course_deviation * 0.017453)) + ((float)(channel_1 - 1500) * cos((heading_lock_course_deviation + 90) * 0.017453));
  }
  pid_roll_setpoint_base = channel_1_base;
  pid_pitch_setpoint_base = channel_2_base;

  //Gioi han setpoint trong 1000..2000us.
  if (pid_roll_setpoint_base > 2000) pid_roll_setpoint_base = 2000;
  if (pid_roll_setpoint_base < 1000) pid_roll_setpoint_base = 1000;
  if (pid_pitch_setpoint_base > 2000) pid_pitch_setpoint_base = 2000;
  if (pid_pitch_setpoint_base < 1000) pid_pitch_setpoint_base = 1000;

  calculate_pid();                                        //Tinh dau ra PID.

  start_stop_takeoff();                                   //Khoi dong / dung dong co / phat hien cat canh.

  //Ga co so cho moi dong co.
  if (takeoff_detected == 1 && start == 2) {              //Dang bay.
    throttle = channel_3 + takeoff_throttle;              //Ga co so = ga RC + ga cat canh da phat hien.
    if (flight_mode >= 2) {                               //Che do giu do cao.
      throttle = 1500 + takeoff_throttle + pid_output_altitude + manual_throttle;
    }
  }

  ////////////////////////////////////////////////////////////////////////////////////////
  //Tron lenh va tao xung cho 4 ESC
  ////////////////////////////////////////////////////////////////////////////////////////
  if (start == 2) {                                       //Dong co dang chay.
    if (throttle > 1800) throttle = 1800;                 //Chua khoang du de dieu khien khi ga toi da.
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;        //ESC 1 (truoc-phai, CCW).
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;        //ESC 2 (sau-phai,   CW).
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;        //ESC 3 (sau-trai,   CCW).
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;        //ESC 4 (truoc-trai, CW).

    if (esc_1 < motor_idle_speed) esc_1 = motor_idle_speed;                        //Giu dong co quay.
    if (esc_2 < motor_idle_speed) esc_2 = motor_idle_speed;
    if (esc_3 < motor_idle_speed) esc_3 = motor_idle_speed;
    if (esc_4 < motor_idle_speed) esc_4 = motor_idle_speed;

    if (esc_1 > 2000) esc_1 = 2000;                                                //Gioi han xung toi da 2000us.
    if (esc_2 > 2000) esc_2 = 2000;
    if (esc_3 > 2000) esc_3 = 2000;
    if (esc_4 > 2000) esc_4 = 2000;
  }
  else {
    esc_1 = 1000;                                         //start != 2: giu xung 1000us (dong co dung).
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }

  TIMER4_BASE->CCR1 = esc_1;                              //Xuat xung ra ESC 1.
  TIMER4_BASE->CCR2 = esc_2;
  TIMER4_BASE->CCR3 = esc_3;
  TIMER4_BASE->CCR4 = esc_4;
  TIMER4_BASE->CNT = 5000;                                //Reset TIM4 de xung ESC duoc tao ngay lap tuc.

  debug_update();                                         //Log Serial Monitor (khong chan, ~5Hz).

  //! ! ! Vi tinh goc phu thuoc thoi gian, vong lap PHAI dung 4000us. Khong them lenh cham vao day. ! ! !
  loop_timer_capture = micros() - loop_timer;
  if (loop_timer_capture > 4050) error = 5;               //Vong lap vuot 4050us.
  while (micros() - loop_timer < 4000);                   //Cho du 4000us.
  loop_timer = micros();                                  //Dat bo dem cho vong lap ke tiep.
}
