#include "globals.h"

void calibrate_compass(void) {
  compass_calibration_on = 1;                             //Tat bu offset/scale de lay gia tri tho.
  DEBUG_SERIAL.println("[CAL] Hieu chuan la ban: xoay drone moi huong, xong thi day can pitch len het co");

  //Dat lai min/max (code goc bat dau tu gia tri EEPROM, o day bat dau tu cuc tri de khong bi nhiem gia tri mac dinh).
  compass_cal_values[0] = compass_cal_values[2] = compass_cal_values[4] = 32767;
  compass_cal_values[1] = compass_cal_values[3] = compass_cal_values[5] = -32768;

  uint16_t n = 0;
  while (channel_2 < 1900) {                              //Cho den khi phi cong day can pitch.
    delayMicroseconds(3700);                              //Mo phong vong lap 250Hz.
    read_compass();
    if (compass_x < compass_cal_values[0]) compass_cal_values[0] = compass_x;
    if (compass_x > compass_cal_values[1]) compass_cal_values[1] = compass_x;
    if (compass_y < compass_cal_values[2]) compass_cal_values[2] = compass_y;
    if (compass_y > compass_cal_values[3]) compass_cal_values[3] = compass_y;
    if (compass_z < compass_cal_values[4]) compass_cal_values[4] = compass_z;
    if (compass_z > compass_cal_values[5]) compass_cal_values[5] = compass_z;

    if (++n % 250 == 0) {                               //Moi ~1 giay bao tien trinh.
      DEBUG_SERIAL.print("[CAL] x:"); DEBUG_SERIAL.print(compass_cal_values[0]); DEBUG_SERIAL.print(".."); DEBUG_SERIAL.print(compass_cal_values[1]);
      DEBUG_SERIAL.print(" y:");      DEBUG_SERIAL.print(compass_cal_values[2]); DEBUG_SERIAL.print(".."); DEBUG_SERIAL.print(compass_cal_values[3]);
      DEBUG_SERIAL.print(" z:");      DEBUG_SERIAL.print(compass_cal_values[4]); DEBUG_SERIAL.print(".."); DEBUG_SERIAL.println(compass_cal_values[5]);
    }
  }
  compass_calibration_on = 0;

  DEBUG_SERIAL.print("[CAL] Xong. Dan vao config.h: #define COMPASS_CAL_DEFAULT { ");
  for (uint8_t i = 0; i < 6; i++) {
    DEBUG_SERIAL.print(compass_cal_values[i]);
    DEBUG_SERIAL.print(i < 5 ? ", " : " }");
  }
  DEBUG_SERIAL.println();

  compass_compute_calibration();                          //Ap dung ngay trong phien chay nay.
  read_compass();
  angle_yaw = actual_compass_heading;                     //Dat huong ban dau.

  error = 0;
  loop_timer = micros();                                  //Dat lai bo dem vong lap.
}

//Can bang ngang gia toc ke: dat drone nam phang, gat len goc trai tren de vao,
//day can pitch len (channel_2 >= 1100) de bat dau lay mau.
void calibrate_level(void) {
  level_calibration_on = 1;
  DEBUG_SERIAL.println("[CAL] Can bang ngang: dat drone nam phang, day can pitch len de bat dau");

  while (channel_2 < 1100) delay(10);                     //Cho phi cong day can pitch.

  int16_t old_pitch_cal = acc_pitch_cal_value;            //Giu gia tri cu de khoi phuc neu bi huy.
  int16_t old_roll_cal = acc_roll_cal_value;
  acc_pitch_cal_value = 0;
  acc_roll_cal_value = 0;

  bool too_tilted = false;
  for (uint8_t i = 0; i < 40 && !too_tilted; i++) {       //Lay 40 mau.
    gyro_signalen();
    acc_pitch_cal_value += acc_y;
    acc_roll_cal_value += acc_x;
    if (acc_y > 800 || acc_y < -800) too_tilted = true;   //Drone nghieng qua nhieu -> huy.
    if (acc_x > 800 || acc_x < -800) too_tilted = true;
    delayMicroseconds(3700);
  }

  acc_pitch_cal_value /= 40;
  acc_roll_cal_value /= 40;

  if (!too_tilted) {
    DEBUG_SERIAL.print("[CAL] Xong. Dan vao config.h: #define ACC_PITCH_CAL_DEFAULT ");
    DEBUG_SERIAL.print(acc_pitch_cal_value);
    DEBUG_SERIAL.print("  va  #define ACC_ROLL_CAL_DEFAULT ");
    DEBUG_SERIAL.println(acc_roll_cal_value);
    error = 0;
  } else {
    DEBUG_SERIAL.println("[CAL] Drone nghieng qua nhieu, huy can bang ngang (error = 3)");
    acc_pitch_cal_value = old_pitch_cal;                  //Khoi phuc gia tri cu.
    acc_roll_cal_value = old_roll_cal;
    error = 3;
  }

  level_calibration_on = 0;
  gyro_signalen();
  //Tinh goc gia toc ke.
  acc_total_vector = sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z));

  if (abs(acc_y) < acc_total_vector) {                    //Tranh asin cho ra NaN.
    angle_pitch_acc = asin((float)acc_y / acc_total_vector) * 57.296;
  }
  if (abs(acc_x) < acc_total_vector) {
    angle_roll_acc = asin((float)acc_x / acc_total_vector) * 57.296;
  }
  angle_pitch = angle_pitch_acc;                          //Goc gyro = goc gia toc ke luc bat dau.
  angle_roll = angle_roll_acc;
  loop_timer = micros();
}
