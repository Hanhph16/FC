#include "globals.h"

//Cau hinh thanh ghi MPU-6050.
void gyro_setup(void) {
  HWire.beginTransmission(gyro_address);
  HWire.write(0x6B);                                      //PWR_MGMT_1.
  HWire.write(0x00);                                      //Danh thuc cam bien.
  HWire.endTransmission();

  HWire.beginTransmission(gyro_address);
  HWire.write(0x1B);                                      //GYRO_CONFIG.
  HWire.write(0x08);                                      //500 do/giay toan thang do.
  HWire.endTransmission();

  HWire.beginTransmission(gyro_address);
  HWire.write(0x1C);                                      //ACCEL_CONFIG.
  HWire.write(0x10);                                      //+/- 8g.
  HWire.endTransmission();

  HWire.beginTransmission(gyro_address);
  HWire.write(0x1A);                                      //CONFIG.
  HWire.write(0x03);                                      //Loc thong thap so ~43Hz.
  HWire.endTransmission();
  //(Gia tri can bang acc pitch/roll lay tu config.h, khong con doc tu EEPROM.)
}

//Hieu chuan offset gyro: trung binh 2000 mau.
void calibrate_gyro(void) {
  cal_int = 0;
  for (cal_int = 0; cal_int < 2000; cal_int++) {
    gyro_signalen();
    gyro_roll_cal += gyro_roll;
    gyro_pitch_cal += gyro_pitch;
    gyro_yaw_cal += gyro_yaw;
    delay(4);                                             //Mo phong vong lap 250Hz.
  }
  gyro_roll_cal /= 2000;
  gyro_pitch_cal /= 2000;
  gyro_yaw_cal /= 2000;
}

//Doc du lieu tho gia toc + gyro tu MPU-6050.
void gyro_signalen(void) {
  HWire.beginTransmission(gyro_address);
  HWire.write(0x3B);                                      //Bat dau doc tu thanh ghi 0x3B, tu dong tang dia chi.
  HWire.endTransmission();
  HWire.requestFrom(gyro_address, 14);                    //Doc 14 byte.
  acc_y = i2c_read_i16();
  acc_x = i2c_read_i16();
  acc_z = i2c_read_i16();
  temperature = i2c_read_i16();
  gyro_roll = i2c_read_i16();
  gyro_pitch = i2c_read_i16();
  gyro_yaw = i2c_read_i16();
  gyro_pitch *= -1;                                       //Dao chieu truc.
  gyro_yaw *= -1;

  if (level_calibration_on == 0) {
    acc_y -= acc_pitch_cal_value;                         //Tru offset can bang pitch.
    acc_x -= acc_roll_cal_value;                          //Tru offset can bang roll.
  }
  if (cal_int >= 2000) {
    gyro_roll -= gyro_roll_cal;                           //Tru offset gyro.
    gyro_pitch -= gyro_pitch_cal;
    gyro_yaw -= gyro_yaw_cal;
  }
}
