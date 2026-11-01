#include "globals.h"

//Doc HMC5883L va tinh huong (heading) 0..360 do.
void read_compass(void) {
  HWire.beginTransmission(compass_address);
  HWire.write(0x03);                                      //Bat dau doc tu thanh ghi du lieu 0x03.
  HWire.endTransmission();

  HWire.requestFrom(compass_address, 6);                  //Doc 6 byte (thu tu cua chip: X, Z, Y).
  compass_y = i2c_read_i16();
  compass_y *= -1;                                        //Dao chieu truc.
  compass_z = i2c_read_i16();
  compass_x = i2c_read_i16();
  compass_x *= -1;                                        //Dao chieu truc.

  //Ap dung offset / he so scale tu hieu chuan (khi khong o che do hieu chuan).
  if (compass_calibration_on == 0) {
    compass_y += compass_offset_y;
    compass_y *= compass_scale_y;
    compass_z += compass_offset_z;
    compass_z *= compass_scale_z;
    compass_x += compass_offset_x;
  }

  //Bu nghieng: quy gia tri x, y ve mat phang ngang ao theo goc roll / pitch.
  compass_x_horizontal = (float)compass_x * cos(angle_pitch * -0.0174533) + (float)compass_y * sin(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533) - (float)compass_z * cos(angle_roll * 0.0174533) * sin(angle_pitch * -0.0174533);
  compass_y_horizontal = (float)compass_y * cos(angle_roll * 0.0174533) + (float)compass_z * sin(angle_roll * 0.0174533);

  //Tinh huong (do).
  if (compass_y_horizontal < 0) actual_compass_heading = 180 + (180 + ((atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14)));
  else actual_compass_heading = (atan2(compass_y_horizontal, compass_x_horizontal)) * (180 / 3.14);

  actual_compass_heading += declination;                  //Cong do lech tu de ra bac dia ly.
  if (actual_compass_heading < 0) actual_compass_heading += 360;
  else if (actual_compass_heading >= 360) actual_compass_heading -= 360;
}

//Tinh offset / scale tu compass_cal_values[] (min/max cac truc).
//Truoc day cac gia tri nay doc tu EEPROM; nay lay tu config.h hoac tu lan hieu chuan hien tai.
void compass_compute_calibration(void) {
  bool valid = (compass_cal_values[1] > compass_cal_values[0]) &&
               (compass_cal_values[3] > compass_cal_values[2]) &&
               (compass_cal_values[5] > compass_cal_values[4]);
  if (!valid) {                                           //Tranh chia cho 0 -> NaN lam hong heading.
    DEBUG_SERIAL.println("[COMPASS] Gia tri hieu chuan khong hop le -> dung mac dinh (chua hieu chuan)");
    const int16_t def[6] = COMPASS_CAL_DEFAULT;
    for (uint8_t i = 0; i < 6; i++) compass_cal_values[i] = def[i];
  }

  compass_scale_y = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[3] - compass_cal_values[2]);
  compass_scale_z = ((float)compass_cal_values[1] - compass_cal_values[0]) / (compass_cal_values[5] - compass_cal_values[4]);

  compass_offset_x = (compass_cal_values[1] - compass_cal_values[0]) / 2 - compass_cal_values[1];
  compass_offset_y = (((float)compass_cal_values[3] - compass_cal_values[2]) / 2 - compass_cal_values[3]) * compass_scale_y;
  compass_offset_z = (((float)compass_cal_values[5] - compass_cal_values[4]) / 2 - compass_cal_values[5]) * compass_scale_z;
}

//Ghi thanh ghi cau hinh HMC5883L roi tinh he so hieu chuan.
void compass_setup(void) {
  HWire.beginTransmission(compass_address);
  HWire.write(0x00);                                      //Configuration Register A.
  HWire.write(0x78);                                      //Trung binh 8 mau, 75Hz.
  HWire.write(0x20);                                      //Configuration Register B: gain +/-1.3Ga.
  HWire.write(0x00);                                      //Mode: do lien tuc.
  HWire.endTransmission();

  compass_compute_calibration();
}

//Hieu nho nhat giua 2 goc huong (xu ly vong 0/360).
float course_deviation(float course_b, float course_c) {
  float course_a = course_b - course_c;
  if (course_a < -180 || course_a > 180) {
    float base_course_mirrored, actual_course_mirrored;
    if (course_c > 180) base_course_mirrored = course_c - 180;
    else base_course_mirrored = course_c + 180;
    if (course_b > 180) actual_course_mirrored = course_b - 180;
    else actual_course_mirrored = course_b + 180;
    course_a = actual_course_mirrored - base_course_mirrored;
  }
  return course_a;
}
