#ifndef CONFIG_H
#define CONFIG_H

// =====================================================================================
//  CAU HINH NGUOI DUNG (tunables) - sua o day, khong can dung vao code logic
// =====================================================================================

// ---------- Serial Monitor (theo doi / kiem tra) ----------
// Log di qua cong micro-USB cua Blue Pill (USB CDC, chan PA11/PA12) -> hien thanh cong COM tren may tinh.
// Can build_flags = -D SERIAL_USB trong platformio.ini (da them san).
// Neu muon dung UART that (USB-TTL tren PA9/PA10): dat DEBUG_USB_CDC 0 va DEBUG_PORT Serial1.
#define DEBUG_USB_CDC           1
#define DEBUG_PORT              Serial  // cong that cua core (USB CDC khi co -D SERIAL_USB)
#define DEBUG_SERIAL            dbg     // doi tuong boc: tu bo qua khi chua co may tinh mo cong COM (xem globals.h)
#define DEBUG_BAUD              57600   // USB CDC bo qua baud, nhung van dat trung monitor_speed
#define DEBUG_INTERVAL_LOOPS    50      // in 1 dong log moi 50 vong lap (50 x 4ms = 5 Hz)
#define DEBUG_BYTES_PER_LOOP    4       // so byte day ra Serial moi vong lap (giu loop 4ms)
#define DEBUG_WAIT_HOST_MS      3000    // cho toi da 3s luc khoi dong de may tinh mo cong COM (khong co PC thi bo qua)

// Kiem tra da co may tinh mo cong COM chua. Khi chua co, KHONG ghi gi ra Serial de tranh vong lap 4ms bi chan.
#if DEBUG_USB_CDC
  #define DEBUG_LINK_UP()       (!(!DEBUG_PORT))
#else
  #define DEBUG_LINK_UP()       (true)
#endif

// ---------- PID (thay cho gia tri doc tu EEPROM) ----------
// Yaw
#define PID_P_GAIN_YAW          4.0f
#define PID_I_GAIN_YAW          0.02f
#define PID_D_GAIN_YAW          0.0f
#define PID_MAX_YAW             400
// Altitude (giu do cao bang MS5611)
#define PID_P_GAIN_ALTITUDE     1.6f
#define PID_I_GAIN_ALTITUDE     0.25f
#define PID_D_GAIN_ALTITUDE     0.8f
#define PID_MAX_ALTITUDE        400
// Roll (pitch dung chung he so voi roll)
#define PID_P_GAIN_ROLL         1.3f
#define PID_I_GAIN_ROLL         0.04f
#define PID_D_GAIN_ROLL         18.0f
#define PID_MAX_ROLL            400

// ---------- Dong co ----------
#define MOTOR_IDLE_SPEED        1100    // xung ESC khi quay cham (1000..1200 us)
#define MANUAL_TAKEOFF_THROTTLE 0       // 0 = tu dong phat hien cat canh; 1400..1600 = ga hover thu cong

// ---------- La ban ----------
#define COMPASS_DECLINATION     2.0f    // do lech tu (do). Tra theo vi tri tai NOAA roi sua lai.

// ---------- Dia chi I2C ----------
#define MPU6050_ADDRESS         0x68
#define MS5611_ADDRESS          0x77
#define HMC5883L_ADDRESS        0x1E

// ---------- Gia tri hieu chuan (thay cho EEPROM 24LC256) ----------
// Sau khi hieu chuan bang cap gat, Serial Monitor se in ra dong can dan vao day.
// La ban: {x_min, x_max, y_min, y_max, z_min, z_max}. Gia tri mac dinh = chua hieu chuan (he so 1, lech 0).
#define COMPASS_CAL_DEFAULT     { -100, 100, -100, 100, -100, 100 }
// Gia toc ke (can bang ngang): offset pitch / roll
#define ACC_PITCH_CAL_DEFAULT   0
#define ACC_ROLL_CAL_DEFAULT    0

#endif // CONFIG_H
