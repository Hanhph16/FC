#include "globals.h"


static const char *error_text(uint8_t e) {
  switch (e) {
    case 0: return "OK";
    case 1: return "MPU6050 khong phan hoi";
    case 2: return "HMC5883L khong phan hoi";
    case 3: return "MS5611 khong phan hoi / can bang ngang bi huy";
    case 4: return "chua nhan tin hieu RC";
    case 5: return "vong lap >4050us hoac ga hover thu cong khong hop le";
    case 6: return "chua phat hien cat canh khi ga dat 1750";
    case 7: return "ga luc cat canh ngoai 1400..1700";
    case 8: return "mat tin hieu RC";
    default: return "loi khong ro";
  }
}

void debug_begin(void) {
  DEBUG_PORT.begin(DEBUG_BAUD);
  //USB CDC: may tinh can ~1-2s de nhan dien cong COM sau khi reset -> cho toi da DEBUG_WAIT_HOST_MS.
  //Neu khong cam USB (bay that) thi chi mat toi da 3s luc khoi dong.
  uint32_t t0 = millis();
  while (!DEBUG_LINK_UP() && millis() - t0 < DEBUG_WAIT_HOST_MS) delay(10);
  delay(350);                                             //Cho cong Serial on dinh de khong mat du lieu.
}

//Loi nang luc khoi dong: in lap lai moi 500ms den khi reset.
void debug_halt(uint8_t code, const char *message) {
  error = code;
  while (1) {
    DEBUG_SERIAL.print("[LOI "); DEBUG_SERIAL.print(code); DEBUG_SERIAL.print("] ");
    DEBUG_SERIAL.println(message);
    delay(500);
  }
}

//Dang cho dieu kien nao do (vd receiver): in 1 lan moi 500ms, goi trong vong lap cho.
void debug_report_waiting(uint8_t code, const char *message) {
  static uint32_t next_ms = 0;
  error = code;
  if (millis() >= next_ms) {
    next_ms = millis() + 500;
    DEBUG_SERIAL.print("[CHO "); DEBUG_SERIAL.print(code); DEBUG_SERIAL.print("] ");
    DEBUG_SERIAL.print(message);
    DEBUG_SERIAL.print("  CH1-4: ");
    DEBUG_SERIAL.print(channel_1); DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.print(channel_2); DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.print(channel_3); DEBUG_SERIAL.print(' ');
    DEBUG_SERIAL.println(channel_4);
  }
}

//Ghep chuoi thu cong vao bo dem (khong dung snprintf de tiet kiem ~10-20KB flash).
static char *wp;                                          //Con tro ghi hien tai.
static char *wend;                                        //Cuoi bo dem (chua 1 byte du phong).
static void put_c(char c) { if (wp < wend) *wp++ = c; }
static void put_s(const char *t) { while (*t) put_c(*t++); }
static void put_l(long v) {
  if (v < 0) { put_c('-'); v = -v; }
  char tmp[11]; uint8_t n = 0;
  do { tmp[n++] = '0' + (v % 10); v /= 10; } while (v && n < sizeof(tmp));
  while (n) put_c(tmp[--n]);
}
static void put_f1(float v) {                             //So thuc 1 chu so thap phan.
  long t = (long)(v * 10.0f + (v >= 0 ? 0.5f : -0.5f));
  if (t < 0) { put_c('-'); t = -t; }
  put_l(t / 10); put_c('.'); put_l(t % 10);
}

//Goi 1 lan moi vong lap chinh. Khong bao gio chan lau.
void debug_update(void) {
  static char line[220];
  static uint16_t len = 0, pos = 0;
  static uint16_t divider = 0;
  static uint8_t last_error = 0;

  //0) Chua co may tinh mo cong COM -> bo dong dang gui do, khong ghi gi (tranh bi chan).
  if (!DEBUG_LINK_UP()) { len = pos = 0; return; }

  //1) Day tiep dong dang gui do (toi da DEBUG_BYTES_PER_LOOP byte).
  if (pos < len) {
    for (uint8_t i = 0; i < DEBUG_BYTES_PER_LOOP && pos < len; i++) DEBUG_SERIAL.write((uint8_t)line[pos++]);
    return;
  }

  //2) Den ky 5Hz hoac vua doi ma loi -> dung dong moi.
  bool error_changed = (error != last_error);
  if (++divider < DEBUG_INTERVAL_LOOPS && !error_changed) return;
  divider = 0;
  last_error = error;

  wp = line; wend = line + sizeof(line) - 3;
  put_s("st="); put_l(start); put_s(" fm="); put_l(flight_mode); put_s(" err="); put_l(error);
  put_s(" | CH ");
  put_l(channel_1); put_c(' '); put_l(channel_2); put_c(' '); put_l(channel_3); put_c(' ');
  put_l(channel_4); put_c(' '); put_l(channel_5); put_c(' '); put_l(channel_6);
  put_s(" | R"); put_f1(angle_roll); put_s(" P"); put_f1(angle_pitch); put_s(" Y"); put_f1(angle_yaw);
  put_s(" H"); put_f1(actual_compass_heading);
  put_s(" | Pa="); put_l((long)actual_pressure);
  put_s(" | ESC "); put_l(esc_1); put_c(' '); put_l(esc_2); put_c(' '); put_l(esc_3); put_c(' '); put_l(esc_4);
  put_s(" | loop="); put_l((long)loop_timer_capture); put_s("us");
  if (error_changed && error != 0) {
    put_s(" <<ERR "); put_l(error); put_s(": "); put_s(error_text(error)); put_s(">>");
  }
  wend += 2;                                              //Dung phan du phong cho \r\n.
  put_c('\r'); put_c('\n');
  len = wp - line;
  pos = 0;
}
