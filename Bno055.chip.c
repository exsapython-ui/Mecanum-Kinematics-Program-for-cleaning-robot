/*
  BNO055 IMU (disederhanakan) untuk Wokwi - kompatibel dengan library Adafruit_BNO055.
  I2C slave di alamat 0x28 (atribut "addr" = 41 untuk 0x29).
  Data dibangkitkan dari kontrol: yaw rate, set heading, roll, pitch.
*/
#include "wokwi-api.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define W 160                 // HARUS sama dengan display di chip.json
#define H 80
#define FRAME_US  100000      // refresh tampilan 10 fps
#define UPDATE_US 10000       // update data 100 Hz
#define PI_F  3.14159265f
#define DEG   (PI_F / 180.0f)
#define GRAV  9.81f

#define RGB(r,g,b) (0xff000000u | ((uint32_t)(b) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(r))
#define COL_BG    RGB(14, 18, 24)
#define COL_PANEL RGB(26, 33, 42)
#define COL_RING  RGB(80, 92, 106)
#define COL_DIM   RGB(110, 122, 136)
#define COL_TEXT  RGB(210, 220, 230)
#define COL_GREEN RGB(60, 220, 120)
#define COL_RED   RGB(230, 80, 80)

typedef struct {
  pin_t vin, sda, scl;
  uint32_t attr_addr, attr_yaw_rate, attr_heading, attr_roll, attr_pitch;

  uint8_t reg[256];
  uint8_t ptr;
  bool    first;                 // byte pertama transaksi tulis = pointer register

  float heading;                 // 0..360, searah jarum jam
  float last_heading_attr;
  float yaw_rate, roll, pitch;

  buffer_t  fb;
  uint32_t *px;
} chip_t;

/* ---------- matematika ---------- */
static float fsin(float x) {
  while (x >  PI_F) x -= 2 * PI_F;
  while (x < -PI_F) x += 2 * PI_F;
  if (x >  PI_F / 2) x =  PI_F - x;
  else if (x < -PI_F / 2) x = -PI_F - x;
  float x2 = x * x;
  return x * (1 + x2 * (-1.0f/6 + x2 * (1.0f/120 + x2 * (-1.0f/5040 + x2 * (1.0f/362880 + x2 * (-1.0f/39916800))))));
}
static float fcos(float x) { return fsin(x + PI_F / 2); }

/* ---------- register ---------- */
static void put16(chip_t *c, uint8_t addr, float v) {
  if (v >  32767) v =  32767;
  if (v < -32768) v = -32768;
  int16_t s = (int16_t)(v >= 0 ? v + 0.5f : v - 0.5f);
  c->reg[addr]     = (uint8_t)(s & 0xFF);
  c->reg[addr + 1] = (uint8_t)(((uint16_t)s >> 8) & 0xFF);
}

static void reset_regs(chip_t *c) {
  memset(c->reg, 0, sizeof(c->reg));
  c->reg[0x00] = 0xA0;   // CHIP_ID
  c->reg[0x01] = 0xFB;   // ACC_ID
  c->reg[0x02] = 0x32;   // MAG_ID
  c->reg[0x03] = 0x0F;   // GYR_ID
  c->reg[0x04] = 0x11;   // SW_REV_LSB
  c->reg[0x05] = 0x03;   // SW_REV_MSB
  c->reg[0x06] = 0x15;   // BL_REV
  c->reg[0x35] = 0xFF;   // CALIB_STAT: semua sensor terkalibrasi penuh
  c->reg[0x36] = 0x0F;   // ST_RESULT: self-test lulus
  c->reg[0x38] = 0x00;   // SYS_CLK_STATUS
  c->reg[0x39] = 0x05;   // SYS_STATUS: running with sensor fusion
  c->reg[0x3A] = 0x00;   // SYS_ERR
  c->reg[0x3B] = 0x80;   // UNIT_SEL: deg, dps, m/s2, C
  c->reg[0x3D] = 0x00;   // OPR_MODE: CONFIG
  c->reg[0x3E] = 0x00;   // PWR_MODE: normal
  c->reg[0x41] = 0x24;   // AXIS_MAP_CONFIG
  c->reg[0x42] = 0x00;   // AXIS_MAP_SIGN
  c->reg[0x34] = 25;     // TEMP
}

static void reg_write(chip_t *c, uint8_t addr, uint8_t v) {
  if (addr == 0x3F) {                    // SYS_TRIGGER
    if (v & 0x20) { reset_regs(c); return; }
    c->reg[addr] = v & 0x80;
    return;
  }
  if (addr == 0x07 || addr >= 0x3B) c->reg[addr] = v;   // sisanya read-only
}

/* ---------- pembangkit data sensor ---------- */
static void update_data(chip_t *c) {
  if ((c->reg[0x3D] & 0x0F) == 0) return;   // CONFIG mode: data tidak diperbarui

  const float dt = UPDATE_US / 1e6f;

  float ha = attr_read_float(c->attr_heading);
  if (ha != c->last_heading_attr) { c->heading = ha; c->last_heading_attr = ha; }
  c->yaw_rate = attr_read_float(c->attr_yaw_rate);
  c->roll     = attr_read_float(c->attr_roll);
  c->pitch    = attr_read_float(c->attr_pitch);

  c->heading += c->yaw_rate * dt;
  while (c->heading >= 360.0f) c->heading -= 360.0f;
  while (c->heading <    0.0f) c->heading += 360.0f;

  float sr = fsin(c->roll * DEG),  cr_ = fcos(c->roll * DEG);
  float sp = fsin(c->pitch * DEG), cp_ = fcos(c->pitch * DEG);

  // Gravitasi di kerangka sensor (rata = +Z)
  float gx = -GRAV * sp;
  float gy =  GRAV * sr * cp_;
  float gz =  GRAV * cr_ * cp_;

  // Kuaternion ZYX. Heading searah jarum jam => yaw matematis = -heading
  float yaw = -c->heading * DEG;
  float cyh = fcos(yaw / 2),        syh = fsin(yaw / 2);
  float cph = fcos(c->pitch*DEG/2), sph = fsin(c->pitch*DEG/2);
  float crh = fcos(c->roll *DEG/2), srh = fsin(c->roll *DEG/2);
  float qw = crh*cph*cyh + srh*sph*syh;
  float qx = srh*cph*cyh - crh*sph*syh;
  float qy = crh*sph*cyh + srh*cph*syh;
  float qz = crh*cph*syh - srh*sph*cyh;

  // Magnetometer (perkiraan, tanpa kompensasi kemiringan), satuan uT
  float mx = 25.0f * fcos(c->heading * DEG);
  float my = -25.0f * fsin(c->heading * DEG);
  float mz = -40.0f;

  put16(c, 0x08, gx * 100); put16(c, 0x0A, gy * 100); put16(c, 0x0C, gz * 100);   // ACC (m/s2 x100)
  put16(c, 0x0E, mx * 16);  put16(c, 0x10, my * 16);  put16(c, 0x12, mz * 16);    // MAG (uT x16)
  put16(c, 0x14, 0);        put16(c, 0x16, 0);                                    // GYR x,y
  put16(c, 0x18, -c->yaw_rate * 16);                                              // GYR z (aturan tangan kanan)
  put16(c, 0x1A, c->heading * 16);                                                // EUL heading
  put16(c, 0x1C, c->roll * 16);                                                   // EUL roll
  put16(c, 0x1E, c->pitch * 16);                                                  // EUL pitch
  put16(c, 0x20, qw * 16384); put16(c, 0x22, qx * 16384);
  put16(c, 0x24, qy * 16384); put16(c, 0x26, qz * 16384);                         // QUA
  put16(c, 0x28, 0); put16(c, 0x2A, 0); put16(c, 0x2C, 0);                        // LIA
  put16(c, 0x2E, gx * 100); put16(c, 0x30, gy * 100); put16(c, 0x32, gz * 100);   // GRV
}

/* ---------- I2C ---------- */
static bool on_connect(void *ud, uint32_t address, bool read) {
  (void)address;
  chip_t *c = (chip_t *)ud;
  if (!pin_read(c->vin)) return false;   // tanpa VIN: tidak menjawab (NACK)
  c->first = !read;
  return true;
}
static uint8_t on_read(void *ud) {
  chip_t *c = (chip_t *)ud;
  return c->reg[c->ptr++];
}
static bool on_write(void *ud, uint8_t data) {
  chip_t *c = (chip_t *)ud;
  if (c->first) { c->ptr = data; c->first = false; return true; }
  reg_write(c, c->ptr++, data);
  return true;
}
static void on_disconnect(void *ud) { (void)ud; }

/* ---------- gambar ---------- */
typedef struct { char c; uint8_t col[5]; } glyph_t;
static const glyph_t FONT[] = {
  {'0',{0x3E,0x51,0x49,0x45,0x3E}}, {'1',{0x00,0x42,0x7F,0x40,0x00}},
  {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4B,0x31}},
  {'4',{0x18,0x14,0x12,0x7F,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
  {'6',{0x3C,0x4A,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
  {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1E}},
  {'-',{0x08,0x08,0x08,0x08,0x08}}, {'.',{0x00,0x60,0x60,0x00,0x00}},
  {'C',{0x3E,0x41,0x41,0x41,0x22}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
  {'F',{0x7F,0x09,0x09,0x09,0x01}}, {'G',{0x3E,0x41,0x49,0x49,0x7A}},
  {'H',{0x7F,0x08,0x08,0x08,0x7F}}, {'I',{0x00,0x41,0x7F,0x41,0x00}},
  {'L',{0x7F,0x40,0x40,0x40,0x40}}, {'M',{0x7F,0x02,0x0C,0x02,0x7F}},
  {'N',{0x7F,0x04,0x08,0x10,0x7F}}, {'O',{0x3E,0x41,0x41,0x41,0x3E}},
  {'P',{0x7F,0x09,0x09,0x09,0x06}}, {'R',{0x7F,0x09,0x19,0x29,0x46}},
  {'T',{0x01,0x01,0x7F,0x01,0x01}}, {'U',{0x3F,0x40,0x40,0x40,0x3F}},
  {'Y',{0x07,0x08,0x70,0x08,0x07}},
};
#define FONT_N (sizeof(FONT) / sizeof(FONT[0]))

static void put(uint32_t *px, int x, int y, uint32_t col) {
  if (x >= 0 && x < W && y >= 0 && y < H) px[y * W + x] = col;
}
static void fill_rect(uint32_t *px, int x, int y, int w, int h, uint32_t col) {
  for (int j = y; j < y + h; j++)
    for (int i = x; i < x + w; i++) put(px, i, j, col);
}
static void disc(uint32_t *px, int cx, int cy, int r, uint32_t col) {
  for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++)
      if (dx * dx + dy * dy <= r * r) put(px, cx + dx, cy + dy, col);
}
static void ring(uint32_t *px, int cx, int cy, int r, int t, uint32_t col) {
  int inner = (r - t) * (r - t);
  for (int dy = -r; dy <= r; dy++)
    for (int dx = -r; dx <= r; dx++) {
      int d = dx * dx + dy * dy;
      if (d <= r * r && d >= inner) put(px, cx + dx, cy + dy, col);
    }
}
static int iabs(int v) { return v < 0 ? -v : v; }
static void line(uint32_t *px, int x0, int y0, int x1, int y1, int thick, uint32_t col) {
  int dx = iabs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -iabs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    fill_rect(px, x0, y0, thick, thick, col);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}
static void draw_char(uint32_t *px, int x, int y, char ch, int s, uint32_t col) {
  for (unsigned k = 0; k < FONT_N; k++) {
    if (FONT[k].c != ch) continue;
    for (int cc = 0; cc < 5; cc++)
      for (int r = 0; r < 7; r++)
        if ((FONT[k].col[cc] >> r) & 1) fill_rect(px, x + cc * s, y + r * s, s, s, col);
    return;
  }
}
static void draw_text(uint32_t *px, int x, int y, const char *t, int s, uint32_t col) {
  for (; *t; t++, x += 6 * s) draw_char(px, x, y, *t, s, col);
}
static int num_to_str(long v, char *out) {
  char tmp[16]; int n = 0, len = 0;
  bool neg = v < 0;
  unsigned long u = neg ? (unsigned long)(-v) : (unsigned long)v;
  do { tmp[n++] = '0' + (u % 10); u /= 10; } while (u);
  if (neg) out[len++] = '-';
  while (n) out[len++] = tmp[--n];
  out[len] = 0;
  return len;
}
static void fmt1(float v, char *out) {
  int n = 0;
  if (v < 0) { out[n++] = '-'; v = -v; }
  int t = (int)(v * 10 + 0.5f);
  n += num_to_str(t / 10, out + n);
  out[n++] = '.';
  out[n++] = '0' + (t % 10);
  out[n] = 0;
}

static void draw_frame(chip_t *c) {
  uint32_t *px = c->px;
  for (int i = 0; i < W * H; i++) px[i] = COL_BG;

  bool powered = pin_read(c->vin);
  uint8_t mode = c->reg[0x3D] & 0x0F;

  /* Kompas */
  const int cx = 36, cy = 40;
  disc(px, cx, cy, 31, COL_PANEL);
  ring(px, cx, cy, 32, 2, COL_RING);
  for (int k = 0; k < 4; k++) {
    float a = k * PI_F / 2;
    line(px, cx + (int)(fsin(a) * 26), cy - (int)(fcos(a) * 26),
             cx + (int)(fsin(a) * 30), cy - (int)(fcos(a) * 30), 1, COL_DIM);
  }
  draw_text(px, cx - 2, cy - 24, "N", 1, COL_DIM);

  float th = c->heading * DEG;                       // 0 = atas, searah jarum jam
  float dx = fsin(th), dy = -fcos(th);
  float px_ = fcos(th), py_ = fsin(th);              // tegak lurus
  int tipx = cx + (int)(dx * 22), tipy = cy + (int)(dy * 22);
  uint32_t ncol = powered ? COL_GREEN : COL_DIM;
  line(px, cx - (int)(dx * 10), cy - (int)(dy * 10), cx, cy, 2, COL_RED);
  line(px, cx, cy, tipx, tipy, 2, ncol);
  line(px, tipx, tipy, tipx - (int)(dx * 7 + px_ * 4), tipy - (int)(dy * 7 + py_ * 4), 1, ncol);
  line(px, tipx, tipy, tipx - (int)(dx * 7 - px_ * 4), tipy - (int)(dy * 7 - py_ * 4), 1, ncol);
  disc(px, cx, cy, 3, COL_RING);

  /* Angka */
  char buf[24];
  const int x0 = 76;
  draw_text(px, x0, 5, "HDG", 1, COL_DIM);
  fmt1(c->heading, buf);
  draw_text(px, x0, 15, buf, 2, ncol);

  draw_text(px, x0, 36, "ROL", 1, COL_DIM);
  fmt1(c->roll, buf);
  draw_text(px, x0 + 26, 36, buf, 1, COL_TEXT);

  draw_text(px, x0, 47, "PIT", 1, COL_DIM);
  fmt1(c->pitch, buf);
  draw_text(px, x0 + 26, 47, buf, 1, COL_TEXT);

  draw_text(px, x0, 58, "GYR", 1, COL_DIM);
  fmt1(c->yaw_rate, buf);
  draw_text(px, x0 + 26, 58, buf, 1, COL_TEXT);

  const char *mt = buf;
  if (!powered)         mt = "NO PWR";
  else if (mode == 0x0C) mt = "NDOF";
  else if (mode == 0x08) mt = "IMU";
  else if (mode == 0x00) mt = "CFG";
  else { buf[0] = 'M'; num_to_str(mode, buf + 1); }
  if (!powered) { draw_text(px, x0, 69, "NO", 1, COL_RED); }
  else          { draw_text(px, x0, 69, mt, 1, COL_GREEN); }

  buffer_write(c->fb, 0, px, W * H * 4);
}

/* ---------- timer ---------- */
static void data_tick(void *ud)    { update_data((chip_t *)ud); }
static void display_tick(void *ud) { draw_frame((chip_t *)ud); }

void chip_init(void) {
  chip_t *c = (chip_t *)calloc(1, sizeof(chip_t));

  c->vin = pin_init("VIN", INPUT);
  c->sda = pin_init("SDA", INPUT_PULLUP);
  c->scl = pin_init("SCL", INPUT_PULLUP);

  c->attr_addr     = attr_init("addr", 0x28);
  c->attr_yaw_rate = attr_init_float("yawRate", 0.0f);
  c->attr_heading  = attr_init_float("heading", 0.0f);
  c->attr_roll     = attr_init_float("roll", 0.0f);
  c->attr_pitch    = attr_init_float("pitch", 0.0f);

  reset_regs(c);
  c->heading = attr_read_float(c->attr_heading);
  c->last_heading_attr = c->heading;

  i2c_config_t i2c = {
    .user_data  = c,
    .address    = attr_read(c->attr_addr),
    .scl        = c->scl,
    .sda        = c->sda,
    .connect    = on_connect,
    .read       = on_read,
    .write      = on_write,
    .disconnect = on_disconnect,
  };
  i2c_init(&i2c);

  uint32_t fw, fh;
  c->fb = framebuffer_init(&fw, &fh);
  c->px = (uint32_t *)malloc(W * H * 4);

  timer_config_t t1 = { .callback = data_tick, .user_data = c };
  timer_start(timer_init(&t1), UPDATE_US, true);
  timer_config_t t2 = { .callback = display_tick, .user_data = c };
  timer_start(timer_init(&t2), FRAME_US, true);

  draw_frame(c);
}
