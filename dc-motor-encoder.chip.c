#include "wokwi-api.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define STEP_US   100     // langkah fisika
#define FRAME_US  40000   // refresh tampilan (25 fps)
#define W 160             // HARUS sama dengan display di chip.json
#define H 80

#define RGB(r,g,b) (0xff000000u | ((uint32_t)(b) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(r))
#define PI_F 3.14159265f

#define COL_BG     RGB(14, 18, 24)
#define COL_PANEL  RGB(26, 33, 42)
#define COL_RING   RGB(80, 92, 106)
#define COL_DIM    RGB(110, 122, 136)
#define COL_TEXT   RGB(200, 210, 220)
#define COL_FWD    RGB(60, 220, 120)
#define COL_REV    RGB(255, 160, 50)
#define COL_IDLE   RGB(150, 160, 170)

typedef struct {
  pin_t vcc, in1, in2, enca, encb;
  uint32_t attr_max_rpm, attr_ppr, attr_tau;

  float     omega;       // putaran/detik
  double    angle;       // posisi (putaran)
  long long cnt;         // hitungan encoder (x4)
  int       last_state;

  buffer_t  fb;
  uint32_t *px;
} chip_state_t;

static const int A_TAB[4] = {0, 1, 1, 0};
static const int B_TAB[4] = {0, 0, 1, 1};

/* ---------- Font 5x7 (kolom, bit0 = baris atas) ---------- */
typedef struct { char c; uint8_t col[5]; } glyph_t;
static const glyph_t FONT[] = {
  {'0',{0x3E,0x51,0x49,0x45,0x3E}}, {'1',{0x00,0x42,0x7F,0x40,0x00}},
  {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4B,0x31}},
  {'4',{0x18,0x14,0x12,0x7F,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
  {'6',{0x3C,0x4A,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
  {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1E}},
  {'-',{0x08,0x08,0x08,0x08,0x08}},
  {'R',{0x7F,0x09,0x19,0x29,0x46}}, {'P',{0x7F,0x09,0x09,0x09,0x06}},
  {'M',{0x7F,0x02,0x0C,0x02,0x7F}}, {'C',{0x3E,0x41,0x41,0x41,0x22}},
  {'N',{0x7F,0x04,0x08,0x10,0x7F}}, {'T',{0x01,0x01,0x7F,0x01,0x01}},
  {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'J',{0x20,0x40,0x41,0x3F,0x01}},
  {'U',{0x3F,0x40,0x40,0x40,0x3F}}, {'D',{0x7F,0x41,0x41,0x22,0x1C}},
  {'S',{0x46,0x49,0x49,0x49,0x31}}, {'O',{0x3E,0x41,0x41,0x41,0x3E}},
};
#define FONT_N (sizeof(FONT) / sizeof(FONT[0]))

/* ---------- sin/cos sederhana (tanpa math.h) ---------- */
static float fsin(float x) {
  while (x >  PI_F) x -= 2 * PI_F;
  while (x < -PI_F) x += 2 * PI_F;
  float x2 = x * x;
  return x * (1 - x2/6 * (1 - x2/20 * (1 - x2/42 * (1 - x2/72 * (1 - x2/110)))));
}
static float fcos(float x) { return fsin(x + PI_F / 2); }

/* ---------- Primitif gambar ---------- */
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

static void line(uint32_t *px, int x0, int y0, int x1, int y1, int thick, uint32_t col) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
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
    for (int c = 0; c < 5; c++)
      for (int r = 0; r < 7; r++)
        if ((FONT[k].col[c] >> r) & 1) fill_rect(px, x + c * s, y + r * s, s, s, col);
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

/* ---------- Render tampilan ---------- */
static void draw_frame(chip_state_t *c) {
  uint32_t *px = c->px;
  for (int i = 0; i < W * H; i++) px[i] = COL_BG;

  float rpm    = c->omega * 60.0f;
  float maxrpm = attr_read_float(c->attr_max_rpm);
  if (maxrpm < 1) maxrpm = 1;
  uint32_t accent = rpm > 0.5f ? COL_FWD : (rpm < -0.5f ? COL_REV : COL_IDLE);

  /* Cakram motor */
  const int cx = 38, cy = 40;
  disc(px, cx, cy, 31, COL_PANEL);
  ring(px, cx, cy, 32, 2, COL_RING);
  for (int k = 0; k < 12; k++) {                      // tanda skala
    float a = k * PI_F / 6;
    line(px, cx + (int)(fcos(a) * 26), cy + (int)(fsin(a) * 26),
             cx + (int)(fcos(a) * 29), cy + (int)(fsin(a) * 29), 1, COL_DIM);
  }

  double a  = c->angle;
  long long ia = (long long)a; if (a < (double)ia) ia--;
  float theta = (float)(a - (double)ia) * 2 * PI_F;   // 0..2π, searah jarum jam

  for (int k = 3; k >= 0; k--) {                      // jari-jari (k=0 = penanda)
    float ang = theta + k * PI_F / 2;
    int len = (k == 0) ? 27 : 23;
    line(px, cx, cy, cx + (int)(fcos(ang) * len), cy + (int)(fsin(ang) * len),
         k == 0 ? 2 : 1, k == 0 ? accent : COL_DIM);
  }
  disc(px, cx, cy, 5, COL_RING);
  disc(px, cx, cy, 2, COL_BG);

  /* Panel kanan */
  const int x0 = 80;
  draw_text(px, x0, 6, "RPM", 1, COL_DIM);

  char buf[24];
  long r = (long)(rpm + (rpm >= 0 ? 0.5f : -0.5f));
  int len = num_to_str(r, buf);
  int s = (len <= 4) ? 3 : 2;
  draw_text(px, x0, 17, buf, s, accent);

  const char *st = rpm > 0.5f ? "MAJU" : (rpm < -0.5f ? "MUNDUR" : "STOP");
  draw_text(px, x0, 44, st, 1, accent);

  draw_text(px, x0, 56, "CNT", 1, COL_DIM);
  num_to_str((long)c->cnt, buf);
  draw_text(px, x0 + 24, 56, buf, 1, COL_TEXT);

  /* Bar kecepatan */
  fill_rect(px, x0, 68, 74, 7, COL_PANEL);
  float f = (rpm < 0 ? -rpm : rpm) / maxrpm;
  if (f > 1) f = 1;
  fill_rect(px, x0 + 1, 69, (int)(f * 72), 5, accent);

  buffer_write(c->fb, 0, px, W * H * 4);
}

/* ---------- Fisika motor + encoder ---------- */
static void physics_tick(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  const float dt = STEP_US / 1e6f;

  float max_rps = attr_read_float(c->attr_max_rpm) / 60.0f;
  float tau     = attr_read_float(c->attr_tau) / 1000.0f;
  float ppr     = attr_read_float(c->attr_ppr);
  if (tau < 0.001f) tau = 0.001f;

  bool vcc = pin_read(c->vcc);
  bool a   = pin_read(c->in1);
  bool b   = pin_read(c->in2);

  float target = 0.0f;
  float t = tau;

  if (vcc) {
    if (a && !b)      target =  max_rps;
    else if (!a && b) target = -max_rps;
    else if (a && b)  t = tau * 0.2f;   // rem
    else              t = tau * 4.0f;   // coast
  } else {
    t = tau * 4.0f;
  }

  c->omega += (target - c->omega) * dt / t;
  c->angle += (double)c->omega * dt;

  double counts_f = c->angle * ppr * 4.0;
  long long cnt = (long long)counts_f;
  if ((double)cnt > counts_f) cnt--;
  c->cnt = cnt;
  int state = (int)(((cnt % 4) + 4) % 4);

  if (state != c->last_state) {
    pin_write(c->enca, A_TAB[state]);
    pin_write(c->encb, B_TAB[state]);
    c->last_state = state;
  }
}

static void display_tick(void *user_data) {
  draw_frame((chip_state_t *)user_data);
}

void chip_init(void) {
  chip_state_t *c = malloc(sizeof(chip_state_t));

  c->vcc  = pin_init("VCC",  INPUT);
  c->in1  = pin_init("IN1",  INPUT_PULLDOWN);
  c->in2  = pin_init("IN2",  INPUT_PULLDOWN);
  c->enca = pin_init("ENCA", OUTPUT_LOW);
  c->encb = pin_init("ENCB", OUTPUT_LOW);

  c->attr_max_rpm = attr_init_float("maxRpm", 200.0f);
  c->attr_ppr     = attr_init_float("ppr", 11.0f);
  c->attr_tau     = attr_init_float("tauMs", 80.0f);

  c->omega = 0; c->angle = 0; c->cnt = 0; c->last_state = 0;

  uint32_t w, h;
  c->fb = framebuffer_init(&w, &h);
  c->px = malloc(W * H * 4);

  timer_config_t phys = { .callback = physics_tick, .user_data = c };
  timer_start(timer_init(&phys), STEP_US, true);

  timer_config_t disp = { .callback = display_tick, .user_data = c };
  timer_start(timer_init(&disp), FRAME_US, true);

  draw_frame(c);
}
