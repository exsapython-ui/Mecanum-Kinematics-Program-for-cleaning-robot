#include "wokwi-api.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define FRAME_US 100000   // refresh tampilan 10 fps
#define W 160             // HARUS sama dengan display di chip.json
#define H 80

#define RGB(r,g,b) (0xff000000u | ((uint32_t)(b) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(r))
#define COL_BG    RGB(14, 18, 24)
#define COL_PANEL RGB(30, 38, 48)
#define COL_DIM   RGB(110, 122, 136)
#define COL_TEXT  RGB(210, 220, 230)
#define COL_FWD   RGB(60, 220, 120)
#define COL_REV   RGB(255, 160, 50)
#define COL_BRK   RGB(230, 80, 80)
#define COL_OFF   RGB(70, 78, 88)

typedef struct { uint64_t last_t; uint32_t level; uint64_t high_ns; } duty_t;

typedef struct {
  pin_t vcc, rpwm, lpwm, r_en, l_en, r_is, l_is, bp, mp, mm;
  uint32_t attr_load;

  duty_t dr, dl;
  uint64_t win_start;
  float duty_r, duty_l;
  int rstate, lstate;       // 0 = high-Z, 1 = LOW, 2 = HIGH

  buffer_t  fb;
  uint32_t *px;
} chip_state_t;

/* ---------- Font 5x7 ---------- */
typedef struct { char c; uint8_t col[5]; } glyph_t;
static const glyph_t FONT[] = {
  {'0',{0x3E,0x51,0x49,0x45,0x3E}}, {'1',{0x00,0x42,0x7F,0x40,0x00}},
  {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4B,0x31}},
  {'4',{0x18,0x14,0x12,0x7F,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
  {'6',{0x3C,0x4A,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
  {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1E}},
  {'.',{0x00,0x60,0x60,0x00,0x00}}, {'%',{0x23,0x13,0x08,0x64,0x62}},
  {'A',{0x7E,0x11,0x11,0x11,0x7E}}, {'B',{0x7F,0x49,0x49,0x49,0x36}},
  {'D',{0x7F,0x41,0x41,0x22,0x1C}}, {'E',{0x7F,0x49,0x49,0x49,0x41}},
  {'F',{0x7F,0x09,0x09,0x09,0x01}}, {'J',{0x20,0x40,0x41,0x3F,0x01}},
  {'L',{0x7F,0x40,0x40,0x40,0x40}}, {'M',{0x7F,0x02,0x0C,0x02,0x7F}},
  {'N',{0x7F,0x04,0x08,0x10,0x7F}}, {'O',{0x3E,0x41,0x41,0x41,0x3E}},
  {'R',{0x7F,0x09,0x19,0x29,0x46}}, {'S',{0x46,0x49,0x49,0x49,0x31}},
  {'T',{0x01,0x01,0x7F,0x01,0x01}}, {'U',{0x3F,0x40,0x40,0x40,0x3F}},
};
#define FONT_N (sizeof(FONT) / sizeof(FONT[0]))

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
static void fmt_amps(float v, char *out) {
  if (v < 0) v = 0;
  int t = (int)(v * 10 + 0.5f);
  int n = num_to_str(t / 10, out);
  out[n++] = '.'; out[n++] = '0' + (t % 10); out[n++] = 'A'; out[n] = 0;
}

/* ---------- Logika half-bridge ---------- */
static void duty_update(duty_t *d, uint64_t now, uint32_t level) {
  if (d->level) d->high_ns += now - d->last_t;
  d->last_t = now;
  d->level = level;
}

static void drive_half(pin_t out, pin_t is_pin, bool en, bool pwm, bool vb,
                       float is_v, int *state) {
  if (!en) {                       // disabled: kedua FET mati, output mengambang
    pin_mode(out, INPUT);
    pin_dac_write(is_pin, 0);
    *state = 0;
    return;
  }
  pin_mode(out, OUTPUT);
  if (pwm && vb) {                 // high-side ON: M = B+
    pin_write(out, HIGH);
    pin_dac_write(is_pin, is_v);
    *state = 2;
  } else {                         // low-side ON: M = GND
    pin_write(out, LOW);
    pin_dac_write(is_pin, 0);
    *state = 1;
  }
}

static void update_outputs(chip_state_t *c) {
  bool logic = pin_read(c->vcc);
  bool vb    = pin_read(c->bp);

  float is_v = attr_read_float(c->attr_load) / 8.5f;   // 1 kΩ, k_ILIS ≈ 8500
  if (is_v < 0) is_v = 0;
  if (is_v > 5) is_v = 5;

  drive_half(c->mp, c->r_is, logic && pin_read(c->r_en), pin_read(c->rpwm),
             vb, is_v, &c->rstate);
  drive_half(c->mm, c->l_is, logic && pin_read(c->l_en), pin_read(c->lpwm),
             vb, is_v, &c->lstate);
}

static void on_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *c = (chip_state_t *)user_data;
  uint64_t now = get_sim_nanos();
  if (pin == c->rpwm)      duty_update(&c->dr, now, value);
  else if (pin == c->lpwm) duty_update(&c->dl, now, value);
  update_outputs(c);
}

/* ---------- Tampilan ---------- */
static void draw_row(uint32_t *px, int y, const char *label, bool en, float duty, uint32_t col) {
  draw_text(px, 4, y + 1, label, 1, COL_TEXT);
  disc(px, 22, y + 4, 3, en ? COL_FWD : COL_OFF);
  fill_rect(px, 32, y, 96, 9, COL_PANEL);
  if (en) fill_rect(px, 33, y + 1, (int)(duty * 94), 7, col);
  char buf[8];
  int n = num_to_str((long)(duty * 100 + 0.5f), buf);
  buf[n++] = '%'; buf[n] = 0;
  draw_text(px, 132, y + 1, buf, 1, en ? COL_TEXT : COL_DIM);
}

static void draw_frame(chip_state_t *c) {
  uint32_t *px = c->px;
  for (int i = 0; i < W * H; i++) px[i] = COL_BG;

  bool er = c->rstate != 0, el = c->lstate != 0;

  draw_text(px, 4, 5, "BTS7960", 1, COL_DIM);
  char buf[16];
  fmt_amps(attr_read_float(c->attr_load), buf);
  draw_text(px, 120, 5, buf, 1, COL_DIM);

  draw_row(px, 20, "R", er, c->duty_r, COL_FWD);
  draw_row(px, 36, "L", el, c->duty_l, COL_REV);

  const char *st = "OFF";
  uint32_t col = COL_OFF;
  if (er && el) {
    float diff = c->duty_r - c->duty_l;
    if (diff > 0.02f)       { st = "MAJU";   col = COL_FWD; }
    else if (diff < -0.02f) { st = "MUNDUR"; col = COL_REV; }
    else                    { st = "REM";    col = COL_BRK; }
  }
  draw_text(px, 4, 56, st, 2, col);

  buffer_write(c->fb, 0, px, W * H * 4);
}

static void display_tick(void *user_data) {
  chip_state_t *c = (chip_state_t *)user_data;
  uint64_t now = get_sim_nanos();
  uint64_t dur = now - c->win_start;

  duty_update(&c->dr, now, c->dr.level);
  duty_update(&c->dl, now, c->dl.level);

  if (dur > 0) {
    c->duty_r = (float)c->dr.high_ns / (float)dur;
    c->duty_l = (float)c->dl.high_ns / (float)dur;
    if (c->duty_r > 1) c->duty_r = 1;
    if (c->duty_l > 1) c->duty_l = 1;
  }
  c->dr.high_ns = 0; c->dl.high_ns = 0;
  c->win_start = now;

  update_outputs(c);   // ikuti perubahan slider beban
  draw_frame(c);
}

void chip_init(void) {
  chip_state_t *c = calloc(1, sizeof(chip_state_t));

  c->vcc   = pin_init("VCC",  INPUT);
  c->bp    = pin_init("B+",   INPUT);
  c->rpwm  = pin_init("RPWM", INPUT_PULLDOWN);
  c->lpwm  = pin_init("LPWM", INPUT_PULLDOWN);
  c->r_en  = pin_init("R_EN", INPUT_PULLDOWN);
  c->l_en  = pin_init("L_EN", INPUT_PULLDOWN);
  c->r_is  = pin_init("R_IS", ANALOG);
  c->l_is  = pin_init("L_IS", ANALOG);
  c->mp    = pin_init("M+",   INPUT);
  c->mm    = pin_init("M-",   INPUT);

  c->attr_load = attr_init_float("loadA", 5.0f);

  c->dr.level = pin_read(c->rpwm);
  c->dl.level = pin_read(c->lpwm);

  pin_watch_config_t w = { .user_data = c, .edge = BOTH, .pin_change = on_change };
  pin_watch(c->vcc,  &w);
  pin_watch(c->bp,   &w);
  pin_watch(c->rpwm, &w);
  pin_watch(c->lpwm, &w);
  pin_watch(c->r_en, &w);
  pin_watch(c->l_en, &w);

  uint32_t fw, fh;
  c->fb = framebuffer_init(&fw, &fh);
  c->px = malloc(W * H * 4);

  timer_config_t t = { .callback = display_tick, .user_data = c };
  timer_start(timer_init(&t), FRAME_US, true);

  update_outputs(c);
  draw_frame(c);
}
