/*
 * Adafruit_NeoPixel.h — STM32 shim for the WS2812FX library
 *
 * Replaces the real Adafruit_NeoPixel Arduino library with a minimal
 * implementation that:
 *   - Provides the same class interface used by WS2812FX
 *   - Maintains a raw GRB pixel buffer in RAM
 *   - Provides no-op show() so WS2812FX::customShow drives actual output
 *   - Supplies the Arduino compatibility macros WS2812FX expects
 */

#ifndef ADAFRUIT_NEOPIXEL_H
#define ADAFRUIT_NEOPIXEL_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Arduino compatibility types
// ---------------------------------------------------------------------------
typedef uint8_t  byte;
typedef uint16_t neoPixelType;
// __FlashStringHelper maps to const char on STM32 (no PROGMEM needed)
typedef const char __FlashStringHelper;

// ---------------------------------------------------------------------------
// Arduino compatibility macros
// ---------------------------------------------------------------------------
#ifndef constrain
#define constrain(v, lo, hi) \
    ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))
#endif

#ifndef map
#define map(v, il, ih, ol, oh) \
    ((long)((v) - (il)) * ((oh) - (ol)) / ((ih) - (il)) + (ol))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef abs
#define abs(x) ((x) < 0 ? -(x) : (x))
#endif

// PROGMEM & friends: no-ops on STM32
#define PROGMEM
#define pgm_read_byte(p)   (*(const uint8_t *)(p))
#define pgm_read_word(p)   (*(const uint16_t*)(p))
#define F(s)               (s)
#ifdef FSH
#undef FSH
#endif
#define FSH(x)             ((const __FlashStringHelper*)(x))

// ---------------------------------------------------------------------------
// NeoPixel type flags (only NEO_GRB used in this project)
// ---------------------------------------------------------------------------
#define NEO_RGB   0x0006  // wOffset = 3 (≠ rOffset → 4-byte; not used)
#define NEO_GRB   0x0041  // gOffset=0, rOffset=1, bOffset=2 (3-byte)

// ---------------------------------------------------------------------------
// Adafruit_NeoPixel base class
// ---------------------------------------------------------------------------
class Adafruit_NeoPixel {
 public:
  // Public data members accessed directly by WS2812FX
  uint16_t numLEDs;     // Number of pixels
  uint16_t numBytes;    // Size of pixel buffer in bytes
  uint8_t *pixels;      // Pixel buffer (GRB layout)
  uint8_t  rOffset;     // Byte index of Red   within one pixel
  uint8_t  gOffset;     // Byte index of Green within one pixel
  uint8_t  bOffset;     // Byte index of Blue  within one pixel
  uint8_t  wOffset;     // == rOffset for 3-byte (RGB/GRB); else White byte
  uint8_t  brightness;  // 0 = full; stored as raw value (no +1 offset)

  Adafruit_NeoPixel(uint16_t num_leds, uint8_t pin, neoPixelType type);
  virtual ~Adafruit_NeoPixel();

  // Lifecycle
  void begin(void);
  virtual void show(void);       // no-op; output driven by customShow
  void updateLength(uint16_t n); // re-allocate pixel buffer

  // Color setters — WS2812FX overrides these to add gamma
  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
  void setPixelColor(uint16_t n, uint32_t c);

  // Color getters
  uint32_t getPixelColor(uint16_t n) const;

  // Buffer access
  uint8_t* getPixels(void) const { return pixels; }
  uint16_t numPixels(void) const { return numLEDs; }

  // Brightness (not applied in show — caller manages it in customShow)
  void    setBrightness(uint8_t b) { brightness = b; }
  uint8_t getBrightness(void) const { return brightness; }

  // Zero the pixel buffer
  void clear(void) { if (pixels) memset(pixels, 0, numBytes); }

  // Pixel blend utility used by WS2812FXT transition
  uint8_t* blend(uint8_t* dest, uint8_t* src1, uint8_t* src2,
                 uint16_t count, uint8_t amount);

  // Gamma correction (2.6 power curve, same curve as Adafruit)
  static uint8_t gamma8(uint8_t x);

  static uint8_t sine8(uint8_t x);

  static void randomSeed(unsigned long seed) {
    srand((unsigned int)seed);
  }

  static long random(long max) {
    if (max <= 0) return 0;
    return rand() % max;
  }

  static long random(long min, long max) {
    if (min >= max) return min;
    return min + random(max - min);
  }

};

#endif  // ADAFRUIT_NEOPIXEL_H
