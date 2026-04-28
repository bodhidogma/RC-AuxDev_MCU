/*
 * Adafruit_NeoPixel.cpp — STM32 shim implementation
 */

#include "Adafruit_NeoPixel.h"

#include <mymain.h>

namespace {

static uint32_t mix32(uint32_t x) {
  // Small integer hash (Murmur3-style finalizer) to avalanche entropy bits.
  x ^= x >> 16;
  x *= 0x7feb352dU;
  x ^= x >> 15;
  x *= 0x846ca68bU;
  x ^= x >> 16;
  return x;
}

static uint32_t makeSeed(const void* self, uint16_t num_leds, uint16_t num_bytes) {
  uint32_t seed = 0xA5C31F27U;
  seed ^= (uint32_t)HAL_GetTick();
  seed ^= (uint32_t)(uintptr_t)self;
  seed ^= ((uint32_t)num_leds << 16) | (uint32_t)num_bytes;

#if defined(UID_BASE)
  volatile const uint32_t* uid = (volatile const uint32_t*)UID_BASE;
  seed ^= uid[0] ^ uid[1] ^ uid[2];
#endif

#if defined(SysTick)
  seed ^= (uint32_t)SysTick->VAL;
#endif

#if defined(DWT)
  seed ^= (uint32_t)DWT->CYCCNT;
#endif

  return mix32(seed);
}

}  // namespace

static const uint8_t PROGMEM _NeoPixelSineTable[256] = {
    128, 131, 134, 137, 140, 143, 146, 149, 152, 155, 158, 162, 165, 167, 170,
    173, 176, 179, 182, 185, 188, 190, 193, 196, 198, 201, 203, 206, 208, 211,
    213, 215, 218, 220, 222, 224, 226, 228, 230, 232, 234, 235, 237, 238, 240,
    241, 243, 244, 245, 246, 248, 249, 250, 250, 251, 252, 253, 253, 254, 254,
    254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 254, 253, 253, 252, 251,
    250, 250, 249, 248, 246, 245, 244, 243, 241, 240, 238, 237, 235, 234, 232,
    230, 228, 226, 224, 222, 220, 218, 215, 213, 211, 208, 206, 203, 201, 198,
    196, 193, 190, 188, 185, 182, 179, 176, 173, 170, 167, 165, 162, 158, 155,
    152, 149, 146, 143, 140, 137, 134, 131, 128, 124, 121, 118, 115, 112, 109,
    106, 103, 100, 97,  93,  90,  88,  85,  82,  79,  76,  73,  70,  67,  65,
    62,  59,  57,  54,  52,  49,  47,  44,  42,  40,  37,  35,  33,  31,  29,
    27,  25,  23,  21,  20,  18,  17,  15,  14,  12,  11,  10,  9,   7,   6,
    5,   5,   4,   3,   2,   2,   1,   1,   1,   0,   0,   0,   0,   0,   0,
    0,   1,   1,   1,   2,   2,   3,   4,   5,   5,   6,   7,   9,   10,  11,
    12,  14,  15,  17,  18,  20,  21,  23,  25,  27,  29,  31,  33,  35,  37,
    40,  42,  44,  47,  49,  52,  54,  57,  59,  62,  65,  67,  70,  73,  76,
    79,  82,  85,  88,  90,  93,  97,  100, 103, 106, 109, 112, 115, 118, 121,
    124};

// ---------------------------------------------------------------------------
// Gamma correction table (Adafruit standard, gamma ≈ 2.6)
// ---------------------------------------------------------------------------
static const uint8_t PROGMEM _gamma8_table[256] = {
      0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
      0,  0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  1,  1,
      1,  1,  1,  1,  2,  2,  2,  2,  2,  2,  2,  2,  3,  3,  3,  3,
      3,  3,  4,  4,  4,  4,  5,  5,  5,  5,  5,  6,  6,  6,  6,  7,
      7,  7,  8,  8,  8,  9,  9,  9, 10, 10, 10, 11, 11, 11, 12, 12,
     13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19, 20,
     20, 21, 21, 22, 22, 23, 24, 24, 25, 25, 26, 27, 27, 28, 29, 29,
     30, 31, 31, 32, 33, 34, 34, 35, 36, 37, 38, 38, 39, 40, 41, 42,
     42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57,
     58, 59, 60, 61, 62, 63, 64, 65, 66, 68, 69, 70, 71, 72, 73, 75,
     76, 77, 78, 80, 81, 82, 84, 85, 86, 88, 89, 90, 92, 93, 94, 96,
     97, 99,100,102,103,105,106,108,109,111,112,114,115,117,119,120,
    122,124,125,127,129,130,132,134,136,137,139,141,143,145,146,148,
    150,152,154,156,158,160,162,164,166,168,170,172,174,176,178,180,
    182,184,186,188,191,193,195,197,199,202,204,206,209,211,213,215,
    218,220,223,225,227,230,232,235,237,240,242,245,247,250,252,255
};

// ---------------------------------------------------------------------------
// Constructor / Destructor
// ---------------------------------------------------------------------------
Adafruit_NeoPixel::Adafruit_NeoPixel(uint16_t num_leds,
                                     uint8_t  /*pin*/,
                                     neoPixelType /*type*/) {
  numLEDs    = num_leds;
  // NEO_GRB 3-byte layout: G=0, R=1, B=2, wOffset==rOffset (marks 3-byte)
  gOffset    = 0;
  rOffset    = 1;
  bOffset    = 2;
  wOffset    = rOffset;  // same as rOffset → 3-byte pixel
  numBytes   = numLEDs * 3;
  pixels     = (uint8_t*)calloc(numBytes, 1);
  brightness = 0;  // 0 = full brightness

  // Seed libc RNG for Arduino-compatible random() helpers used by WS2812FX.
  // Seed once to avoid resetting PRNG sequence when multiple strips are created.
  static bool rng_seeded = false;
  if (!rng_seeded) {
    randomSeed(makeSeed(this, num_leds, numBytes));
    rng_seeded = true;
  }
}

Adafruit_NeoPixel::~Adafruit_NeoPixel() {
  free(pixels);
  pixels = nullptr;
}

// ---------------------------------------------------------------------------
// Lifecycle
// ---------------------------------------------------------------------------
void Adafruit_NeoPixel::begin(void) {
  // no hardware to initialise — SPI/DMA handled by DevWS2812
}

void Adafruit_NeoPixel::show(void) {
  // Intentional no-op: WS2812FX::execShow() calls customShow instead
}

void Adafruit_NeoPixel::updateLength(uint16_t n) {
  free(pixels);
  numLEDs  = n;
  numBytes = n * ((wOffset == rOffset) ? 3 : 4);
  pixels   = (uint8_t*)calloc(numBytes, 1);
}

// ---------------------------------------------------------------------------
// Pixel color setters
// ---------------------------------------------------------------------------
void Adafruit_NeoPixel::setPixelColor(uint16_t n,
                                      uint8_t r, uint8_t g, uint8_t b) {
  setPixelColor(n, r, g, b, 0);
}

void Adafruit_NeoPixel::setPixelColor(uint16_t n,
                                      uint8_t r, uint8_t g, uint8_t b,
                                      uint8_t w) {
  if (n < numLEDs && pixels) {
    uint8_t *p = (wOffset == rOffset)
                 ? &pixels[n * 3]
                 : &pixels[n * 4];
    p[rOffset] = r;
    p[gOffset] = g;
    p[bOffset] = b;
    if (wOffset != rOffset) p[wOffset] = w;
  }
}

void Adafruit_NeoPixel::setPixelColor(uint16_t n, uint32_t c) {
  uint8_t w = (uint8_t)(c >> 24);
  uint8_t r = (uint8_t)(c >> 16);
  uint8_t g = (uint8_t)(c >>  8);
  uint8_t b = (uint8_t)(c      );
  setPixelColor(n, r, g, b, w);
}

// ---------------------------------------------------------------------------
// Pixel color getter
// ---------------------------------------------------------------------------
uint32_t Adafruit_NeoPixel::getPixelColor(uint16_t n) const {
  if (n >= numLEDs || !pixels) return 0;
  if (wOffset == rOffset) {
    uint8_t *p = &pixels[n * 3];
    return ((uint32_t)p[rOffset] << 16)
         | ((uint32_t)p[gOffset] <<  8)
         |  (uint32_t)p[bOffset];
  } else {
    uint8_t *p = &pixels[n * 4];
    return ((uint32_t)p[wOffset] << 24)
         | ((uint32_t)p[rOffset] << 16)
         | ((uint32_t)p[gOffset] <<  8)
         |  (uint32_t)p[bOffset];
  }
}

// ---------------------------------------------------------------------------
// Blend utility
// ---------------------------------------------------------------------------
uint8_t* Adafruit_NeoPixel::blend(uint8_t* dest,
                                   uint8_t* src1,
                                   uint8_t* src2,
                                   uint16_t count,
                                   uint8_t  amount) {
  for (uint16_t i = 0; i < count; i++) {
    dest[i] = (uint8_t)(((uint16_t)src1[i] * (255u - amount)
                        + (uint16_t)src2[i] * amount) >> 8);
  }
  return dest;
}

// ---------------------------------------------------------------------------
// Gamma correction
// ---------------------------------------------------------------------------
uint8_t Adafruit_NeoPixel::gamma8(uint8_t x) {
  return pgm_read_byte(&_gamma8_table[x]);
}

uint8_t Adafruit_NeoPixel::sine8(uint8_t x) {
  return pgm_read_byte(&_NeoPixelSineTable[x]); // 0-255 in, 0-255 out
}
