// Contributor: Wenhao Yang

// LED color test for 2HPico
// Cycles through all predefined colors (including WHITE/GREY).

#include <2HPico.h>
#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel LEDS(NUMPIXELS, LEDPIN, NEO_GRB + NEO_KHZ800);

struct ColorItem {
  const char *name;
  uint32_t color;
  uint16_t hold_ms;
};

static const ColorItem kColors[] = {
  { "OFF",   0,     300 },
  { "RED",   RED,   900 },
  { "ORANGE", ORANGE, 900 },
  { "YELLOW", YELLOW, 900 },
  { "GREEN", GREEN, 900 },
  { "TIFFANY", TIFFANY, 900 },
  { "AQUA", AQUA,  900 },
  { "BLUE",  BLUE,  900 },
  { "VIOLET", VIOLET, 900 },
  { "WHITE", WHITE, 900 },
  { "GREY",  GREY,  900 },
};

static uint8_t color_index = 0;
static uint32_t buttontimer = 0;
static bool button = 0;

static void applyColor() {
  LEDS.setPixelColor(0, kColors[color_index].color);
  LEDS.show();
  Serial.print("LED: ");
  Serial.println(kColors[color_index].name);
}

void setup() {
  Serial.begin(115200);

  pinMode(BUTTON1, INPUT_PULLUP);

  LEDS.begin();
  LEDS.setPixelColor(0, 0);
  LEDS.show();

  applyColor();
}

void loop() {
  // Button: short press to step to next color
  if (!digitalRead(BUTTON1)) {
    if (((millis() - buttontimer) > DEBOUNCE) && !button) {
      button = 1;
      color_index = (color_index + 1) % (sizeof(kColors) / sizeof(kColors[0]));
      applyColor();
    }
  } else {
    buttontimer = millis();
    button = 0;
  }
}
