# 2HPico LFO

Implemented as looping envelope generator - change the rise and fall times vs setting frequency directly which allows for a wide variety of waveshapes and duty cycles

Rise and Fall controls are exponential which gives a very wide frequency range approx 50hz to around 30 minutes per cycle

LED color changes to indicate the waveform. 

LED brightness changes with the waveform level. Not affected by level setting - otherwise you wouldn't see anything at low output levels

150mHz RP2350 clock works @ 44khz sampling rate - lots of CPU cycles to spare.

**Dependencies:**

Adafruit Neopixel library

2HPicolib support library in this repository


**Usage:**


Top Jack - sync input - resets to rising phase on rising clock edge. Sine and Ramp reset to 0v output which is half way thru rising phase

Middle jack - bipolar LFO output -5v to +5v range (provided you have the board strapped for DAC out on jack 2)

Bottom Jack - bipolar LFO output -5v to +5v range

Top pot - Rise time

Second pot - Fall Time

Third pot - Waveform - full CCW triangle/ramp (Red), sine (Orange), exponential (Green), reverse exponential (Blue), random amplitude pulse (Aqua), full CW pulse (Violet)

Fourth pot - Output level
