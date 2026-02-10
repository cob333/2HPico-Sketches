# Sketches for the 2HPico-Eurorack-Module-Hardware

Some sample sketches that run on the 2HPico eurorack module

You must have Arduino 2.xx installed with the Pico board support package https://github.com/earlephilhower/arduino-pico

Select board type as Raspberry Pi Pico or Raspberry Pico Pico 2 depending on what board you used when building the module. Some sketches require overclocking - check the readme files.

Dependencies:

2HPico library included in this repository - install it in your Arduino/Libraries directory

Adafruit Neopixel library

Some sketches use this fork of ElectroSmith's DaisySP library https://github.com/rheslip/DaisySP_Teensy

What's the difference with the original 2HPico Sketches?
more color options; 
calibration sketches;
more function:
1. 16Step_Sequencer: 4 pages of step pitch editing (16 total steps), per-step ratchet (1-8 via long-press edit), and auto-reset to step 1 after clock timeout;
2. Motion_Recorder: record your knob tweaks as looping CV motion on both outputs, with queued re-record (start next loop) and clock-timeout reset behavior;
3. Bassdrum: Uninified 909 and 808 bassdrum model;
