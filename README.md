# RoboMower - Autonomous RTK GPS Lawnmower
ESP32-S3 firmware for a home-built robot mower.  RTK GPS for positioning, VESC motor controllers over CANBUS, and a RadioMaster TX16S for manual driving & telemetry.

Why build one when you can buy a robot mower off the shelf?  The commercial ones either want a wire burying around the garden, or cost about the same as a small car - and none of them will cut a meadow.  This one is built like a small tank, cuts at whatever height you like and knows where it is to a couple of centimetres.
It's also an interesting project!

My intention with publishing this is so you can design your own mechanical hardware - any brushless drive and blade motors will work, controlled with VESCs. I'm using cheap knock-off 75100 types from AliExpress which seem to work great.

I'm going to publish the cad files for my mechanics but they might not be all that useful as the mower was mostly built out of bits I had laying around. you could do a much better job buying the 'right' parts.

in the settings (Web UI) I've tried to cover as many permutations of the layout of wheels, tracks etc as possible.

[![Watch the RoboMower mowing the lawn](https://img.youtube.com/vi/C5biq-wlkIc/hqdefault.jpg)](https://www.youtube.com/watch?v=C5biq-wlkIc)

![The RoboMower - built like a small tank, because it basically is one](Robo-Mower.jpg)

▶ I'm going to make a cover for the electronics which will make it look tidier! 

## How it works

You teach it the garden by driving it around the edge with the RC transmitter (or drawing the boundary on the map in the WebUI - both work).  From that one polygon it works out everything else: a navigation boundary the chassis must stay inside, and a working area where the stripes go.

The path planning works just like a 3D Printer slicer - which is where I pinched the idea from.  It mows the perimeter first (like the walls of a print), then an outline around each striped region, then fills in with parallel stripes (the infill).  The outline pass matters more than you'd think: without it, anywhere the lawn edge isn't parallel to the stripes gets left as an uncut wedge.  I'd be disappointed if my 3D prints had holes like that, so the mower doesn't either.

Position comes from an RTK GPS (Quectel LC29H) fused with wheel odometry.  I tried using the IMU gyro for heading, but the ground is far too bumpy - the readings were nonsense.  The machine barely slips on grass, so differential wheel odometry between GPS fixes works much better.

If it gets stuck in long wet grass, it raises the cutting deck, has another go, and progressively lowers it again.  If the blade is working too hard, it finishes the strip at maximum height and re-traces lower.  If it hits something (detected by the IMU - no bumpers), it notes the position as an obstacle and routes around it next time.

## Hardware

* ESP32-S3 DevKitC-1, the **N16R8** version (16MB Flash / 8MB PSRAM).  Be careful here - if you flash this firmware onto a generic S3 board with different memory, you'll likely brick it and need the Espressif Flash Download Tool or JTAG to recover it.  Ask me how I know.
* DFRobot GNSS-RTK board (Quectel LC29H) + NTRIP corrections.  Gives 1-2cm accuracy when it has an RTK Fixed solution.
* SparkFun BMI270 IMU - used for tilt safety and collision detection.
* 3x VESC motor controllers on CANBUS at 250 kbit/s.  Mine are a mixed bag: two older HW4.x for the wheels (ID 1 & 2) and a HW6 for the blade (ID 3).  The older ones work fine, but make sure "Send CAN Status" is enabled in VESC Tool on ALL of them - the firmware needs the eRPM feedback, and the old boards default to silent.
* Blade motor is from a Gtech cordless mower (CLM021) - cheap, light and surprisingly tough.  The VESC limits it to 2800 RPM, so the firmware just asks for full speed and lets the VESC's current limit do the soft start.
* 13S battery, RadioMaster TX16S + ER8 receiver (ELRS), a linear actuator for cut height, and a PILZ safety relay as a proper hardware E-stop in the 48V line.  The ESP32 runs on a supercap backup so it doesn't lose its position when the PILZ fires or you swap the battery.

## Controls & monitoring

Day to day you don't need a laptop for anything.  The TX16S runs a Lua widget (in the repository) showing state, battery, blade load, GPS quality and a compass - and it beeps at you when the mower wants attention.  CH4 selects Manual/Auto/Return, CH5 records the perimeter, CH6 arms the blade, CH7 pauses.

![The TX16S running the Lua telemetry widget](TX16S.jpg)

*Everything at a glance on the transmitter - state, compass, GPS quality and blade load.  The big red button is exactly what it looks like.*

There's also a WebUI - a single HTML file that connects over Bluetooth from a phone (Chrome on Android).  That gives you a live map, perimeter editing, config settings and a diagnostics page with the raw VESC data.  If something misbehaves, the System Log on the dashboard tells you what the mower decided and why - copy & paste it if you want help.

When the battery gets low it warns you on the transmitter and the phone, then leaves the decision to you.  The VESCs look after themselves as the voltage drops, so there's no need for the firmware to panic on your behalf.

# Important Note:
If you use a different VESC for the blade, check which bytes of the CAN STATUS_5 message you're reading.  I had a bug where the firmware read the first two bytes (which are actually the tachometer) as the battery voltage.  It decoded as -0.1V, the firmware decided the battery was flat, and quietly refused to start the blade - while the transmitter cheerfully displayed "Armed & Running".  That one took a while to find.  The voltage is in bytes 4-5.
#

## A cautionary tale

During development, an early version had a bug where the wheel speed feedback read zero, so the speed controller just kept adding power - and the mower took off backwards at full speed until the fence stopped it.  One broken fence panel and a plant pot.  Fortunately no people.

The firmware now refuses to wind up the power without live feedback from the wheels - but treat this machine with respect.  It weighs a lot, and it has a spinning blade.  Keep the E-stop where you can reach it unless you're not fond of your fingers!

## Setting it up

1. Arduino IDE 2.x, board "ESP32S3 Dev Module", Partition Scheme "Huge APP (3MB No OTA)", PSRAM "OPI PSRAM".  Libraries: SparkFun BMI270 and FastLED.
2. In VESC Tool: set CAN IDs 1 (left), 2 (right), 3 (blade), baud 250 kbit/s, enable CAN status messages on all three.  Drive motors in current control, blade in speed control with a 2800 RPM limit.
3. Wiring, calibration and the full operating procedure are in manual.md.  The CRSF telemetry format (if you want to build your own display) is in telemetry.md.
4. Drive the perimeter, press Auto, and watch it go.

Thanks to Benjamin Vedder for the VESC project - which makes projects like this possible, and to the OpenMower project for convincing me RTK GPS was the right approach.

Feel free to copy, share, change - whatever you like!

NOTE:
This is a DIY, experimental project.  It is not a turn-key product, and a mistake in setup can put a heavy machine with a spinning blade somewhere you didn't want it.  If you are not confident with electronics, coding and a bit of mechanical work - please walk away.

I do not accept responsibility if it mows your flower beds, your neighbour's fence, the cat - or you!
