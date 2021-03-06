# Lidar-Auto-Focus-BMMCC-Edition
Blackmagic Micro Cinema Camera Controller + Lidar Auto Focus
## Background

Blackmagic Micro Cinema Camera is a small but great camera which can capture RAW up to 60fps. Since it's a cinema camera, BMMCC lacks of convienient features, especially auto focus. When you have a focus puller, it wouldn't be a problem, but unfortunately I don't have. I'm one-man band type of cinematographer.

The problem gets even worse, when gyro gimbals become popular. I bought Ronin-S, and it's almost impossible to keep the sharp focus while having dynamic camera moves.

There's some range-finders in the market, and the most famous one would be "Cine-tape." Not only the one, but most rangefinders use ultra-sound technology to measure the distance - it's quite accurate and no harm to human eye. However, I knew lots of drones use Lidar sensor to keep the minimal altitude. Nowadays most car manufacturers have radar/lidar option. Redrock Micro uses Lidar for its rangefinder/autofocus. Then, I thought, why not?

BMMCC anyway needed external controller to ease camera settings. All I need was to find an affordable Lidar sensor to add on. Benewake makes a cheap but decent enough Lidar for under $100. It measures up to 12meters. Accuracy up to 1cm. Thinking most of my shooting is under 5-6 meters, this sensor works well. Especially when using gimbal, I use wide lenses. If more than 12m, focusing is not that difficult.

Eye health problem-wise, I found that the lidar I use is quite low power. Plus, it adjusts power depends on the return signal strength and distance. However, it is still a good practice that not point the lidar signal to human's eye. 
## Proof of Concept
I tested the concept with TF-MINI Lidar and Arduino Pro 328 16MHz. Software serial skips about half of received data but it's not a problem. A servo was hooked up and voila! the servo moves as distance changes.

You may find the test code in \test\lidar.
## Coding
For the BMMCC control part, [BMC-SBUS library](https://github.com/boldstelvis/BMC_SBUS) saved a lot of time. Other BMMCC controllers use the library, too.

Typical choice of [KY-040 rotary encoder](https://github.com/Billwilliams1952/KY-040-Encoder-Library---Arduino) and cheap chinese OLED ($3 from ebay) works well with [Adafruit's library](https://github.com/adafruit/Adafruit_SSD1306).

Camera Lens' focus scale is not linear and somewhat weird log-like shape. Spline curve would make this shape and find any value on the curve. [arduino-splines](https://github.com/kerinin/arduino-splines) would make this possible.

## Moving up
After the first version was written, I found that it's just big, and I needed more serial connections. I have moved up to Cortex and Teensy LC just fit.

## Further works
* Use of eeprom - not recommended for Teensy LC. This MCU uses emulated EEPROM. Which stops all process while I/O.
* Sensor-flange distance adjustment
* Lens scale - ft-in / meter
* Additional features like, automatic lens calibration, stepper motor support, brushless-encoder support, multiple lens support, SD card option...