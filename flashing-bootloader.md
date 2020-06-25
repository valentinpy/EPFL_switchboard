# Switchboard: general modifications of the Arduino to prevent undesirable relays switching as well as DCDC_Ctrl toggles
Author: Valentin Py
Date: 20.05.2020
This applies to the Arduino Micro

 Another option would be to change physically the pins used to avoid using RX/TX Led and normal green LED, but it's less interesting :)


# Flashing bootloader:

## Why:

Arduino micro uses a bootloader (small part of software) which runs during boot and handle flashing the board over USB.
This bootloader is annoying because it flashed some LEDS, which are connected to essential parts of the Switchboard, and we don't want that to happen.

## How to flash bootloader:
You are probably lucky, the new bootloaded is already modified, tested and compiled.

It based on Katiana, see here: https://github.com/aweatherguy/Katiana

Compiled bootloader is the file "Katiana.hex" and has to be flashed using a different procedure as flashing a normal
application on an arduino.
For more detailed explainations, see online documentation about using an Arduino Uno as an ISP to burn custom bootloader.

### Required Hardware:
- Arduino Uno (programmer)
- Arduino Micro (to be programmed)
- USB cable for the Arduino Uno
- Experimentation cables (6)

### Flashing Arduino Uno
- Using Arduino IDE, open example "ArduinoISP" (File=>Examples=>11.ArduinoISP)
-  make sure that the following defines are correct and not commented:
  - `#define RESET     10 // Use pin 10 to reset the target rather than SS`
  - `#ifdef USE_OLD_STYLE_WIRING`
  - `#define PIN_MOSI	11`
  - `#define PIN_MISO	12`
  - `#define PIN_SCK		13`
- Select board (Arduino Uno! We are not using the Arduino micro yet!!)
- Select com Port
- Compile and Launch programs
- Make sure it worked and close Arduino IDE


### Physical connection
Connect Arduino Uno to Arduino Micro as following (6 wires required)

|Arduino uno | Arduino micro| (recommended color) |
|------------|--------------|---------------------|
5V          | 5V    | RED
GND         | GND   | BLACK
Pin 10      | RST   | YELLOW
Pin 11      | MOSI  | BLUE
Pin 12      | MISO  | GRAY
Pin 13      | SCK   | GREEN

### Compilation of bootloaders
This step is only required is you don't find the file `Katiana.hex`in the folder: `Switchboard/04_software/Katiana-bootloader/lufa-LUFA-170418/lufa-LUFA-170418/Projects/Katiana` or if you want to do some modifications

- (Use a linux PC or find a solution on Windows)
- Install required programs:
  - `sudo apt update`
  - `sudo apt install arduino arduino-mk`
- Using a terminal, go to folder `Switchboard/04_software/Katiana-bootloader/lufa-LUFA-170418/lufa-LUFA-170418/Projects/Katiana`
- launch following commands:
  -`make clean`
  -`make`
- You should now have a file named `Katiana.hex` in the current folder, copy it to you PC

### Burning bootloader:
- Connect the Arduino Uno using USB (arduino micro is not usb-connected, it will be powered by Arduino Uno)
- Open a Windows Terminal (CMD or Git bash)
- Launch the 2 following commands:
  - You'll need to replace a few paths to adapt to your configuration (avrdude, avrdude.cong, Katiana.hex)
  - `"C:\Users\Valentin Py\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/bin/avrdude" -C"C:\Users\Valentin Py\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/etc/avrdude.conf" -v -patmega32u4 -cstk500v1 -PCOM8 -b19200 -e -Ulock:w:0x3F:m -Uefuse:w:0xcb:m -Uhfuse:w:0xd8:m -Ulfuse:w:0xff:m`
  -`"C:\Users\Valentin Py\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/bin/avrdude" -C"C:\Users\Valentin Py\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/etc/avrdude.conf" -v -patmega32u4 -cstk500v1 -PCOM8 -b19200 -Uflash:w:Katiana.hex:i -Ulock:w:0x2F:m`

### Testing
- Unplug the boards
- Connect Arduino micro using USB
- Make sure it's detected by Windows and that you can flash it

# Updating pins_arduino.h
While an Arduino program is running, there are a few hidden things that happens (handling CPU configuration, USB support,...)

The problem we have, is that when a communication happens on USB, the RX_LED and/or TX_LED flash, as well as during boot.

As something can be connected to the pins corresponding to those LEDS, we might want to disable that.

The simple way is to modify the file "pins_arduino.h" corresponding to the board used (be careful: a different file exists for each board).

What has to bbe done is simply editing that file and change the macros for RXLED* and RXLED* to "void(0)"

In my case, the file is located here:
`C:/Users/Valentin Py/AppData/Local/Arduino15/packages/arduino/hardware/avr/1.8.2/variants/micro/pins_arduino.h`

And the content of the new file is:
~~~~
/*
  pins_arduino.h - Pin definition functions for Arduino
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2007 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA
*/

#include "../leonardo/pins_arduino.h"

#undef TXLED0
#undef TXLED1
#undef RXLED0
#undef RXLED1
#undef TX_RX_LED_INIT

#define TXLED0			void(0)
#define TXLED1			void(0)
#define RXLED0			void(0)
#define RXLED1			void(0)
#define TX_RX_LED_INIT	void(0)

~~~~

This modifications disable the macros used by the USB core (and others). It could also be possible to change how `USBCore.cpp` works

## Be careful: this will affect all micro boards flashed with that modification and might break some programs using that
