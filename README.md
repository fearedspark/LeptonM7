# LeptonM7 Project

Lepton interface for the STM32F746G-DISCO board

# Description

This project uses the STM32F746G-DISCO to acquire the thermal images from a FLIR Lepton module and displays it on the screen.
It uses the Lepton Thermal Camera Breakout v1.3 from PURE Engineering (http://www.pureengineering.com/projects/lepton)

# Pin mapping:

-CS:	"D10" - PA8
-MOSI:	"D11" - PB15 (Not required)
-MISO:	"D12" - PB14
-SCLK:	"D13" - PI1
-SDA:	"D14" - PB9
-SCL:	"D15" - PB8

# Versions

-v1.0: First working version, displays the image on the screen

# Future work

-Implement touch screen to select measurement points
-Improve the color representation
-Implement SD card and file system to save images