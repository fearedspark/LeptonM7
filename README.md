# LeptonM7 Project

Lepton interface for the STM32F746G-DISCO board

# Description

This project uses the STM32F746G-DISCO to acquire the thermal images from a FLIR Lepton module and displays it on the screen.
It uses the Lepton Thermal Camera Breakout v1.3 from PURE Engineering (http://www.pureengineering.com/projects/lepton)

A prototype enclosure is also provided, in STL format, for 3D printing.

# Pin mapping:

- CS:	"D10" - PA8
- MOSI:	"D11" - PB15 (Not required)
- MISO:	"D12" - PB14
- SCLK:	"D13" - PI1
- SDA:	"D14" - PB9
- SCL:	"D15" - PB8

# Versions

- v1.0: First working version, displays the image on the screen
- v1.1:
	* Improved color representation using 6 colors (Black-Blue-Green-Yellow-Red-White) instead of 3 (Blue-Yellow-Red)
	* Added layout for buttons
	* Added level indicator on the left of the image
- v1.2:
	* Changed the buffering principle to reduce SDRAM access
	* Fixed the gradient function not working after last compiler update
	   
# Known issues

- Sync with the camera often not happening correclty at startup

# Future work

- Implement touch screen to select measurement points
- Implement SD card and file system to save images