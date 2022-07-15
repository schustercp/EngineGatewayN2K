# EngineGatewayN2K
N2K EngineGateway for a Volvo D1-30 A

This is a PlatformIO project for an Arduino Due as a node on a NMEA2000 network to inject engine data.  The Volvo D1-30 A has no electronics.  Everything is analog wired to a control panel. This project is designed to be Tee’ed into the wiring harness to detect the following states:

- Engine RPM
- Oil Pressure Switch
- Oil Presure
- Coolant Temp Switch 
- Coolant Temp
- Voltage from Alternator
- Preheat Activation

Micro switched will be added to the engine to allow detection of Neutral, Drive, and Reverse.

The engine hours will also be tracked by this processor.

Due to the easy of adding a a tie into the fuel float, The fuel level is also reported.

**Credits:**
This project uses the N2K libraries by Timo Lappalainen (ttlappalainen).  Thanks Timo for your hard work.
[https://github.com/ttlappalainen]

# Oil Presure Sensor
- sensor is from Amazon
- AUTEX Pressure Transducer Sensor 100 Psi Pressure Sender 316 Stainless Steel Oil Pressure Transmitter 1/8" -27 NPT For Oil Fuel Air Water Pressure
- Output: 0.5V – 4.5V linear voltage output. 0 psi outputs 0.5V, 50 psi outputs 2.5V, 100 psi outputs 4.5V

# Water Temp Sensor
- Sensor was supplied with the Engine, Was connected to harness, but was not connected and control panel
- Temp. 60oC: 134.0 ±13.5 Ω (±4oC)
- Temp. 90oC: 51.2 ±4.3 Ω (±4oC)
- Temp. 100oC: 38.5 ±3.0 Ω (±4oC)
- Overheat => 110oC ± 2oC

# Desired Inputs
```
1 - Oil Presure Sensor
2 - Oil Presure Switch Input
3 - Coolant Temp Sensor
4 - Coolant OverTemp Switch Input
5 - System Voltage
6 - Preheat Indicator
```

# Conector Pinout
```
1 - Purple       - RPM Sensor +
2 - Grey         - Coolant Temp Sensor
3 - Green        - System Voltage
4 - White        - Preheat Indicator
5 - White/Black  - Coolant OverTemp Switch Input
6 - Green/Black  - Oil Presure Switch Input
7 - Grey/Black   - Ground
8 - Purple/Black - RPM Sensor -
```

**Arduino Due Used Pins:** (Note There are electrical components required to bring every thing down to 3.3V max)
```
A0 - Analog 0 - Green - R1/R2 - 67.0K / 11.62K
A1 - Analog 1 - Green/Black - R1/R2 - 67.2K / 11.67K
A2 - Analog 2 - White - R1/R2 - 67.1K / 12.04K
A3 - Analog 3 - White/Black - R1/R2 - 67.1K / 11.86K
A4 - Analog 4 - Oil Presure Sensor - R1/R2 - 1486 / 3227
A5 - Analog 5 - Gray - R1/R2 - 67.1K / 11.94K
A6 - Analog 6 - 
A7 - Analog 7 - 
A8 - Analog 8 - Purge Input - GND
GND - Grey/Black


D2 - Digital 0 - Tack Input
D3 - Neutral Switch
D4 - Forward Switch
D5 - Reverse Switch
```

**TODO:**
- Add a schematic
- Add code to track Engine hours
- Update Max and Min of fuel tack readings.
