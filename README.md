# EngineGatewayN2K
N2K EngineGateway for a Volvo D1-30 A

This is a PlatformIO project for an Arduino Due as a node on a NMEA2000 network to inject engine data.  The Volvo D1-30 A has no electronics.  Everything is analog wired to a control panel. This project is designed to be Tee’ed into the wiring harness to detect the following states:

- Engine RPM
- Oil Pressure Switch
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

**Arduino Due Used Pins:** (Note There are electrical components required to bring every thing down to 3.3V max)
```
A0 - Analog 0 - Oil Presure Switch Input
A1 - Analog 1 - Coolant OverTemp Switch Input
A2 - Analog 2 - Altenator Voltage
A3 - Analog 3 - System Voltage
A4 - Analog 4 - Preheat Indicator
A5 - Analog 5 - Coolant Temp Input
A6 - Analog 6 - NC
A7 - Analog 7 - Tank Level Sensor.

D2 - Digital 0 - Tack Input
D3 - Neutral Switch
D4 - Forward Switch
D5 - Reverse Switch
```

**TODO:**
- Add a schematic
- Add code to track Engine hours
- Update Max and Min of fuel tack readings.
