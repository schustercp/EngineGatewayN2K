#include <Arduino.h>
#include "NMEA2000_CAN.h" // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"

// List here messages your device will transmit.
const unsigned long EngineGatewayTransmitMessages[] PROGMEM = {127488L, 127489L, 127493L, 127505L, 0};

#define ENGINE_INSTANCE (0)

volatile double EngineSpeed = 0.0;         //  - EngineSpeed           RPM (Revolutions Per Minute)
volatile double TackDuty = 0;
double EngineCoolantTemp = 200.1; //  - EngineCoolantTemp     in Kelvin
double AltenatorVoltage = 14.3;   //  - AltenatorVoltage      in Voltage
double EngineHours = 0.0;         //  - EngineHours           in seconds
bool flagCheckEngine = false;
bool flagOverTemp = false;
bool flagLowOilPress = false;
bool flagLowSystemVoltage = false;
bool flagChargeIndicator = false;
bool flagPreheatIndicator = false;
tN2kTransmissionGear TransmissionGear = N2kTG_Neutral;
const tN2kFluidType FluidType = N2kft_Fuel; //N2kft_Water
double Level = 0.0;                         //  - Level                 Tank level in % of full tank.
double Capacity = 378.0;                    //  - Capacity              Tank Capacity in litres
const double MaxLevelValue = 1023.0;
const double MinLevelValue = 100.0;
//int ledState3 = LOW;
//int ledState4 = LOW;
//int ledState5 = LOW;
//int ledState6 = LOW;
//int ledState7 = LOW;

void TC0_Handler()
{
  static uint32_t ra = 0;
  //static uint32_t rb = 0;
  uint32_t status = TC_GetStatus(TC0, 0);

  //digitalWrite(3, ledState3);
  //ledState3 = ledState3 ? LOW : HIGH;

  // load overrun on RA or RB
  if (status & TC_SR_LOVRS)
  {
    EngineSpeed = 0.0;
    //digitalWrite(4, ledState4);
    //ledState4 = ledState4 ? LOW : HIGH;
  }

  // RA loaded?
  if (status & TC_SR_LDRAS)
  {
    ra = TC0->TC_CHANNEL[0].TC_RA;
  }

  // RB loaded?
  if (status & TC_SR_LDRBS)
  {
    uint32_t rb = TC0->TC_CHANNEL[0].TC_RB;
    double localEngineSpeed = (42000000.0 * 60.0) / (double)rb;
    double localTackDuty = (double)ra / (double)rb * 100.0;

    if(localEngineSpeed < 4000.0 && localTackDuty > 15.0 && localTackDuty < 85.0)
    {
      EngineSpeed = localEngineSpeed;
      TackDuty = localTackDuty;
    }

    //digitalWrite(7, ledState7);
    //ledState7 = ledState7 ? LOW : HIGH;
  }

  // RC compare interrupt?
  if (status & TC_SR_CPCS)
  {
    EngineSpeed = 0.0;
    //digitalWrite(5, ledState5);
    //ledState5 = ledState5 ? LOW : HIGH;
  }
}

void setup()
{
  pinMode(2, INPUT);
  //pinMode(3, OUTPUT);
  //pinMode(4, OUTPUT);
  //pinMode(5, OUTPUT);
  //pinMode(6, OUTPUT);
  //pinMode(7, OUTPUT);

  //Serial.begin(115200);

  NMEA2000.SetDeviceCount(1); // Enable multi device support for devices
  // Set Product information for temperature monitor
  NMEA2000.SetProductInformation("D21X00001",                   // Manufacturer's Model serial code.
                                 100,                           // Manufacturer's product code
                                 "Volvo D1-30A Engine Gateway", // Manufacturer's Model ID
                                 "1.0.0.0 (2021-03-19)",        // Manufacturer's Software version code
                                 "1.0.0.0 (2021-03-19)",        // Manufacturer's Model version
                                 0xff,                          // load equivalency - use default
                                 0xffff,                        // NMEA 2000 version - use default
                                 0xff,                          // Sertification level - use default
                                 0);

  // Set device information for temperature monitor
  NMEA2000.SetDeviceInformation(10000001, // Unique number. Use e.g. Serial number.
                                160,      // Device function=Engine Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                50,       // Device class=Sensor Communication Interface. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2060,     // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
                                4,        // Marine
                                0);

  // Uncomment 2 rows below to see, what device will send to bus. Use e.g. OpenSkipper or Actisense NMEA Reader
  //NMEA2000.SetForwardStream(&Serial);
  // If you want to use simple ascii monitor like Arduino Serial Monitor, uncomment next line
  //NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  //NMEA2000.SetForwardOwnMessages(false);

  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, 22);
  //NMEA2000.SetDebugMode(tNMEA2000::dm_Actisense); // Uncomment this, so you can test code without CAN bus chips on Arduino Mega
  NMEA2000.EnableForward(true); // Disable all msg forwarding to USB (=Serial)

  // Here we tell, which PGNs we transmit from Engine Gateway
  NMEA2000.ExtendTransmitMessages(EngineGatewayTransmitMessages, 0);

  NMEA2000.Open();

  // PMC settings
  pmc_set_writeprotect(0);
  pmc_enable_periph_clk(uint32_t(TC0_IRQn));

  // timing setings in capture mode
  TC_Configure(TC0, 0,
               TC_CMR_TCCLKS_TIMER_CLOCK1 | // clock prescaler set to /2
                   TC_CMR_CPCTRG |          // timer reset on RC match
                   TC_CMR_LDRA_RISING |     // capture to RA on rising edge
                   TC_CMR_LDRB_FALLING |    // capture to RB on falling edge
                   TC_CMR_ETRGEDG_FALLING | // external trigger on falling edge
                   TC_CMR_ABETRG            // external trigger on TIOA
  );

  // seting RC to the capture window
  TC_SetRC(TC0, 0, 0x03C1FFFF);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_LOVRS | TC_IER_LDRAS | TC_IER_LDRBS | TC_IER_CPCS;
  NVIC_SetPriority(TC0_IRQn, 0);
  NVIC_EnableIRQ(TC0_IRQn);
  NVIC_ClearPendingIRQ(TC0_IRQn);
  TC_Start(TC0, 0);
}

#define EngineRapidUpdatePeriod 100
#define EngineUpdatePeriod 1000

// A0 - Analog 0 - Oil Presure Switch Input
// A1 - Analog 1 - Coolant OverTemp Switch Input
// A2 - Analog 2 - Altenator Voltage
// A3 - Analog 3 - System Voltage
// A4 - Analog 4 - Preheat Indicator
// A5 - Analog 5 - Coolant Temp Input
// A6 - Analog 6
// A7 - Analog 7 - Tank Level Sensor.

// D0 - Digital 0 - Tack Input
// D1 - Neutral Switch
// D2 - Forward Switch
// D3 - Reverse Switch

void SendN2kEngineInfo()
{
  static unsigned long Updated = millis();
  static unsigned long RapidUpdated = millis();
  tN2kMsg N2kMsg;

  if (RapidUpdated + EngineRapidUpdatePeriod < millis())
  {
    RapidUpdated = millis();
    SetN2kEngineParamRapid(N2kMsg, ENGINE_INSTANCE, EngineSpeed); //,820000,48);
    NMEA2000.SendMsg(N2kMsg, 0);

    //Serial.print(EngineSpeed);
    //Serial.print(" :: ");
    //Serial.println(TackDuty);
  }

  if (Updated + EngineUpdatePeriod < millis())
  {
    Updated = millis();
    SetN2kEngineDynamicParam(N2kMsg,
                             ENGINE_INSTANCE,
                             0.0,
                             0.0,
                             EngineCoolantTemp,
                             AltenatorVoltage,
                             0.0,
                             EngineHours,
                             -1000000000.0,
                             -1000000000.0,
                             (int8_t)'\177',
                             (int8_t)'\177',
                             flagCheckEngine,
                             flagOverTemp,
                             flagLowOilPress,
                             false,
                             false,
                             flagLowSystemVoltage,
                             false,
                             false,
                             false,
                             flagChargeIndicator,
                             flagPreheatIndicator);
    NMEA2000.SendMsg(N2kMsg, 0);

    SetN2kTransmissionParameters(N2kMsg, ENGINE_INSTANCE, TransmissionGear, 0.0, 0.0);
    NMEA2000.SendMsg(N2kMsg, 0);

    SetN2kFluidLevel(N2kMsg, ENGINE_INSTANCE, FluidType, Level, Capacity);
    NMEA2000.SendMsg(N2kMsg, 0);
  }
}

void loop()
{
  static unsigned int sample_state = 0;

  //digitalWrite(6, ledState6);
  //ledState6 = ledState6 ? LOW : HIGH;

  SendN2kEngineInfo();
  NMEA2000.ParseMessages();

  switch (sample_state)
  {
  case 0:
  {
    // A0 - Analog 0 - Oil Presure Switch Input
    analogReadResolution(10);
    uint32_t sensorValue = analogRead(A0);
    flagLowOilPress = (sensorValue <= 310);
    sample_state++;
  }
  break;

  case 1:
  {
    // A1 - Analog 1 - Coolant OverTemp Switch Input
    analogReadResolution(10);
    uint32_t sensorValue = analogRead(A1);
    flagOverTemp = (sensorValue > 310);
    sample_state++;
  }
  break;

  case 2:
  {
    // A2 - Analog 2 - Altenator Voltage
    analogReadResolution(10);
    uint32_t sensorValue = analogRead(A2);
    AltenatorVoltage = (3.3 / 1024.0) * (double)sensorValue * 6.0; /*This nees to be adjusted for resitor values*/
    flagChargeIndicator = (AltenatorVoltage < 12.0);
    sample_state++;
  }
  break;

  case 3:
  {
    // A3 - Analog 3 - System Voltage
    analogReadResolution(10);
    uint32_t sensorValue = analogRead(A3);
    double SystemVoltage = (3.3 / 1024.0) * (double)sensorValue * 6.0; /*This nees to be adjusted for resitor values*/
    flagLowSystemVoltage = (SystemVoltage < 11.0);
    sample_state++;
  }
  break;

  case 4:
  {
    // A4 - Analog 4 - Preheat Indicator
    uint32_t sensorValue = analogRead(A4);
    flagPreheatIndicator = (sensorValue > 310);
    sample_state++;
  }
  break;

  case 5:
  {
    // A7 - Analog 7 - Tank Level Sensor.
    uint32_t sensorValue = analogRead(A7);
    Level = (sensorValue - MinLevelValue) / (MaxLevelValue - MinLevelValue);
    sample_state = 0;
  }
  break;

  default:
    sample_state = 0;
    break;
  }

  flagCheckEngine = flagLowOilPress | flagOverTemp | flagChargeIndicator | flagLowSystemVoltage;
}