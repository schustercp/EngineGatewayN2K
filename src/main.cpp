#include <Arduino.h>
#include "NMEA2000_CAN.h" // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"
#include <N2kMessagesEnumToStr.h>
#include <rtc_clock.h>
#include <DueFlashStorage.h>

RTC_clock rtc_clock(XTAL);
DueFlashStorage dueFlashStorage;

// List here messages your device will transmit.
const unsigned long EngineGatewayTransmitMessages[] PROGMEM = {127488L, 127489L, 127493L, 127505L, 0};

#define ENGINE_INSTANCE (0)
#define EngineRapidUpdatePeriod 100
#define EngineUpdatePeriod 1000

// A0 - Analog 0 - Pin 3 - Green - System Voltage - R1/R2 - 67.0K / 11.62K
// A1 - Analog 1 - Pin 6 - Green/Black - Oil Presure Switch Input - R1/R2 - 67.2K / 11.67K
// A2 - Analog 2 - Pin 4 - White - Preheat Indicator - R1/R2 - 67.1K / 12.04K
// A3 - Analog 3 - Pin 5 - White/Black - Coolant OverTemp Switch Input - R1/R2 - 67.1K / 11.86K
// A4 - Analog 4 - Oil Presure Sensor - R1/R2 - 1486 / 3227
// A5 - Analog 5 - pin 2 - Grey - Coolant Temp Sensor - R1/R2 - 992 Ohms
// A6 - Analog 6 - NC
// A7 - Analog 7 - NC
// A8 - Analog 8 - Purge Input - GND

// D0 - Digital 0 - Tack Input
// D1 - Neutral Switch
// D2 - Forward Switch
// D3 - Reverse Switch

#define ADC_OILPRESSURESENSOR A4
#define ADC_OILPRESSURESWITCH A1
#define ADC_COOLANTTEMPSENSOR A5
#define ADC_COOLANTTEMPSWITCH A3
#define ADC_PREHEAT A2
#define ADC_SYSTEMVOLTAGE A0
#define ADC_PURGE A8

uint32_t EngineSeconds = 7016400;

volatile uint32_t MainLoopCount = 0;
volatile double EngineSpeed = 0.0;         //  - EngineSpeed           RPM (Revolutions Per Minute)
volatile double TackDuty = 0.0;
volatile double EngineCoolantTemp = 0.0; //  - EngineCoolantTemp     in Kelvin
volatile double AltenatorVoltage = 0.0;   //  - AltenatorVoltage      in Voltage
volatile double EngineHours = 0.0;         //  - EngineHours           in seconds
volatile double EngineOilPress = 0.0;
volatile bool flagCheckEngine = false;
volatile bool flagOverTemp = false;
volatile bool flagLowOilPress = true;
volatile bool flagLowSystemVoltage = true;
volatile bool flagPreheatIndicator = false;
volatile bool EngineRunningFlag = false;
volatile uint32_t T0Count = 0;
tN2kTransmissionGear TransmissionGear = N2kTG_Neutral;

const uint32_t sizeOfEngineSpeedArray = 64;
volatile static double EngineSpeedArray[sizeOfEngineSpeedArray];
volatile static uint32_t EngineSpeedArrayIndex = 0;

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void writeEngineTime()
{
  // then write it to flash like this:
  byte b2[sizeof(EngineSeconds)]; // create byte array to store the struct
  memcpy(b2, &EngineSeconds, sizeof(EngineSeconds)); // copy the struct to the byte array
  dueFlashStorage.write(4, b2, sizeof(EngineSeconds)); // write byte array to flash at address 4
}

void readEngineTime()
{
  byte* b = dueFlashStorage.readAddress(4); // byte array which is read from flash at adress 4
  memcpy(&EngineSeconds, b, sizeof(EngineSeconds)); // copy byte array to temporary struct
  EngineHours = (double)EngineSeconds;
}

void TC0_Handler()
{
  static uint32_t ra = 0;
  static uint32_t RCCompareCount = 0;
  uint32_t status = TC_GetStatus(TC0, 0);

  // load overrun on RA or RB
  if (status & TC_SR_LOVRS)
  {
    ra = TC0->TC_CHANNEL[0].TC_RA;
    uint32_t rb = TC0->TC_CHANNEL[0].TC_RB;
    TackDuty = ra / rb;
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

    //The DUE is running at 84MHz.  
    // clock prescaler set to /2
    // T0 is running at 42MHz
    // There are 28 pulses per rotattion.
    // 42MHz / RB == Pulses / Second * 60 Seconds / Min * 1/28 Rotation / Pulse
    double RPM = (42000000.0 * 60.0) / ((double)rb * 28.0);

    if(RPM > 0.0 && RPM < 4000.0)
    {
      EngineSpeedArray[EngineSpeedArrayIndex++] = RPM;
      if(EngineSpeedArrayIndex >= sizeOfEngineSpeedArray) EngineSpeedArrayIndex = 0;
    }
    
    RCCompareCount = 0;
    T0Count++;
  }

  // RC compare interrupt?
  if (status & TC_SR_CPCS)
  {
    RCCompareCount++;

    if(RCCompareCount > 2 && EngineSpeed > 0.0)
    {
      for( uint32_t i = 0; i < sizeOfEngineSpeedArray; i++)
      {
        EngineSpeedArray[i] = 0.0;
      }
      
      if (Serial)
      {
        Serial.println("*");
      }
    }
  }
}

void setup()
{
  for( uint32_t i = 0; i < sizeOfEngineSpeedArray; i++)
  {
    EngineSpeedArray[i] = 0.0;
  }

  pinMode(2, INPUT);

  Serial.begin(115200);

  NMEA2000.SetDeviceCount(1); // Enable multi device support for devices
  // Set Product information for temperature monitor
  NMEA2000.SetProductInformation("D21X00001",                   // Manufacturer's Model serial code.
                                 100,                           // Manufacturer's product code
                                 "Volvo D1-30A Engine Gateway", // Manufacturer's Model ID
                                 "1.0.0.0 (2022-07-15)",        // Manufacturer's Software version code
                                 "1.0.0.0 (2022-07-15)",        // Manufacturer's Model version
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

  // Seting RC to the capture window
  TC_SetRC(TC0, 0, 0x03C1FFFF);

  TC0->TC_CHANNEL[0].TC_IER = TC_IER_LOVRS | TC_IER_LDRAS | TC_IER_LDRBS | TC_IER_CPCS;
  NVIC_SetPriority(TC0_IRQn, 0);
  NVIC_EnableIRQ(TC0_IRQn);
  NVIC_ClearPendingIRQ(TC0_IRQn);
  TC_Start(TC0, 0);

  readEngineTime();
  if(EngineSeconds < 7016400 || EngineSeconds == 0xFFFFFFFF )
  {
    EngineSeconds = 7016400;
    writeEngineTime();
    if (Serial)
    {
      Serial.println("Engine Time Initalized.");
    }
    readEngineTime();
  }

  rtc_clock.init();

  const char *time = __TIME__;
  const char *date = __DATE__;

  rtc_clock.set_time(time);
  rtc_clock.set_date(date);

  Serial.print("Unixtime: ");
  Serial.println(rtc_clock.unixtime());
}

void PrintInfo()
{
  if (Serial)
  {
    Serial.print(flagCheckEngine);
    Serial.print(",");
    Serial.print(flagOverTemp);
    Serial.print(",");
    Serial.print(flagLowOilPress);
    Serial.print(",");
    Serial.print(flagLowSystemVoltage);
    Serial.print(",");
    Serial.print(flagPreheatIndicator);
    Serial.print(",");
    Serial.print(EngineHours / 3600.0);
    Serial.print(",");
    Serial.print(EngineOilPress);
    Serial.print(",");
    Serial.print(AltenatorVoltage);
    Serial.print(",");
    Serial.print(EngineCoolantTemp);
    Serial.print(",");
    Serial.print(TackDuty);
    Serial.print(",");
    Serial.print(EngineSpeed);
    Serial.print(",");
    Serial.print(MainLoopCount);
    Serial.print(",");
    Serial.println(T0Count);
    T0Count = 0;
  }

  MainLoopCount = 0;
}

bool EngineRunning()
{
  static uint32_t startTime = 0;

  double localEngineSpeed = 0.0;
  double localEngineSpeedMin = 4000.0;
  double localEngineSpeedMax = 0.0;
  for( uint32_t i = 0; i < sizeOfEngineSpeedArray; i++)
  {
    localEngineSpeed += EngineSpeedArray[i];

    if(EngineSpeedArray[i] > localEngineSpeedMax)
    {
      localEngineSpeedMax = EngineSpeedArray[i];
    }

    if(EngineSpeedArray[i] < localEngineSpeedMin)
    {
      localEngineSpeedMin = EngineSpeedArray[i];
    }
  }
  localEngineSpeed /= sizeOfEngineSpeedArray;
  if(localEngineSpeed >= 0.0 && localEngineSpeed < 4000.0)
  {
    EngineSpeed = localEngineSpeed;
  }

  if(!EngineRunningFlag && EngineSpeed > 275.0)
  { //Engine Started
    //SetRTC
    readEngineTime();
    startTime = rtc_clock.unixtime();
    EngineRunningFlag = true;
    if (Serial)
    {
      Serial.print("Engine Started at: ");
      Serial.println(startTime);
    }
  }
  else if(EngineRunningFlag && EngineSpeed < 225.0)
  {
    //Engine Stopped
    uint32_t stopTime = rtc_clock.unixtime();
    uint32_t engineRunTime = stopTime - startTime;
    readEngineTime();
    EngineSeconds += engineRunTime;
    EngineHours = (double)EngineSeconds;
    writeEngineTime();
    EngineRunningFlag = false;
    startTime = 0;
    if (Serial)
    {
      Serial.print("Engine ran for: ");
      Serial.println(engineRunTime);
    }
  }

  if(EngineRunningFlag)
  {
    uint32_t nowTime = rtc_clock.unixtime();
    uint32_t engineRunTime = nowTime - startTime;
    EngineHours = (double)(EngineSeconds + engineRunTime);
  }

  return EngineRunningFlag;
}

void SendN2kEngineInfo()
{
  static unsigned long Updated = 0;
  static unsigned long RapidUpdated = 0;
  tN2kMsg N2kMsg;

  if (EngineRunningFlag && (RapidUpdated + EngineRapidUpdatePeriod <= millis()))
  {
    RapidUpdated = millis();
    SetN2kEngineParamRapid(N2kMsg, ENGINE_INSTANCE, EngineSpeed); //,820000,48);
    NMEA2000.SendMsg(N2kMsg, 0);
  }

  if (Updated + EngineUpdatePeriod <= millis())
  {
    //Field 1: Engine Instance (8-bit unsigned integer) This field indicates the particular engine for which this data applies. A single engine will have an instance of 0. Engines in multi-engine boats will be numbered starting at 0 at the bow of the boat incrementing to n going towards the stern of the boat. For engines at the same distance from the bow and stern, the engines are numbered starting from the port side and proceeding towards the starboard side.
    //Field 2: Engine Oil Pressure (16-bit unsigned integer) This field indicates the oil pressure of the engine in units of 100 Pa.
    //Field 3: Engine Oil Temperature (16-bit unsigned integer) This field indicates the oil temperature of the engine in units of 0.1°K.
    //Field 4: Engine Temperature (16-bit unsigned integer) This field indicates the temperature of the engine coolant in units of 0.1°K.
    //Field 5: Alternator Potential (16-bit signed integer) This field indicates the alternator voltage in units of 0.01V.
    //Field 6: Fuel Rate (16-bit signed integer) This field indicates the fuel consumption rate of the engine in units of 0.0001 cubic-meters/hour.
    //Field 7: Total Engine Hours (32-bit unsigned integer) This field indicates the cumulative runtime of the engine in units of 1 second.
    //Field 8: Engine Coolant Pressure (16-bit unsigned integer) This field indicates the pressure of the engine coolant in units of 100 Pa.
    //Field 9: Fuel Pressure (16-bit unsigned integer) This field indicates the pressure of the engine fuel in units of 1000 Pa.
    
    Updated = millis();
    SetN2kEngineDynamicParam(N2kMsg,
                             ENGINE_INSTANCE,
                             EngineOilPress,
                             0.0, //Engine Oil Temperature
                             EngineCoolantTemp,
                             AltenatorVoltage,
                             0.0, //Fuel Rate
                             EngineHours,
                             -1000000000.0, //Engine Coolant Pressure
                             -1000000000.0, //Fuel Pressure
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
                             flagLowSystemVoltage,
                             flagPreheatIndicator);
    NMEA2000.SendMsg(N2kMsg, 0);

    // SetN2kTransmissionParameters(N2kMsg, ENGINE_INSTANCE, TransmissionGear, 0.0, 0.0);
    // NMEA2000.SendMsg(N2kMsg, 0);

    // SetN2kFluidLevel(N2kMsg, ENGINE_INSTANCE, FluidType, Level, Capacity);
    // NMEA2000.SendMsg(N2kMsg, 0);

    PrintInfo();
  }
}

void loop()
{
  static unsigned int sample_state = 0;

  EngineRunning();
  SendN2kEngineInfo();
  NMEA2000.ParseMessages();

  if ((sample_state % 2) == 0)
  {
    switch (sample_state)
    {
    case 0:
    {
      MainLoopCount++;
      const uint32_t sizeOfArray = 64;
      static uint32_t oilPressureArray[sizeOfArray];
      static uint32_t oilPressureIndex = 0;

      // Oil Presure Sensor Input
      // A4 - Analog 4 - Oil Presure Sensor - R1/R2 - 1486 / 3227
      analogReadResolution(10);
      const double ADC_Ref_Voltage = 3.3;
      const double maxADCCount = 1024.0;
      const double R1 = 1486.0;
      const double R2 = 3227.0;
      oilPressureArray[oilPressureIndex++] = analogRead(ADC_OILPRESSURESENSOR);
      if (oilPressureIndex >= sizeOfArray ) oilPressureIndex = 0;

      uint32_t sensorValue = 0;
      for(uint32_t i = 0; i < sizeOfArray; i++)
      {
        sensorValue += oilPressureArray[i];
      }
      sensorValue /= sizeOfArray;

      double V1 = ADC_Ref_Voltage / maxADCCount * sensorValue;
      double V2 = V1*(R1+R2)/R2;
      EngineOilPress = 25.0 * V2 - 12.5;
      //flagLowOilPress = EngineOilPress < 10.0;
      if(EngineOilPress < 1.0)
      {
        EngineOilPress = 0.0;
      }

      //Convert PSI to 100 Pa
      EngineOilPress *= 6894.76;
      sample_state++;

      // if (Serial)
      // {
      //   Serial.println("****");
      //   Serial.print("ADC Oil Press: ");
      //   Serial.print(EngineOilPress);
      //   Serial.print(" : ");
      //   Serial.print(sensorValue);
      //   Serial.print(" : ");
      //   Serial.print(V1);
      //   Serial.print(" : ");
      //   Serial.println(V2);
      // }
    }
    break;

    case 2:
    {
      //Coolant OverTemp Switch Input
      // A3 - Analog 3 - Pin 5 - White/Black - Coolant OverTemp Switch Input - R1/R2 - 67.1K / 11.86K
      analogReadResolution(10);
      const double ADC_Ref_Voltage = 3.3;
      const double maxADCCount = 1024.0;
      const double R1 = 67100.0;
      const double R2 = 11860.0;
      uint32_t sensorValue = analogRead(ADC_COOLANTTEMPSWITCH);
      //Switch closes if temp of engine is to hight.
      double V1 = ADC_Ref_Voltage / maxADCCount * sensorValue;
      double V2 = V1*(R1+R2)/R2;

      if(flagOverTemp)
      {
        flagOverTemp = (V2 < 1.0);
      }
      else
      {
        flagOverTemp = (V2 < 1.5);
      }
      sample_state++; 

      // if (Serial)
      // {
      //   Serial.print("Coolant Switch: ");
      //   Serial.print(V2);
      //   Serial.print(" : ");
      //   Serial.println(sensorValue);
      // }
    }
    break;

    case 4:
    {
      // Engine Coolant Temp
      // A5 - Analog 5 - pin 2 - Grey - Coolant Temp Sensor - R1/R2 - 992 Ohms

      const uint32_t sizeOfArray = 64;
      static uint32_t collantTempArray[sizeOfArray];
      static uint32_t arrayIndex = 0;

      analogReadResolution(10);
      collantTempArray[arrayIndex++] = analogRead(ADC_COOLANTTEMPSENSOR);
      if (arrayIndex >= sizeOfArray ) arrayIndex = 0;

      uint32_t sensorValue = 0;
      for(uint32_t i = 0; i < sizeOfArray; i++)
      {
        sensorValue += collantTempArray[i];
      }
      sensorValue /= sizeOfArray;

      if(sensorValue >= 346)
      {
        sensorValue = 345;
      }
      //TODO Convert to Engine Temp
      const double ADC_Ref_Voltage = 3.3;
      const double maxADCCount = 1024.0;
      const double R1 = 992.0;
      double V1 = ADC_Ref_Voltage / maxADCCount * sensorValue;
      const double V2 = 3.3;
      double R2 = V1*R1/(V2-V1);
      double EngineCoolantTempDegC = 0.000605653*(R2-134)*(R2-484.6)-0.000008620208891455*(R2-134)*(R2-51.2)*(R2-484.6)-0.0998289*(R2-484.6)+25;

      //Convert Deg C to Deg K
      EngineCoolantTemp = EngineCoolantTempDegC + 273.15;

      if(!flagOverTemp)
      {
        flagOverTemp = EngineCoolantTempDegC > 109.0;
      }
      sample_state++;

      // if (Serial)
      // {
      //   Serial.print("Coolant Sensor: ");
      //   Serial.print(EngineCoolantTemp);
      //   Serial.print(" : ");
      //   Serial.print(sensorValue);
      //   Serial.print(" : ");
      //   Serial.print(V1);
      //   Serial.print(" : ");
      //   Serial.println(R2);
      // }
    }
    break;

    case 6:
    {
      // Altenator Voltage
      // A0 - Analog 0 - Pin 3 - Green - Altenator Voltage - R1/R2 - 67.0K / 11.62K

      const uint32_t sizeOfArray = 256;
      static uint32_t AltenatorVoltageArray[sizeOfArray];
      static uint32_t arrayIndex = 0;

      analogReadResolution(10);
      AltenatorVoltageArray[arrayIndex++] = analogRead(ADC_SYSTEMVOLTAGE);
      if (arrayIndex >= sizeOfArray ) arrayIndex = 0;

      uint32_t sensorValue = 0;
      for(uint32_t i = 0; i < sizeOfArray; i++)
      {
        sensorValue += AltenatorVoltageArray[i];
      }
      sensorValue /= sizeOfArray;

      const double ADC_Ref_Voltage = 3.3;
      const double maxADCCount = 1024.0;
      const double R1 = 67000.0;
      const double R2 = 11620.0;
      double V1 = ADC_Ref_Voltage / maxADCCount * sensorValue;
      AltenatorVoltage = V1*(R1+R2)/R2;

      if(flagLowSystemVoltage)
      {
        flagLowSystemVoltage = (AltenatorVoltage < 12.2);
      }
      else
      {
        flagLowSystemVoltage = (AltenatorVoltage < 11.5);
      }
      sample_state++;

      // if (Serial)
      // {
      //   Serial.print("System Voltage: ");
      //   Serial.println(AltenatorVoltage);
      // }
    }
    break;

    case 8:
    {
      // Preheat Indicator
      // A2 - Analog 2 - Pin 4 - White - Preheat Indicator - R1/R2 - 67.1K / 12.04K
      analogReadResolution(10);
      uint32_t sensorValue = analogRead(ADC_PREHEAT);
      const double ADC_Ref_Voltage = 3.3;
      const double maxADCCount = 1024.0;
      const double R1 = 67100.0;
      const double R2 = 12040.0;
      double V1 = ADC_Ref_Voltage / maxADCCount * sensorValue;
      double V2 = V1*(R1+R2)/R2;

      if(flagPreheatIndicator)
      {
        flagPreheatIndicator = (V2 > 5.5);
      }
      else
      {
        flagPreheatIndicator = (V2 > 5.0);
      }
      sample_state++;

      // if (Serial)
      // {
      //   Serial.print("Preheat: ");
      //   Serial.println(V2);
      // }
    }
    break;

    case 10:
    {
      // Oil Presure Switch Input
      // A1 - Analog 1 - Pin 6 - Green/Black - Oil Presure Switch Input - R1/R2 - 67.2K / 11.67K
      analogReadResolution(10);
      uint32_t sensorValue = analogRead(ADC_OILPRESSURESWITCH);
      const double ADC_Ref_Voltage = 3.3;
      const double maxADCCount = 1024.0;
      const double R1 = 67200.0;
      const double R2 = 11670.0;
      //Switch closes if oil presure of engine is to LOW.
      double V1 = ADC_Ref_Voltage / maxADCCount * sensorValue;
      double V2 = V1*(R1+R2)/R2;

      if(flagLowOilPress)
      {
        flagLowOilPress = (V2 < 1.0);
      }
      else
      {
        flagLowOilPress = (V2 < 1.5);
      }
      sample_state++;

      // if (Serial)
      // {
      //   Serial.print("Oil press Switch: ");
      //   Serial.println(V2);
      // }
    }
    break;

    default:
      sample_state = 0;
      break;
    }
  }
  else
  { //Samplestate is odd
    analogReadResolution(10);
    analogRead(ADC_PURGE);

    // if (Serial)
    // {
    //   Serial.print("ADC Purge Value: ");
    //   Serial.println(gndValue);
    // }

    sample_state++;
  }

  if(EngineRunningFlag)
  {
    flagCheckEngine = flagLowOilPress | flagOverTemp | flagLowSystemVoltage;
  }
  else
  {
    flagCheckEngine = false;
  }
}