#include <Arduino.h>
#include "NMEA2000_CAN.h" // This will automatically choose right CAN library and create suitable NMEA2000 object
#include "N2kMessages.h"
#include <N2kMessagesEnumToStr.h>
#include <NMEA2000_Teensyx.h>
#include <EEPROM.h>
#include "PinInfo.h"
#include <algorithm>
#include "clock_functions.h"
#include "Audio_pll.h"
#include "TeensyTimerTool.h"
#include <ADC_Module.h>
#include "ADC_Data.h"

using namespace TeensyTimerTool;

// List here messages your device will transmit.
const unsigned long EngineGatewayTransmitMessages[] PROGMEM = {127488L, 127489L, 127493L, 127505L, 0};

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

uint32_t EngineSeconds = InitalEngineSeconds;

volatile uint32_t MainLoopCount = 0;
volatile double EngineSpeed = 0.0;         //  - EngineSpeed           RPM (Revolutions Per Minute)
volatile double EngineHours = 0.0;         //  - EngineHours           in seconds
volatile bool EngineRunningFlag = false;
double adcOffset = 0.0;
ADC_Data dataBank[8];
tN2kTransmissionGear TransmissionGear = N2kTG_Neutral;

const uint32_t sizeOfEngineSpeedArray = 512;
volatile static double EngineSpeedArray[sizeOfEngineSpeedArray];
volatile static uint32_t EngineSpeedArrayIndex = 0;

const uint8_t channel2sc1aADC0[] = {
    // new version, gives directly the sc1a number. 0x1F=31 deactivates the ADC.
    7, 8, 12, 11, 6, 5, 15, 0, 13, 14, 1, 2, 31, 31, // 0-13, we treat them as A0-A13
    7, 8, 12, 11, 6, 5, 15, 0, 13, 14,               // 14-23 (A0-A9)
    1, 2, 31, 31                                     // A10, A11, A12, A13
};

ADC_Module* adc_obj;

typedef struct 
{
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

void writeEngineTime()
{ 
  EEPROM.put( 4, EngineSeconds);
}

void readEngineTime()
{
  EEPROM.get( 4, EngineSeconds );
  EngineHours = (double)EngineSeconds;
}

void printPins(PinInfo* pins[], unsigned nrOfPins)
{
    Serial.println("Pin |  GPIO Reg  |  PWM timer");
    Serial.println("----|------------|-------------");
    for (unsigned i = 0; i < nrOfPins; i++)
    {
        Serial.printf("%02d  |  %-9s |  %-10s\n", pins[i]->pin, pins[i]->gpioInfo.name, pins[i]->pwmTimerInfo.name);
    }
}

void setupPins()
{
    while (!Serial) {}

    // setup an array containing info for all digital pins
    PinInfo* pins[CORE_NUM_DIGITAL];
    for (unsigned i = 0; i < CORE_NUM_DIGITAL; i++)
    {
        pins[i] = new PinInfo(i);
    }

    // Print out info sorted by pin numbers
    Serial.println("-------------------------------");
    Serial.println("    Sorted by pin number");
    printPins(pins, CORE_NUM_DIGITAL);

    // Serial.println("\n-------------------------------");
    // Serial.println("     Sorted by PWM timer");
    // std::sort(pins, pins + CORE_NUM_DIGITAL, [](PinInfo* a, PinInfo* b) {
    //     if (a->pwmTimerInfo.type < b->pwmTimerInfo.type) return false;
    //     if (a->pwmTimerInfo.type > b->pwmTimerInfo.type) return true;
    //     if (a->pwmTimerInfo.module < b->pwmTimerInfo.module) return true;
    //     return false;
    // });
    // printPins(pins, CORE_NUM_DIGITAL);

    // Serial.println("\n-------------------------------");
    // Serial.println("   Sorted by GPIO register:        ");
    // std::sort(pins, pins + CORE_NUM_DIGITAL, [](PinInfo* a, PinInfo* b) {
    //     if (a->gpioInfo.gpioPortNr < b->gpioInfo.gpioPortNr) return true;
    //     if (a->gpioInfo.gpioPortNr > b->gpioInfo.gpioPortNr) return false;
    //     if (a->gpioInfo.gpioBitNr < b->gpioInfo.gpioBitNr) return true;
    //     return false;
    // });
    // printPins(pins, CORE_NUM_DIGITAL);

  Serial.printf("System Clock: %d\r\n", CLOCK_GetAhbFreq());
  Serial.printf("IPG Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_IpgClk));
  Serial.printf("Semc Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_SemcClk));
  Serial.printf("RTC Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_RtcClk));
  Serial.printf("USB1pll Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllClk));
  Serial.printf("Peripheral Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_PerClk));
  Serial.printf("Osc Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_OscClk));
  Serial.printf("Arm Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_ArmPllClk));
  Serial.printf("Usb1PllPfd0 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllPfd0Clk));
  Serial.printf("Usb1PllPfd1 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllPfd1Clk));
  Serial.printf("Usb1PllPfd2 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllPfd2Clk));
  Serial.printf("Usb1PllPfd3 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_Usb1PllPfd3Clk));
  Serial.printf("Usb2Pll Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_Usb2PllClk));
  Serial.printf("SysPll Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_SysPllClk));
  Serial.printf("SysPllPfd0 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd0Clk));
  Serial.printf("SysPllPfd1 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd1Clk));
  Serial.printf("SysPllPfd2 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd2Clk));
  Serial.printf("SysPllPfd3 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_SysPllPfd3Clk));
  Serial.printf("EnetPll0 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_EnetPll0Clk));
  Serial.printf("EnetPll1 Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_EnetPll1Clk));
  Serial.printf("AudioPll Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_AudioPllClk));
  Serial.printf("VideoPll Clock: %d\r\n", CLOCK_GetFreq(kCLOCK_VideoPllClk));
}

void adc_isr(void)
{
  const uint32_t sequence[] = { A7, A6, A7, A5, A7, A4, A7, A3, A7, A2, A7, A1, A7, A0 };
  static uint32_t sequenceIndex = 0;

  switch(sequenceIndex)
  {
    case 0: //Ground Sample
    case 2: //Ground Sample
    case 4: //Ground Sample
    case 6: //Ground Sample
    case 8: //Ground Sample
    case 10: //Ground Sample
    case 12: //Ground Sample
      dataBank[7].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 1: //A6 - 2.5V Ref
      dataBank[6].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 3: //A5 - Analog 5 - Grey - Coolant Temp Sensor
      dataBank[5].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 5: //A4 - Analog 4 - Oil Presure Sensor
      dataBank[4].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 7: //A3 - Analog 3 - Pin 5 - White/Black - Coolant OverTemp Switch Input
      dataBank[3].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 9: //A2 - Analog 2 - Pin 4 - White - Preheat Indicator
      dataBank[2].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 11: //A1 - Analog 1 - Pin 6 - Green/Black - Oil Presure Switch Input
      dataBank[1].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[++sequenceIndex]);
      break;

    case 13: //A0 - Analog 0 - Pin 3 - Green - System Voltage
      dataBank[0].addSample(adc_obj->readSingle());
      adc_obj->startSingleRead(sequence[sequenceIndex]);
      sequenceIndex = 0;
      break;
  }
}

void setup()
{
   /*set osc clock crystal freq */
  CLOCK_SetXtalFreq(24000000UL);

  for( uint32_t i = 0; i < sizeOfEngineSpeedArray; i++)
  {
    EngineSpeedArray[i] = 0.0;
  }

  pinMode(10, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);
  pinMode(17, INPUT);
  pinMode(18, INPUT);
  pinMode(19, INPUT);
  pinMode(20, INPUT);
  pinMode(21, INPUT);

  Serial.begin(115200);

  NMEA2000.SetDeviceCount(1); // Enable multi device support for devices
  // Set Product information for temperature monitor
  NMEA2000.SetProductInformation(SerialNumberStr,                   // Manufacturer's Model serial code.
                                 100,                           // Manufacturer's product code
                                 "Volvo D1-30A Engine Gateway", // Manufacturer's Model ID
                                 "1.0.0.1 (2022-07-22)",        // Manufacturer's Software version code
                                 "1.0.0.1 (2022-07-22)",        // Manufacturer's Model version
                                 0x02,                          // load equivalency - use default
                                 0xffff,                        // NMEA 2000 version - use default
                                 0xff,                          // Sertification level - use default
                                 0);

  // Set device information for temperature monitor
  NMEA2000.SetDeviceInformation(SerialNumber, // Unique number. Use e.g. Serial number.
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

  readEngineTime();
  if(EngineSeconds < InitalEngineSeconds || EngineSeconds == 0xFFFFFFFF )
  {
    EngineSeconds = InitalEngineSeconds;
    writeEngineTime();
    if (Serial)
    {
      Serial.println("Engine Time Initalized.");
    }
    readEngineTime();
  }

  pinMode(LED_BUILTIN,OUTPUT);
  TMR::getTMR1().begin(false);
  TMR::getTMR2().begin(true);

  if(Serial)
  {
    Serial.print("Unixtime: ");
    Serial.println(Teensy3Clock.get());
  }

  adc_obj = new ADC_Module(0, channel2sc1aADC0, ADC0_START);
  adc_obj->setResolution(10);
  adc_obj->setAveraging(32);
  adc_obj->setConversionSpeed(ADC_CONVERSION_SPEED::LOW_SPEED);
  adc_obj->setSamplingSpeed(ADC_SAMPLING_SPEED::LOW_SPEED);
  adc_obj->enableInterrupts(adc_isr);
  adc_obj->startSingleRead(A7);

  //setupPins();
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
    Serial.print(EngineCoolantTempDegC);
    Serial.print(",");
    Serial.println(EngineSpeed);
  }
}

bool EngineRunning()
{
  static time_t startTime = 0;

  if(TMR::getTMR1().capDiff > 0)
  {
    
    double Freq = 1.0 / (double(TMR::getTMR1().capDiff) * 128.0 / 75600000.0);
    double RPM = Freq * 60.0 / 28.0;

    if(RPM < 4000.0)
    {
      EngineSpeedArray[EngineSpeedArrayIndex++] = RPM;
    }
  }
  else
  {
    EngineSpeedArray[EngineSpeedArrayIndex++] = 0;
  }
  if(EngineSpeedArrayIndex >= sizeOfEngineSpeedArray) EngineSpeedArrayIndex = 0;

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
    readEngineTime();
    startTime = Teensy3Clock.get();
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
    time_t stopTime = Teensy3Clock.get();
    time_t engineRunTime = stopTime - startTime;
    readEngineTime();
    EngineSeconds += (uint32_t)engineRunTime;
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
    uint32_t nowTime = Teensy3Clock.get();
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
    if(EngineRunningFlag)
    {
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
    }
    else
    {
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
                              false,
                              false,
                              false,
                              false,
                              false,
                              false,
                              false,
                              false,
                              false,
                              false,
                              flagPreheatIndicator);
    }
    NMEA2000.SendMsg(N2kMsg, 0);

    // SetN2kTransmissionParameters(N2kMsg, ENGINE_INSTANCE, TransmissionGear, 0.0, 0.0);
    // NMEA2000.SendMsg(N2kMsg, 0);

    // SetN2kFluidLevel(N2kMsg, ENGINE_INSTANCE, FluidType, Level, Capacity);
    // NMEA2000.SendMsg(N2kMsg, 0);

    PrintInfo();
  }
}

void PrintADC()
{
  if (Serial)
  {
    Serial.print(dataBank[0].avg());
    Serial.print(",");
    Serial.print(dataBank[1].avg());
    Serial.print(",");
    Serial.print(dataBank[2].avg());
    Serial.print(",");
    Serial.print(dataBank[3].avg());
    Serial.print(",");
    Serial.print(dataBank[4].avg());
    Serial.print(",");
    Serial.print(dataBank[5].avg());
    Serial.print(",");
    Serial.println(dataBank[6].ADCOffset());
  }
}

void loop()
{
  digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));

  adcOffset = dataBank[6].ADCOffset();
  dataBank[0].CalculateSystemVoltage(adcOffset);
  dataBank[1].CalculateOilPresureSwitch(adcOffset);
  dataBank[2].CalculatePreheatIndicator(adcOffset);
  dataBank[3].CalculateCoolantSwitch(adcOffset);
  dataBank[4].CalculateOilPressure(adcOffset);
  dataBank[5].CalculateCoolantTemp(adcOffset);

  EngineRunning();
  SendN2kEngineInfo();
  NMEA2000.ParseMessages();

  if(EngineRunningFlag)
  {
    double EngineOilPressPSI = EngineOilPress / 6894.76; //Convert back to PSI
    double calcMinOilPresurePSI = 0.0074 * EngineSpeed + 6.2963;  //Calculate min Oil Presure based on RPM from D1-30 Workshop Manual

    if(EngineOilPressPSI < calcMinOilPresurePSI)
    {
      flagLowOilPress = true;
    }

    flagCheckEngine = flagLowOilPress | flagOverTemp | flagLowSystemVoltage;
  }
  else
  {
    flagCheckEngine = false;
  }
}
