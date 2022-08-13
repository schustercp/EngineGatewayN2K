#include <Arduino.h>
#include <ADC_Module.h>

volatile double EngineCoolantTemp = 0.0; //  - EngineCoolantTemp     in Kelvin
volatile double EngineCoolantTempDegC = 0.0;
volatile double EngineCoolantTempDegF = 0.0;
volatile double EngineCoolantTempDegK = 0.0;
volatile double AltenatorVoltage = 0.0;   //  - AltenatorVoltage      in Voltage
volatile double EngineOilPress = 0.0;
volatile bool flagCheckEngine = false;
volatile bool flagOverTemp = false;
volatile bool flagLowOilPress = true;
volatile bool flagLowSystemVoltage = true;
volatile bool flagPreheatIndicator = false;

class ADC_Data
{
  public:
    void addSample(uint32_t sample)
    {
      dataArray[dataArrayIndex++] = sample;
      if (dataArrayIndex >= dataArraySize ) dataArrayIndex = 0;
    }

    uint32_t avg()
    {
      uint32_t avgValue = 0 ;
      for(uint32_t index = 0; index < dataArraySize; index++)
      {
        avgValue += dataArray[index];
      }
      
      return (avgValue / dataArraySize);
    }

    double ADCOffset()
    {
      double countsPerRef = Ref_Voltage / (ADC_Ref_Voltage / maxADCCount);
      return (countsPerRef - double(avg()));
    }

    void CalculateSystemVoltage(double adcOffset)
    {
      // Altenator Voltage
      // A0 - Analog 0 - Pin 3 - Green - Altenator Voltage
      double V1 = (ADC_Ref_Voltage / maxADCCount * (double(avg()) + adcOffset));
      if(V1 > 0.5)
      {
        AltenatorVoltage = V1*(A0_R1+A0_R2)/A0_R2 + A0_V_offset;
      }
      else
      {
        AltenatorVoltage = 0.0;
      }

      if(flagLowSystemVoltage)
      {
        flagLowSystemVoltage = (AltenatorVoltage < 12.2);
      }
      else
      {
        flagLowSystemVoltage = (AltenatorVoltage < 11.5);
      }

    //   if (Serial)
    //   {
    //     Serial.print("System Voltage: ");
    //     Serial.println(AltenatorVoltage);
    //   }
    }

    void CalculateOilPressure(double adcOffset)
    {
      double V1 = (ADC_Ref_Voltage / maxADCCount * (double(avg()) + adcOffset));
      double V2 = V1*(A4_R1+A4_R2)/A4_R2;
      EngineOilPress = 25.0 * V2 - 12.5;
      //flagLowOilPress = EngineOilPress < 10.0;
      if(EngineOilPress < 1.0)
      {
        EngineOilPress = 0.0;
      }

      //Convert PSI to 100 Pa
      EngineOilPress *= 6894.76;

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

    void CalculateCoolantSwitch(double adcOffset)
    {
      //Coolant OverTemp Switch Input
      // A3 - Analog 3 - Pin 5 - White/Black - Coolant OverTemp Switch Input - R1/R2 - 67.1K / 11.86K
      //Switch closes if temp of engine is to hight.
      double V1 = (ADC_Ref_Voltage / maxADCCount * (double(avg()) + adcOffset));
      double V2 = V1*(A3_R1+A3_R2)/A3_R2;

      if(flagOverTemp)
      {
        flagOverTemp = (V2 < 1.0);
      }
      else
      {
        flagOverTemp = (V2 < 1.5);
      }

      // if (Serial)
      // {
      //   Serial.print("Coolant Switch: ");
      //   Serial.print(V2);
      //   Serial.print(" : ");
      //   Serial.println(sensorValue);
      // }
    }

    void CalculateCoolantTemp(double adcOffset)
    {
      // Engine Coolant Temp
      // A5 - Analog 5 - pin 2 - Grey - Coolant Temp Sensor - R1/R2 - 992 Ohms
      // Convert to Engine Temp
      double V1 = (ADC_Ref_Voltage / maxADCCount * (double(avg()) + adcOffset));
      const double V2 = ADC_Ref_Voltage;
      double R2 = V1 * A5_R1/(V2-V1);
      //EngineCoolantTempDegC = 0.000605653*(R2-134)*(R2-484.6)-0.000008620208891455*(R2-134)*(R2-51.2)*(R2-484.6)-0.0998289*(R2-484.6)+25;
      EngineCoolantTempDegC = -29.28 * log(R2) + 205.54;
      EngineCoolantTempDegF = 9.0/5.0 * EngineCoolantTempDegC + 32.0;
      EngineCoolantTempDegK = 273.15 + EngineCoolantTempDegC;

      //Convert Deg C to Deg K
      EngineCoolantTemp = EngineCoolantTempDegK;

      if(!flagOverTemp)
      {
        flagOverTemp = EngineCoolantTempDegC > 109.0;
      }

      // if (Serial)
      // {
      //   Serial.print("Coolant Sensor: ");
      //   Serial.print(ADC_Value);
      //   Serial.print(" : ");
      //   Serial.print(V1);
      //   Serial.print(" : ");
      //   Serial.print(R2);
      //   Serial.print(" : ");
      //   Serial.print(EngineCoolantTempDegC);
      //   Serial.print(" : ");
      //   Serial.print(EngineCoolantTempDegF);
      //   Serial.print(" : ");
      //   Serial.println(EngineCoolantTempDegK);
      // }
    }

    void CalculatePreheatIndicator(double adcOffset)
    {
      // Preheat Indicator
      // A2 - Analog 2 - Pin 4 - White - Preheat Indicator - R1/R2 - 67.1K / 12.04K
      double V1 = (ADC_Ref_Voltage / maxADCCount * (double(avg()) + adcOffset));
      double V2 = V1*(A2_R1+A2_R2)/A2_R2;

      if(flagPreheatIndicator)
      {
        flagPreheatIndicator = (V2 > 5.5);
      }
      else
      {
        flagPreheatIndicator = (V2 > 5.0);
      }

      // if (Serial)
      // {
      //   Serial.print("Preheat: ");
      //   Serial.println(V2);
      // }
    }

    void CalculateOilPresureSwitch(double adcOffset)
    {
      // Oil Presure Switch Input
      // A1 - Analog 1 - Pin 6 - Green/Black - Oil Presure Switch Input - R1/R2 - 67.2K / 11.67K
      double V1 = (ADC_Ref_Voltage / maxADCCount * (double(avg()) + adcOffset));
      double V2 = V1*(A1_R1+A1_R2)/A1_R2;

      if(flagLowOilPress)
      {
        flagLowOilPress = (V2 < 1.0);
      }
      else
      {
        flagLowOilPress = (V2 < 1.5);
      }

      // if (Serial)
      // {
      //   Serial.print("Oil press Switch: ");
      //   Serial.println(V2);
      // }
    }
    
  protected:
    const uint32_t dataArraySize = 64;
    volatile uint32_t dataArray[64];
    volatile uint32_t dataArrayIndex = 0;
};