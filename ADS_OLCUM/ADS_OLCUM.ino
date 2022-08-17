/*working code
  REGISTERS
  REG   VAL     USE
  0     52      Status Register, Everyting Is Default, Except Auto-Cal
  1     8       Multiplexer Register, AIN0 POS, AINCOM
  2     0       ADCON, Everything is OFF, PGA = 1
  3     130     DataRate = 50 SPS
  4     239     GPIO, Everything Is Default

  SYSCAL is needed before performing measurement.

  Not used legs should be pinned to ground, or else it will
  measure false data.

  //too high SPS will mess up the channels!

  Wiring:
  A13 - SCLK
  A12 - DOUT
  A11 - DIN
  A10 - CS
  A9 - Not connected
  A8 - Reset
  A2 - DRDY

  SPI

  SS    10    CS
  MOSI  11    DIN
  MISO  12    DOUT
  SCK   13    SCLK

  PINS for Arduino NANO

  ADS1256 - ARDUINO

  GND     - GND
  VCC     - +5V
  NC      - DO NOT CONNECT THIS
  RST     - D8
  CS      - D10
  DRDY    - D2
  DOUT    - D12
  DIN     - D11
  SCLK    - D13
  PDWN    - +3.3 V

*/

//2019-11-11: Now it works on NANO
//2020-01-28: Updated a few lines

#include <SPI.h>
#include "ads12xx.h"
#include "EEPROM.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
SoftwareSerial mySerial(9, 11);
LoRa_E32 e32ttl(&mySerial);
typedef struct  {

  char  komut;

} Signal;

Signal data;

const int  START = 8;
const int  CS = 10;
const int  DRDY = 2;
const int RESET_  = 9;
float samplingfreq;
extern double volt;
float Mcisim =10193;
float Vzero;
float Vcisim;
float Voutput;
float Moutput;
String _Mcisim;
float egim;
float Ratio2;
float Ratio1;
float R;
float offsetLoad;
float offsetLoadVolt;
ads12xx ADS;  //initialize ADS as object of the ads12xx class


void setup()
{
  //pinMode(7, OUTPUT); //set pin7 to output  GREEN LED
  //pinMode(6, OUTPUT); //set pin4 to output  RED LED
  //pinMode(4, OUTPUT); //set pin4 to output  YELLOW LED



  Serial.begin(230400);
  //while (!Serial) {}

  Serial.println("ADS1256");

  ADS.begin(CS, START, DRDY);  //initialize ADS as object of the ads12xx class

  ADS.Reset(); //Reset the AD chip. Every time you see the message on the Serial Terminal, everything is set to default!

  delay(10);

  //preconfig
  ADS.SetRegisterValue(STATUS, B00110100);  //Autocal is ON
  ADS.SetRegisterValue(MUX, B00001000);
  ADS.SetRegisterValue(ADCON, B00000111);
  //ADS.SetRegisterValue(DRATE, B11010000); //7500 SPS
  ADS.SetRegisterValue(DRATE,3);  // 2.5 SPS
  ADS.SetRegisterValue(IO, B11101111);
  ADS.SendCMD(SELFCAL);

  //end of preconfig

  EEPROM.get(10,Vzero);
  EEPROM.get(14,Ratio2);
  ADS.SetRegisterValue(DRATE,130);  // 2.5 SPS
}

void loop() {


  double startTime = millis();
/*
  if (e322ttl.available() > 0) {
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    data = *(Signal*) rsc.data;
    rsc.close();
    
  }
  */
  if(Serial.available()>0){
    data.komut=Serial.read();
  }

  switch (data.komut) {

    case 'd': //d: differential - 4 channels (less channels, better accuracy)

      //another cin for receiving the delay.
      samplingfreq = Serial.parseFloat();
      // so, the command should be "d 500" which means that the while() below will be repeated at every 500 ms.
      // we neglect the time needed to perform the running time of the code.

      //ADS.SetRegisterValue(DRATE, B01100011);  //7500 SPS// for 1 sample

      while (data.komut != 'N') {
        digitalWrite(4, HIGH);  //while collecting data, light is on

        //double elapsedTime = millis() - startTime;
        //prints time since program started
        //Serial.print(elapsedTime);
        //Serial.print("\t");

        Voutput = getVoltageLoadCell(Voutput);
        Voutput=KALMAN(Voutput);
        delayMicroseconds(1);
        //Serial.print(Voutput,8);Serial.print("\t");
        Ratio1 = Voutput - Vzero;
        float R = Ratio1 / Ratio2;
        Moutput  =  R * Mcisim;
        Serial.println(Moutput);
      }

  }
  
}

double getVoltageLoadCell(double deger) {
  ADS.SetRegisterValue(MUX, B00000001);//AIN0+AIN1 -- CH0
  deger = ADS.GetConversion();
  deger = volt;
  return deger;
}
double getVoltageBasinc(double deger) {
  ADS.SetRegisterValue(MUX, B00100011);//AIN2+AIN3 -- CH1
  deger = ADS.GetConversion();
  deger = volt;
  return deger;
}
double getVoltageSicaklik(double deger) {
  ADS.SetRegisterValue(MUX, B01000101);//AIN4+AIN5 -- CH2
  deger = ADS.GetConversion();
  deger = volt;
  return deger;
}
double KALMAN(double U){
  static const double R=10;
  static const double H=1.00;
  static double Q=10;
  static double P=0;
  static double U_hat=0;
  static double K=0;

  K=P*H/(H*P*H+R);
  U_hat=U_hat + K*(U-H*U_hat);
  P=(1-K*H)*P+Q;
  return U_hat;
  }
