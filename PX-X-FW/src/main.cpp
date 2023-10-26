#include <Arduino.h>
#include <SPI.h>

#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif

#include <ModbusIP_ESP8266.h>


#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "SPI_shiftreg.h"

#include "creds.h"
#include "WifiControl.h"
#include "ModeControl.h"


/* PX-X is a special unit that uses a lot more registers and because of this
*  it doesn't follow the usual register rules of the PX series.
* it's registers work as follows:
* register 0 to 8 control the servos
* registers 9 to 17 control the leave folding
*   
*/

#define PX_NUM 10

#define PX_STATE_REG 200 + (PX_NUM * 10) // base status modbus register
//#define PX_REG 104
 enum
  {
    PX_ERR = -1,
    PX_OK
  };

  //ModbusIP object
ModbusIP pxModbus;

char ssid[] = SSID ;        // your network SSID (name)
char pass[] = PW;                    // your network password
WifiControl pxWifi(ssid, pass, PX_NUM);


// called this way, it uses the default address 0x40
//on nodemcu pins are: SCL gpio_5 aka D1 and SDA gpio_4 aka D2

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  262 //535 //600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;

int16_t servoGoals[9];
int16_t servoVal[9];
int16_t servoMax[9] = { SERVOMAX + 20, SERVOMAX + 40, SERVOMAX + 60, SERVOMAX + 40, SERVOMAX, SERVOMAX + 80, SERVOMAX + 80, SERVOMAX + 80, SERVOMAX + 40};

uint32_t servoPeriod = 20; //update servo's every 20ms
uint32_t servoT = 0;


#define SHIFT_REG_OE D0
#define SHIFT_REG_LATCH D3
#define SHIFT_REG_CLR D4

#define NUM_SHIFT_BOARDS 2

#define LEAF_1 0
#define LEAF_2 1
#define LEAF_3 2
#define LEAF_4 3
#define LEAF_5 4
#define LEAF_6 5
#define LEAF_7 6
//index jumps here due to soldering issues on the first mosfet board last unit
#define LEAF_8 8
#define LEAF_9 9

//possible leaf states
enum leaf_state{
  LEAF_OPEN,
  LEAF_CLOSED,
  LEAF_UP
};

leaf_state leaf_states[9];

SPI_shiftreg pxReg(SHIFT_REG_OE, SHIFT_REG_LATCH, SHIFT_REG_CLR, NUM_SHIFT_BOARDS);

void setServo(uint8_t idx, uint8_t val);
void setLeaf(uint8_t idx, uint8_t val);


#define DEMO_SW_PIN 3 //uart rx pin -> uart must operate in TX only mode!
int8_t demoState = 0;
void demoCallback(uint32_t dTime, px_mode_t mode);
ModeControl pxMC(DEMO_SW_PIN, &demoCallback, 15000, &pxWifi);

void setup() {
  Serial.begin(57600, SERIAL_8N1, SERIAL_TX_ONLY);

  pwm.begin();
  //some hints from adafruit about the servo pwm board:
  /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is importaint to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
  pwm.setOscillatorFrequency(26905000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  for(uint8_t i= 0; i < 9; i++){
    leaf_states[i] = LEAF_OPEN;
    servoGoals[i] = SERVOMIN;
    servoVal[i] = SERVOMIN;
    pwm.setPWM(i, 0, servoGoals[i]);
  }


  

  delay(10);

  pxReg.enable_output();

  Serial.println("Connecting to C&C...");
  int8_t res = pxWifi.init();
  if(res == -1){
    Serial.println("No C&C found, starting up in demo mode!");
  }

  //create the modbus server, and add a holding register (addHreg) and Ireg
  pxModbus.server(502);
 
  pxModbus.addHreg(PX_STATE_REG, PX_OK);

  //add registers (this could also be done with coils since we have only 2 positions / states), but this works just fine
  for(uint8_t i = 0; i < 18; i++){
      pxModbus.addHreg(i, 0);
      pxModbus.addIreg(i, 0);
  }

  pxMC.init();

  Serial.println("Setup complete");


}


void loop() {

  //Call once inside loop() - all modbus magic here
  pxModbus.task();

  pxMC.run();

  pxWifi.run();

  bool pxReg_update = false;

  for(uint8_t i = 0; i < 18; i++){
    if(pxModbus.Hreg(i) != pxModbus.Ireg(i)){
      pxModbus.Ireg(i, pxModbus.Hreg(i));
      if(i < 9){
        setServo(i, pxModbus.Ireg(i));
      }
      if(i >= 9){
        setLeaf(i, pxModbus.Ireg(i));
        pxReg_update = true;
      }
    }
  }

  //update mosfet boards only when there's something to update
  if(pxReg_update){
    pxReg.shift_data();
  }


  if(millis() - servoT >= servoPeriod){
    servoT = millis();
    for(uint8_t i = 0; i < 9; i++){

      int curPos = servoVal[i];

      servoVal[i] = (servoGoals[i] * 0.025) + (curPos * 0.975);
      pwm.setPWM(i, 0, servoVal[i]);

    }
  }


  
}


void setServo(uint8_t idx, uint8_t val)
{
  if(val >= 1){
    if(leaf_states[idx] == LEAF_CLOSED){
      servoGoals[idx] = servoMax[idx];
      leaf_states[idx] = LEAF_UP;
    }
  }else if(val == 0){
    if(leaf_states[idx] == LEAF_UP){
      servoGoals[idx] = SERVOMIN;
      //this is somewhat dangerous to assume... should be checked with reality in pxReg ideally
      leaf_states[idx] = LEAF_CLOSED;
    }
  }
}

void setLeaf(uint8_t idx, uint8_t val)
{
  int8_t leaf_idx = -1; // -1 to indicate invalid idx

  //there's a switch here instead of a simple substraction because of some hardware issues that caused an idx jump
  switch(idx){
    case 9:
      leaf_idx = LEAF_1;
      break;
    case 10:
      leaf_idx = LEAF_2;
      break;
    case 11:
      leaf_idx = LEAF_3;
      break;
    case 12:
      leaf_idx = LEAF_4;
      break;
    case 13:
      leaf_idx = LEAF_5;
      break;
    case 14:
      leaf_idx = LEAF_6;
      break;
    case 15:
      leaf_idx = LEAF_7;
      break;
    case 16:
      leaf_idx = LEAF_8;
      break;
    case 17:
      leaf_idx = LEAF_9;
      break;
    default:
      break; 
  }



  if(val >= 1 && leaf_idx >= 0){
      pxReg.set_data_bit(leaf_idx, 1);
      leaf_states[idx - 9] = LEAF_CLOSED;
  }else if(val == 0 && leaf_idx >= 0){
    //check if leaf is down before opening
    if(servoGoals[idx - 9] == SERVOMIN){
      pxReg.set_data_bit(leaf_idx, 0);
      leaf_states[idx - 9] = LEAF_OPEN;
    }
  }
}



void demoCallback(uint32_t dTime, px_mode_t mode)
{
  if(mode == PX_DEMO_MODE){
    
    for(uint8_t i= 0; i < 9; i++){
      uint32_t randNumber = RANDOM_REG32;
      if(randNumber % 2 == 1){
          if(leaf_states[i] == LEAF_OPEN){
            pxModbus.Hreg(i + 9, 1); //set leaf to close
          }else if(leaf_states[i] == LEAF_CLOSED){
            pxModbus.Hreg(i, 1); //set leaf to go up
          }
          // nothing happens when the leaf is already up
      }else{
        if(leaf_states[i] == LEAF_CLOSED){
            pxModbus.Hreg(i + 9, 0); //set leaf to open
          }else if(leaf_states[i] == LEAF_UP){
            pxModbus.Hreg(i, 0); //set leaf to go down
          }
          //nothing happens when the leaf is already down and open
      }

    }

  }else if(mode == PX_CC_MODE){
    Serial.println("to CC mode demo calback");
    for(uint8_t i= 0; i < 9; i++){
      pxModbus.Hreg(i, 0); //set leaves to down
      pxModbus.Hreg(i + 9, 0); //set leaves to open
    }
  }

}