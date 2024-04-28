/*
SLUG: Glove by Brennan Borchert

The glove acts as a system to gather information. The sensors onboard are 5 flex sensors (the one for the thumb is slightly
different), 2 touch sensors (pads that detect connection), and an IMU. The flex sensors are for detecting how much overall bend
in each finger. The 
*/

/*
Pseudocode

Set up:
- Initialize UART and I2C
- Tell the user to lay their hand on a flat surface for orientation

Running:
- Constantly update and read the IMU for the orientation
- Receive a UART command:
  - Does it say to send back the gathered information?
    - Read the ADCs
    - Read the contacts
    - Package information and send it back through UART
  - Does it say to vibrate the motor?
    - Vibrate the motor at the specified frequency

*/

//#define DEBUG // Define if debugging

#include <Arduino.h>
#include <Wire.h>
#include <ICM_20948.h>
#include <EEPROM.h>
#include <Adafruit_ADS1X15.h>

#define RXD0 3
#define TXD0 1

#define PAYLOAD_SIZE 23
#define PACKAGE_SIZE PAYLOAD_SIZE + 1
#define RECEIVE_SIZE 3

#define START_BYTE 0xFE

#define CMD_RETURN_DATA 'P'
#define CMD_CTRL_MOTOR 'M'

#define I2C_SDA 21
#define I2C_SCL 22

#define ADC0_ADDR 0b1001000
#define ADC1_ADDR 0b1001001
#define IMU_ADDR  0b1101000

#define SW0_PIN 5
#define SW1_PIN 4
#define LED_PIN 2

#define PWM_PIN 4
#define PWM_CHAN 0
#define PWM_FREQ 5000
#define PWM_RES 8

#define NUM_CALI_STEPS 3

// I2C Devices
TwoWire I2C_MAIN = TwoWire(0);
Adafruit_ADS1115 ads0, ads1;
ICM_20948_I2C myICM;

void floatToCharArray(float d, uint8_t* array, uint8_t offset);
uint8_t receiveUARTCommand();

void setup(void) {
  // Set up UART
  #ifdef DEBUG
    Serial.begin(9600); // Start communication with PC
    Serial.println("Initialized Debug Serial");
    //while(!Serial);
  #else
    Serial.begin(115200, SERIAL_8N1, RXD0, TXD0);
    //while(!Serial);
  #endif

  ledcSetup(PWM_CHAN, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWM_CHAN, PWM_PIN);

  pinMode(SW0_PIN, INPUT_PULLUP);
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  // Set up I2C lines
  I2C_MAIN.begin(I2C_SDA, I2C_SCL, 400000);

  ads0.setGain(GAIN_ONE);     // +/- 4.096V  1 bit = 0.125mV

  if (!ads0.begin(ADC0_ADDR, &I2C_MAIN)) {
    #ifdef DEBUG
      Serial.println("Failed to initialize ADS0.");
    #endif
    while (1);
  }
  #ifdef DEBUG
    Serial.println("Initialized ADS0.");
  #endif

  ads1.setGain(GAIN_ONE);     // +/- 4.096V  1 bit = 0.125mV

  if (!ads1.begin(ADC1_ADDR, &I2C_MAIN)) {
    #ifdef DEBUG
      Serial.println("Failed to initialize ADS1.");
    #endif
    while (1);
  }
  #ifdef DEBUG
    Serial.println("Initialized ADS1.");
  #endif

  myICM.begin(I2C_MAIN, 1);
  #ifdef DEBUG
    Serial.print( F("Initialization of the sensor returned: ") );
    Serial.println( myICM.statusString() );
  #endif

  bool initialized = false;
  while(!initialized) {
    if( myICM.status != ICM_20948_Stat_Ok ){
    #ifdef DEBUG
      Serial.println( "Trying again..." );
    #endif
    delay(500);
    } else {
      initialized = true;
    }
  }

  bool success = true;

  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok);
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success) {
    #ifdef DEBUG
      Serial.println("DMP enabled!");
    #endif
  } else {
    #ifdef DEBUG
      Serial.println("Enable DMP failed!");
    #endif
  }

  // Start calibration sequence
  /*
  double prev_q1 = 0, prev_q2 = 0, prev_q3 = 0, temp_q1 = 0, temp_q2 = 0, temp_q3 = 0;

  icm_20948_DMP_data_t data;
  
  myICM.readDMPdataFromFIFO(&data);

  prev_q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
  prev_q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
  prev_q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

  for (int i = 0; i < NUM_CALI_STEPS; i++) {
    temp_q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
    temp_q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
    temp_q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

    if ((temp_q1 != prev_q1) && (temp_q2 != prev_q2) && (temp_q3 != prev_q3)) {
      i = 0;
      ledcWrite(PWM_CHAN, 255/2);
      delay(10);
      ledcWrite(PWM_CHAN, 0);
      delay(10);
      ledcWrite(PWM_CHAN, 255/2);
      delay(10);
      ledcWrite(PWM_CHAN, 0);
      delay(10);
    }
  }

  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);
  ledcWrite(PWM_CHAN, 255/2);
  delay(30);
  ledcWrite(PWM_CHAN, 0);*/


  #ifdef DEBUG
    Serial.println("Initialized IMU.");
    Serial.println("Begining program.");
  #endif

  digitalWrite(LED_PIN, HIGH);
}
 
uint8_t uart_buffer[PACKAGE_SIZE];

float q1 = 0, q2 = 0, q3 = 0;
#ifdef DEBUG
  float q0 = 0;
  int16_t accuracy;
#endif

void loop(void) {

  icm_20948_DMP_data_t data;
  
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
    if ((data.header & DMP_header_bitmap_Quat9) > 0) {
      q1 = ((float)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      q2 = ((float)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      q3 = ((float)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      #ifdef DEBUG
        q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
        accuracy = data.Quat9.Data.Accuracy;
      #endif
    }
  }

  if (Serial.available() > 0) {
    #ifdef DEBUG
      uint8_t cmd = Serial.read();
    #else
      uint8_t cmd = receiveUARTCommand();
    #endif

    if (cmd == 'M') {
      ledcWrite(PWM_CHAN, 255/2);
      delay(10);
      ledcWrite(PWM_CHAN, 0);
      delay(10);
      ledcWrite(PWM_CHAN, 255/2);
      delay(10);
      ledcWrite(PWM_CHAN, 0);
      delay(10);
    } else if (cmd == 'P') {
      //digitalWrite(LED_PIN, HIGH);
      uart_buffer[0] = START_BYTE;
      uint8_t offset = 1;
      int16_t adc;

      // Load the ADC values
      for (uint8_t i = 0; i < 5; i++) {
        if (i < 4) {
          adc = ads0.readADC_SingleEnded(i);
          // The lower byte first
          uart_buffer[offset] = uint8_t(adc % 256);
          // Have the higher byte second
          uart_buffer[offset+1] = uint8_t(adc / 256);
          offset += 2;
        } else {
          adc = ads1.readADC_SingleEnded(i-4);
          uart_buffer[offset] = uint8_t(adc % 256);
          uart_buffer[offset+1] = uint8_t(adc / 256);
          offset += 2;
        }
      }

      floatToCharArray(q1, uart_buffer, offset);
      offset += sizeof(float);
      floatToCharArray(q2, uart_buffer, offset);
      offset += sizeof(float);
      floatToCharArray(q3, uart_buffer, offset);
      offset += sizeof(float);

      uart_buffer[offset] = (~digitalRead(SW0_PIN) & 0x1) << 0 | (~digitalRead(SW1_PIN) & 0x1) << 1;
      
      Serial.write(uart_buffer, PACKAGE_SIZE);

      //digitalWrite(LED_PIN, LOW);
      
      #ifdef DEBUG
        /*int16_t test[5];
        for (uint8_t i = 0; i < 5; i++) {
          if (i < 4) {
            test[i] = ads0.readADC_SingleEnded(i);
          } else {
            test[i] = ads1.readADC_SingleEnded(i-4);
          }
        };
        Serial.print("Pinky: ");
        Serial.print(String(test[0]));
        Serial.print(' ');
        Serial.print("Ring: ");
        Serial.print(String(test[1]));
        Serial.print(' ');
        Serial.print("Middle: ");
        Serial.print(String(test[2]));
        Serial.print(' ');
        Serial.print("Pointer: ");
        Serial.print(String(test[3]));
        Serial.print(' ');
        Serial.print("Thumb: ");
        Serial.print(String(test[4]));
        Serial.print(' ');
        Serial.print("Q1: ");
        Serial.print(String(q1));
        Serial.print(' ');
        Serial.print("Q2: ");
        Serial.print(String(q2));
        Serial.print(' ');
        Serial.print("Q3: ");
        Serial.print(String(q3));
        Serial.print(' ');
        Serial.print("Q0: ");
        Serial.print(String(q0));
        Serial.print(' ');
        Serial.print("Q Acc: ");
        Serial.print(String(accuracy));
        Serial.print(' ');
        Serial.println();*/
      #endif
    }
  }  
};

void floatToCharArray(float d, uint8_t* array, uint8_t offset) {
  union{
      float myFloat;
      uint8_t myChars[sizeof(float)];
  } float_int;

  float_int.myFloat = d;

  for(int k = 0; k < sizeof(float); k++ ) {
    array[k+offset] = float_int.myChars[k];
  }
};

uint8_t receiveUARTCommand() {
  static bool receiveInProgress = false;
  uint8_t rb;

  while(Serial.available() > 0) {
    rb = Serial.read();
    if (receiveInProgress) {
      receiveInProgress = false;
      return rb;
    } else if (rb == START_BYTE) {
      receiveInProgress = true;
    }
  }
  return 0;
};

bool receiveUART23Package(uint8_t* &array) {
  static bool receiveInProgress = false;
  uint8_t rb;
  uint8_t count = 0;

  while(Serial.available() > 0) {
    rb = Serial.read();
    if (receiveInProgress) {
      array[count] = rb;
      count++;
      if (count > 23) {
        receiveInProgress = false;
        return true;
      }
      
    } else if (rb == START_BYTE) {
      receiveInProgress = true;
    }
  }
  return false;
};