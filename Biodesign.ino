/*
 * Seven Sensor Biodesign Project
 * Sarah Bi, Jessica Fritz, Vignesh Ravindranath, Aradhana Sridaran, Vikas Yerneni
 * 
 * Device takes in four pressure sensor inputs (two on the balls of the feet and two 
 * on the heels) to determine if a heel-striking occurs at midstance. Once pressure
 * has been detected, three inertial sensor inputs are read and stored, and calculations
 * for the hip-ankle angle at midstance are made.
 * 
 * Sound outputs occur for heel striking (heel pressure sensor goes high before pressure
 * sensor on balls of feet) and hip-ankle angle greater than 15 degrees (0.261799 rads).
 * 
 */
#include <MPU9250.h>
#include <MatrixMath.h>


/*----------------------------------------------------------------------------------*/
/*------------------------------------PIN LAYOUT------------------------------------*/
/*----------------------------------------------------------------------------------*/
  
/* PIN LAYOUT - ANALOG */
const uint8_t rHeelPin = 0; // HEEL PRESSURE SENSORS
const uint8_t rToesPin = 1; // BALL OF FOOT/TOE PRESSURE SENSORS
const uint8_t lHeelPin = 2; 
const uint8_t lToesPin = 3; 

/* PIN LAYOUT - DIGITAL */
const uint8_t spkrPin = 8;  // Speaker pin
const uint8_t hipPin  = 7;  // Hip Inertial Sensor nCS pin
const uint8_t rakPin  = 6;  // Right AnKle nCS pin
const uint8_t lakPin  = 5;  // Left AnKle nCS pin

/* PIN LAYOUT - MPU9250 Inertial Sensor
VCC       3.3V
GND       GND
SCL/SCLK  13
SDA/MOSI  11
EDA       -
ECL       -
AD0/MISO  12
INT       -
NCS       [7,6,5]
FSYNC     GND
*/

/*----------------------------------------------------------------------------------*/
/*-------------------------------INITIALIZE VARIABLES-------------------------------*/
/*----------------------------------------------------------------------------------*/

// Initialize MPU9250 object: SPI bus 0 and chip select pins 7,6,5
MPU9250 IMU_HIP(SPI,hipPin);   // Hip
MPU9250 IMU_RAK(SPI,rakPin);   // Right AnKle
MPU9250 IMU_LAK(SPI,lakPin);   // Left  AnKle

// Status variables to check if sensor connection is there
int status_HIP, status_RAK, status_LAK;

// Data variables
volatile float Data[8];
/* Data storage map
 * Data[0] = accelX for ankle reading 1
 * Data[1] = accelZ for ankle reading 1
 * Data[2] = gyroX for ankle reading 1
 * Data[3] = gyroZ for ankle reading 1
 * Data[4] = gyroX for ankle reading 2
 * Data[5] = gyroZ for ankle reading 2
 * Data[6] = accelX for hip 
 * Data[7] = accelZ for hip
 */

// Matrix for hip-ankle calculations
volatile mtx_type HIP_Mat[2][2];
volatile mtx_type RAK_Mat[2][1];
volatile mtx_type LAK_Mat[2][1]; 
volatile mtx_type solution[2][1];

// Flags
volatile bool rHeel, rToes, lHeel, lToes;
volatile bool MidstanceError;
volatile bool HipAbductionError;

// Pin Change Interrupt Information
volatile static byte oldPorta;
volatile static byte PCMask;
volatile byte flgPressure = 0,intsFound = 0;

// Pressure sensor/sound information
const uint8_t threshold = 1;      // minimum reading of the sensors that generates a note

// Measurements
volatile uint8_t lx = 1; //38 * 0.0254; // length from hip to ankle (m)
//const uint8_t delT = 1000;
const float g = 0.0;

/*----------------------------------------------------------------------------------*/
/*---------------------------------------SETUP--------------------------------------*/
/*----------------------------------------------------------------------------------*/

void setup(void) {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // Output pins
  pinMode(hipPin, OUTPUT);
  pinMode(rakPin, OUTPUT);
//  pinMode(lakPin, OUTPUT);

  // make A0-A3 interrupt pins
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  oldPorta = PINC;
  PCMSK1 |= (1<<PCINT8);  //A0
  PCMSK1 |= (1<<PCINT9);  //A1
  PCMSK1 |= (1<<PCINT10); //A2
  PCMSK1 |= (1<<PCINT11); //A3
//  Serial.println(PCMSK1,HEX); // What is the total mask.
  PCMask = PCMSK1;
  PCICR |= (1<<PCIE1); // Enable PCINT on portc.
  
  // start communication with IMU 
  status_HIP = IMU_HIP.begin();
  status_RAK = IMU_RAK.begin();
  status_LAK = IMU_LAK.begin();
  
  if (status_HIP < 0 ) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status(H,R,L): ");
    Serial.print(status_HIP);
    Serial.print(status_RAK);
    Serial.print(status_LAK);
    //while(1) {}
  }
//  Serial.print("Connected!!\n");
}

///*----------------------------------------------------------------------------------*/
///*---------------------------------------LOOP---------------------------------------*/
///*----------------------------------------------------------------------------------*/
void loop(void) {  
  if(rToes){
    if(rHeel){
      // Heel striking in right foot
      BEEP(50, 100);
      rHeel = false;
    }
    // Begin hip-ankle calculations
    getSensorData(&IMU_LAK, 2);
    LAK_Mat[0][1] = (Data[0] - g) + (lx * sq(Data[2])); 
    LAK_Mat[1][1] = (Data[1] - g) + (lx * ((Data[4]-Data[2])/ 0.2));
    Matrix.Invert((mtx_type*)HIP_Mat, 2);
    Matrix.Multiply((mtx_type*)HIP_Mat, (mtx_type*)LAK_Mat, 2, 2, 1, (mtx_type*)solution);
    if(acos(abs(solution[1][1])) > 0.261799){ // 0.261799 rads = 15 degrees
      BEEP(1000, 400);
    }
    
    rToes = false;
  }

  if(lToes){
    if(lHeel){
      // Heel striking in left foot
      BEEP(2000, 100);
      lHeel = false;  
    }

    // Begin hip-ankle calculations
    getSensorData(&IMU_LAK, 2);
    LAK_Mat[0][1] = (Data[0] - g) + (lx * sq(Data[2])); 
    LAK_Mat[1][1] = (Data[1] - g) + (lx * ((Data[4]-Data[2])/ 0.2));
    Matrix.Invert((mtx_type*)HIP_Mat, 2);
    Matrix.Multiply((mtx_type*)HIP_Mat, (mtx_type*)LAK_Mat, 2, 2, 1, (mtx_type*)solution);
    if(acos(abs(solution[1][1])) > 0.261799){ // 0.261799 rads = 15 degrees
      BEEP(1000, 400);
    }
    
    lToes = false;  
  }

  delay(500);
  
}

/*----------------------------------------------------------------------------------*/
/*----------------------------------------ISR---------------------------------------*/
/*----------------------------------------------------------------------------------*/

void rHeel_ISR(){ 
  uint8_t sensorReading = analogRead(rHeelPin);
  if (sensorReading > threshold){  
    if(!rToes) rHeel = true;
  }
}

void rToes_ISR(){ 
  uint8_t sensorReading = analogRead(rToesPin);   
  if (sensorReading > threshold){    
    rToes = true;
    // Collect data when runner is in midstance
    getSensorData(&IMU_HIP, 0);
    getSensorData(&IMU_RAK, 1);
  }
}

void lHeel_ISR(){ 
  uint8_t sensorReading = analogRead(lHeelPin); 
  if (sensorReading > threshold){
    if(!lToes) lHeel = true;
  }
}

void lToes_ISR(){ 
  uint8_t sensorReading = analogRead(lToesPin);   
  if (sensorReading > threshold){
    lToes = true;
    // Collect data when runner is in midstance
    getSensorData(&IMU_HIP, 0);
    getSensorData(&IMU_LAK, 1);
  }
}

ISR( PCINT1_vect ) {
  // ISR to make A0-A3 interrupt pins
  byte change,v1,v2,v3,v4;
  change = oldPorta ^ PINC;

  v1 = oldPorta & (1<<PCINT8);
  v2 = oldPorta & (1<<PCINT9);
  v3 = oldPorta & (1<<PCINT10);
  v4 = oldPorta & (1<<PCINT11);

  // v1==0 previously so detecting rising edge.
  if (v1==0 && change & (1<<PCINT8))  rHeel_ISR();
  if (v2==0 && change & (1<<PCINT9))  rToes_ISR();
  if (v3==0 && change & (1<<PCINT10)) lHeel_ISR();
  if (v4==0 && change & (1<<PCINT11)) lToes_ISR();
  if (change && (v1==0 || v2==0 || v3==0 || v4==0) ) {
    intsFound++; // Only count rising edge interrupts (as this interrupt reacts to both).
    flgPressure = 1; // rising edge only
  }

  oldPorta = PINC;
}

/*----------------------------------------------------------------------------------*/
/*-------------------------------------FUNCTIONS------------------------------------*/
/*----------------------------------------------------------------------------------*/
void getSensorData(MPU9250* IMU, int arg){
  // General function to get/store data from inertial sensors
  IMU->readSensor();
  switch(arg){
    case 0:
      Data[6] = HIP_Mat[0][0] = HIP_Mat[1][1] = IMU->getAccelX_mss();
      Data[7] = HIP_Mat[0][1] = HIP_Mat[1][0] = IMU->getAccelZ_mss();
      break;
    case 1:
      Data[0] = IMU->getAccelX_mss(); 
      Data[1] = IMU->getAccelZ_mss();
      Data[2] = IMU->getGyroX_rads();
      Data[3] = IMU->getGyroZ_rads();
      break;
    case 2:
      Data[4] = IMU->getGyroX_rads();
      Data[5] = IMU->getGyroZ_rads();
      break;
  }
}

// Sound outputs
void BEEP(uint8_t freq, uint8_t dur){
  tone(spkrPin, freq, dur);  
}
