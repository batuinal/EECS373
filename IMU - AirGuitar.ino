#include <Wire.h>

#define HW__VERSION_CODE 10736

#define MODE_CALIBR 0
#define MODE_BINARY 1
#define MODE_SERIAL 2
#define MODE_IDLE 3

#define ACC_ADDRESS ((int) 0x53) // 0x53 = 0xA6 / 2
#define GYR_ADDRESS  ((int) 0x68) // 0x68 = 0xD0 / 2

#define GYR_NUMSAMPLES 250
#define OUTPUT_RATE 20

// --- //

int output_mode = MODE_CALIBR;
short accdata[3] = {0, 0, 0};
byte accfinal[3] = {0, 0, 0};
float gyrodata[3] = {0, 0, 0};
float gyroaccum[3] = {0, 0, 0};
int gyrofinal[3] = {0, 0, 0};

int timestamp = 0;
int report_time = 0;
int last_meas = 0;
int difference = 0;

float accmin[3] = {100000, 100000, 100000};
float accmax[3] = {-100000, -100000, -100000};
float accmed[3] = {0, 0, -5};
float accrng[3] = {650, 650, 625};
float gyrozero[3] = {27.90, 109.75, 1.75};

char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void transmit(int address, byte data1, byte data2){
 Wire.beginTransmission(address);
 Wire.write(data1); 
 Wire.write(data2);
 Wire.endTransmission();
 delay(5);
}

void acc_init(){
  transmit(ACC_ADDRESS,0x2D,0x08);
  transmit(ACC_ADDRESS,0x31,0x09);
  transmit(ACC_ADDRESS,0x2C,0x0A);
}

void gyro_init(){
  transmit(GYR_ADDRESS,0x3E,0x80);
  transmit(GYR_ADDRESS,0x16,0x1B);
  transmit(GYR_ADDRESS,0x15,0x0A);
  transmit(GYR_ADDRESS,0x3E,0x00); 
}

void setup(){
  
 Serial.begin(9600);

 output_mode = MODE_BINARY;
 for (int i = 0; i < 3; ++i){
  accdata[i] = 0;
  gyrodata[i] = 0; 
 }  
 
 delay(50);  // Give sensors enough time to start
 Wire.begin();
 acc_init();
 gyro_init();
 delay(20);  // Give sensors enough time to collect data 
}

void acc_read(){
  int i = 0;
  int buff[6] = {0, 0, 0, 0, 0, 0};
  
  Wire.beginTransmission(ACC_ADDRESS); 
  Wire.write((byte) 0x32);
  Wire.endTransmission();
  
  Wire.beginTransmission(ACC_ADDRESS);
  Wire.requestFrom(ACC_ADDRESS, 6);  // Request 6 bytes
  
  while(Wire.available() && (i < 6))
    buff[i++] = Wire.read();
    
  Wire.endTransmission();
  
  if (i != 6){
    Serial.println("-- Error: Accelerometer fucked up.\n\r");
    return;
  }  
    
  accdata[0] = (buff[3] << 8) | buff[2];
  accdata[1] = (buff[1] << 8) | buff[0];
  accdata[2] = (buff[5] << 8) | buff[4];
}

void gyro_read(){
  int i = 0;
  int buff[6] = {0, 0, 0, 0, 0, 0};
  
  Wire.beginTransmission(GYR_ADDRESS); 
  Wire.write((byte) 0x1D);
  Wire.endTransmission();
  
  Wire.beginTransmission(GYR_ADDRESS);
  Wire.requestFrom(GYR_ADDRESS, 6);  // Request 6 bytes
  
  while(Wire.available() && (i < 6))
    buff[i++] = Wire.read();
  
  Wire.endTransmission();
  
  if (i != 6){
    Serial.println("-- Error: Gyroscope fucked up.\n\r");
    return;
  } 
  
  gyrodata[0] = -1 * ((buff[2] << 8) | buff[3]);
  gyrodata[1] = -1 * ((buff[0] << 8) | buff[1]);
  gyrodata[2] = -1 * ((buff[4] << 8) | buff[5]);

}

void acc_calibr(){
  Serial.println("\n /// Accelerometer ///");
  Serial.println("-- Send '#' to start calibration.");
  Serial.println("-- Gently tilt the accelerometer in all directions.");
  Serial.println("-- Try to achieve the maximum value for all axes.");
  Serial.println("-- Send '#' to end calibration once satisfied.");
  
  while(readChar() != '#')
    Serial.println("-- Unrecognized character. (Send '#' to calibrate)"); 
  
  for (int i = 0; i < 3; ++i){
   accmin[i] = 100000;
   accmax[i] = -100000; 
  }  
  
  int s = 0;
  while ((Serial.available() < 1) || (Serial.read() != '#')){ 
    acc_read();
    delay(20);
    
    if (!(s%10)){
      Serial.print("X: ");
      Serial.print(accdata[0]);
      Serial.print(", Y: ");
      Serial.print(accdata[1]);
      Serial.print(", Z: ");
      Serial.println(accdata[2]);
    }
    ++s;
    
    for (int i = 0; i < 3; ++i){
        accmin[i] = min(accmin[i],accdata[i]);
        accmax[i] = max(accmax[i],accdata[i]);
    }
  }
  
  for (int i= 0; i < 3; ++i){
     accmed[i] = (accmin[i] + accmax[i]) / 2;
     accrng[i] = accmax[i] - accmin[i];
  }   
  
  for(int i = 0; i < 3; ++i){
    Serial.print("med[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(accmed[i]);
    Serial.print("rng[");
    Serial.print(i);
    Serial.print("]: ");
    Serial.println(accrng[i]);   
  }  
  Serial.println("\n-- Calibration successful.");   
}

void gyro_calibr(){
  
  Serial.println("\n /// Gyroscope ///");
  Serial.println("-- Send '#' to start and end calibration.");
  Serial.println("-- Hold the gyroscope such that Y is pointing straight up.");
  Serial.println("-- Keep the gyroscope perfectly still for 5 seconds.");
  Serial.println("-- Calibration will end automatically.");
  
  while(readChar() != '#')
    Serial.println("-- Unrecognized character. (Send '#' to calibrate)"); 
  
  Serial.println("-- Calibration started.");
  
  for (int i = 0; i < 3; ++i){
   gyrozero[i] = 0; 
  }  
  
  int gyrobuffer[3][GYR_NUMSAMPLES];
  
  int s = 0;
  while (s++ < GYR_NUMSAMPLES){    
    gyro_read();
    delay(20);
    for (int i = 0; i < 3; ++i)
        gyrozero[i] += gyrodata[i]; 
    if (!(s%50)){
      Serial.print("(");
      Serial.print((250-s)/50);
      Serial.println(") seconds left.");
    }
  }
  
  for (int i = 0; i < 3; ++i){
    gyrozero[i] /= GYR_NUMSAMPLES;
    gyrofinal[i] = 0;
    gyroaccum[i] = 0;
  }
  
  Serial.print(gyrozero[0]);
  Serial.print(" ");
  Serial.print(gyrozero[1]);
  Serial.print(" ");
  Serial.println(gyrozero[2]);
  
  Serial.println("\n-- Calibration successful.");  
}

void scaledata(){
  
 for (int i = 0; i < 3; ++i){
   gyrodata[i] -= gyrozero[i];
   gyrodata[i] *= (difference / 1000.0);
   gyroaccum[i] += gyrodata[i];
 
 /*  
   if (gyroaccum[i] >= 180)
     gyroaccum[i] -= 360;
   else if (gyroaccum[i] <= -180)
     gyroaccum[i] += 360;
 */
 
   gyrofinal[i] = gyroaccum[i];
 }  
    
 for (int i = 0; i < 3; ++i){
    accdata[i] -= accmed[i];
    accdata[i] *= (75 / accrng[i]);
    if (accdata[i] > 127)
      accdata[i] = 127;
    else if (accdata[i] < -128)
      accdata[i] = -128;   
    accfinal[i] = accdata[i];
 }  
  
}

void loop(){
  
  if (Serial.available() && (Serial.read() == '#'))
    output_mode = MODE_CALIBR; 
  else if(Serial.available() && (Serial.read() == 'B'))
    output_mode = MODE_BINARY;
  else if(Serial.available() && (Serial.read() == 'S'))
    output_mode = MODE_SERIAL;
   
  
  if (output_mode == MODE_CALIBR){
   Serial.println("===== Calibration Mode =====");
   acc_calibr();
//   gyro_calibr();
   timestamp = 0;
   
   output_mode = MODE_BINARY;
   return; 
  } 
  
  int curtime = millis();
  if ((curtime - timestamp) < OUTPUT_RATE)
    return;
  
  difference = curtime - last_meas;
  last_meas = curtime;
  
  report_time += (curtime-timestamp);
  
  timestamp = curtime;
  acc_read();
  gyro_read();
  scaledata();
  
  if (output_mode == MODE_SERIAL){
    if (report_time < 200)
      return;
  
    report_time = 0;
    Serial.println("===== ACC Data =====");
    Serial.print("X: ");
    Serial.print(accdata[0]);
    Serial.print(", Y: ");
    Serial.print(accdata[1]);
    Serial.print(", Z: ");
    Serial.print(accdata[2]);
//    Serial.println("\n===== GYR Data =====");
//    Serial.print("X: ");
//    Serial.print(gyrofinal[0]);
//    Serial.print(", Y: ");
//    Serial.print(gyrofinal[1]);
//    Serial.print(", Z: ");
//    Serial.print(gyrofinal[2]);
    Serial.println("\n");
  }
  else if (output_mode == MODE_BINARY){
    Serial.write((byte*) accfinal, 3);
//    Serial.write((byte*) gyrofinal, 6);
  }
}

