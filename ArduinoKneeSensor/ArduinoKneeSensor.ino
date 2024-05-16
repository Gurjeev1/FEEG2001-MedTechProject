//libaries

  #include <Arduino_LSM6DSOX.h>
  #include <MadgwickAHRS.h>
  #include <Wire.h>
  #include <Adafruit_MSA301.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_GFX.h>
  #include <SPI.h>
  #include "thingProperties.h"



//floats+ints+constants

  float angle, Nano_angle;
  float xAcc, yAcc, zAcc;
  float xGyro, yGyro, zGyro;
  float roll, pitch, heading;
  float pitchFilteredOld;
  float x,y,z;
  const float sensorRate = 104.00;
  int FangleMAX;
  int FangleMIN =180;
  float FangleB,FangleC,FangleD,FangleE;


  Adafruit_MSA301 msa;
  Madgwick filter;




void setup() {
  //start serial monitor
  Serial.begin(9600);

  //starts IMU on nano
  IMU.begin();

  filter.begin(sensorRate);

  //starts MSA
  msa.begin();
  initProperties();
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);


}
void loop() {

  //updates the cloud
  ArduinoCloud.update();

 
  
  //arduino nano angle
  //checks accelerometer and gryo
  IMU.readAcceleration(xAcc, yAcc, zAcc);
  IMU.readGyroscope(xGyro, yGyro, zGyro); 
  filter.updateIMU(xGyro, yGyro, zGyro, xAcc, yAcc, zAcc);
  pitch = filter.getPitch();
  float pitchFiltered = 0.101 * pitch + 0.901 * pitchFilteredOld; // low pass filter
  pitchFilteredOld = pitchFiltered;

  //turns the angle from nano into angle which is easier to work with
  if ( zAcc < 0 ){
    if(pitchFiltered > 0){  
    Nano_angle = pitchFiltered;
    }
    else{
    Nano_angle = 360 + pitchFiltered;
    }
  }
  else{
    if(pitchFiltered > 0){  
    Nano_angle = 180 - pitchFiltered;
    }
    else{
    Nano_angle = 180 - pitchFiltered;
    }
  }


//accelerometer angle 

  msa.read(); // get X Y and Z data at once
  sensors_event_t event; 
  msa.getEvent(&event);

  y = event.acceleration.y; //assigning the data to variables
  x = event.acceleration.x;
  z = event.acceleration.z;

  //the following skips values which we want to ignore
  if(y > -9.81 && y < 9.81){
  //the next splits up the calculation depending on input as the sensitivity of the device decrease as you get close to 9.81 in the y direction.

    if (y > -9.0){
    //Serial.println(y);
      if(x>0){
      angle = abs((acos(((-y)/9.81)) * 57.2957795131) - 180);
      }
      else{
      angle = 180 + acos(((-y)/9.81)) * 57.2957795131;
      }

    }

    else{
    //Serial.println(y);
      if(x>0){
      angle = abs((acos(((-y)/9.71)) * 57.2957795131) - 180);
      }
      else{
      angle = 180 + acos(((-y)/9.71)) * 57.2957795131;
      }

    }

  }


  FangleA = Nano_angle - angle + 90;
  
}


/*
  Since FangleINT is READ_WRITE variable, onFangleINTChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onFangleINTChange()  {
  // Add your code here to act upon FangleINT change
}
/*
  Since FangleA is READ_WRITE variable, onFangleAChange() is
  executed every time a new value is received from IoT Cloud.
*/
void onFangleAChange()  {
  // Add your code here to act upon FangleA change
}

