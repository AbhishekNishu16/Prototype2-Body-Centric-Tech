#include "SparkFunLSM6DS3.h"
#include "Wire.h"
#include "six_axis_comp_filter.h"

//define the IMU object
LSM6DS3 nano33IMU(I2C_MODE, 0x6A); 
CompSixAxis CompFilter(0.1, 2); 


float pitch;
float roll;
int ledPin1=2;
int ledPin2=3;
void blink(){
digitalWrite(ledPin1, HIGH);   
  delay(200);                      
  digitalWrite(ledPin1, LOW);   
  delay(200);  
}

void setup() 
{
  Serial.begin(9600);
  nano33IMU.begin();

   pinMode (ledPin1,OUTPUT);
   pinMode (ledPin2,OUTPUT);
}


void loop() 
{
  if (pitch<170) {
blink();
  }

  if (pitch>190) {
blink();
  }

  if (roll<110) {
blink(); 
  }

    if (roll>145) {
blink();  
  }
  
  else {
   digitalWrite(ledPin1,false);
   digitalWrite(ledPin2,true);
}



calculatePitchAndRoll();
}


void calculatePitchAndRoll()
{
  float accelX, accelY, accelZ, // variables to store sensor values
      gyroX, gyroY, gyroZ,
      xAngle, yAngle;       

  //  Get all motion sensor (in this case LSM6DS3) parameters,
  //  If you're using a different sensor you'll have to replace the values
  accelX = nano33IMU.readFloatAccelX();
  accelY = nano33IMU.readFloatAccelY();
  accelZ = nano33IMU.readFloatAccelZ();

  gyroX = nano33IMU.readFloatGyroX();
  gyroY = nano33IMU.readFloatGyroY();
  gyroZ = nano33IMU.readFloatGyroZ();

  // Convert these values into angles using the Complementary Filter
  CompFilter.CompAccelUpdate(accelX, accelY, accelZ); // takes arguments in m/s^2
  CompFilter.CompGyroUpdate(gyroX, gyroY, gyroZ); // takes arguments un rad/s 
  CompFilter.CompUpdate();
  CompFilter.CompStart();

  // Get angle relative to X and Y axes and write them to the variables in the arguments
  //in radians
  CompFilter.CompAnglesGet(&xAngle, &yAngle);

  //convert from radians to angles
  pitch = xAngle*RAD_TO_DEG;
  roll = yAngle*RAD_TO_DEG;
  
  Serial.print(pitch);
  Serial.print('\t');
  Serial.println(roll);

 delay(200);
}
