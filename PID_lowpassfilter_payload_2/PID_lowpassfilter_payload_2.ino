//As of 1:42pm Jan30//Has PID, servo, filter- only uses magnetometer from IMU
#include <SPI.h> 
#include <Wire.h>
#include <PID_v1.h>
#include <SFE_LSM9DS0.h>
#include <Servo.h>

#define LSM9DS0_XM 0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G 0x6B // Would be 0x6A if SDO_G is LOW
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
#define PRINT_CALCULATED
float A = 1;
float total = 0;
const int index = 30;
float average = 0;
float roll_avg[index];
float x, y, z, roll_mag, smoothData1;

const byte INT1XM = 9; // INT1XM tells us when accel data is ready
const byte INT2XM = 8; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready
boolean printRaw = true;

double Setpoint, Input, Output;
double Kp=0.01, Ki=0.00, Kd=0.10; /////////////////////////////////////////////////////////////////////////
pidMin = -5.0;
pidMax = pidMin*(-1);
  
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

Servo myservo;

boolean useAccel = true;
boolean testMode = false;

float imuReading;
double pidMin, pidMax;

void setup()
{
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);
  Serial.begin(115200); // Start serial at 115200 bps
  uint16_t status = dof.begin();
  Serial.print("LSM9DS0 WHO_AM_I's returned: 0x");
  Serial.println(status, HEX);
  Serial.println("Should be 0x49D4");
  Serial.println();

  Setpoint = 90;////////////////////////////////////////////////////////////////////////////////////
  
  myPID.SetMode(AUTOMATIC); //turn PID on
  myservo.attach(5);
  
  //Serial.println(pidMin);
  //Serial.println(pidMax);
}

void loop()
{
//Read values  
  dof.readGyro();
  dof.readAccel();
  dof.readMag();
  x = dof.calcMag(dof.mx);
  y = dof.calcMag(dof.my);
  z = dof.calcMag(dof.mz);
//Obtain roll_mag (Roll as determined only from magnetometer)
  roll_mag = atan2(y, z);
  roll_mag *= 180.0 / PI;
  if (roll_mag < 0) roll_mag = roll_mag + 360;
  //Filter roll_mag
  roll_avg[index] = roll_mag;
  smoothData1 = digitalSmooth(roll_mag, roll_avg);
//Send data so it can be graphed
  Serial.write(smoothData1);

//PID stuff
  Input = roll_mag;///////////////////////////This replaces the above two lines
  myPID.SetTunings(Kp, Ki, Kd);
  myPID.Compute();
  Serial.print(" Output (from PID): ");
  Serial.print(Output);
  //if (Output > pidMax) pidMax = Output;
  //if (Output < pidMin) pidMin = Output;
  double servoOutput = ((Output - (pidMin)) * (179) / (pidMax - (pidMin)));
  servoOutput = constrain(servoOutput, 29, 149);
//        servoOutput = map(servoOutput, 0, 179, 179, 0);
 
  myservo.write(servoOutput);
  Serial.print("; Servo Output: ");
  Serial.println(servoWrite);
  delay(5);
}

float digitalSmooth(float rawIn, float *sensSmoothArray){
  int j, k, temp, top, bottom;
  long total;
  static int i;
  static int sorted[index];
  boolean done;
  
  i = (i + 1) % index;
  sensSmoothArray[i] = rawIn;
  
  for (j = 0; j<index; j++){
    sorted[j] = sensSmoothArray[j];
  }
  done = 0;
  while(done != 1){
    done = 1;
    for (j = 0; j < (index - 1); j++){
      if (sorted[j] > sorted[j + 1]){
        temp = sorted[j +1];
        sorted [j+1] = sorted[j];
        sorted [j] = temp;
        done = 0;
      }
    }
  }
  //throw out top and bottom 15% of samples
  bottom = max(((index*20)/100), 1);
  top = min ((((index*80)/100) + 1), (index - 1));
  k = 0;
  total = 0;
  for (j = bottom; j < top; j++){
    total += sorted[j];
    k++;
  }
  return total/k;
}
