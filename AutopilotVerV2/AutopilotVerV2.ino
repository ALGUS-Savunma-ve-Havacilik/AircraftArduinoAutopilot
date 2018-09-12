#include "MPU9250.h"
#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Servo.h>
Servo YawServo;
Servo PitchServo;
Servo RollServo;
Servo ThrustServo;
static const int RXPin = 12, TXPin = 13;
static const uint32_t GpsBaud = 9600;

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int MPUstatus;

SoftwareSerial ss(RXPin, TXPin);
TinyGPSPlus gps;

SFE_BMP180 pressure;
#define ALTITUDE 100.0
bool blinkState = false;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}




void setup() {
  Wire.begin();
  Serial.begin(9600);
  // start communication with IMU
  MPUstatus = IMU.begin();
  if (MPUstatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(MPUstatus);
    while (1) {}
  }
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);

  IMU.calibrateGyro();
  IMU.calibrateAccel();

  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 init fail\n\n");
    while (1); // Pause forever.
  }

  //GPS
  //ss.begin(GpsBaud);
  // configure LED for output
  RollServo.attach(8);
  PitchServo.attach(9);
  ThrustServo.attach(10);
  YawServo.attach(11);
}

double LastTime = millis();
double dt;
double SetYaw = 0;
double SetPitch = 0;
double SetRoll = 0;
double Yaw_output = 0;
double Pitch_output = 0;
double Roll_output = 0;
double Pitch_Written_Output;
double Roll_Written_Output;

//Yaw
//double Yaw_Error;
//double Yaw_Previous_Error = 0;
//double Yaw_Integral = 0;
//double Yaw_KP = 0.001; //Change!
//double Yaw_KI = 0.001;//Change!
//double Yaw_KD = 0.001;//Change!

//Pitch
double Pitch_Error;
double Pitch_Previous_Error = 0;
double Pitch_Integral = 0;
double Pitch_KP = 1; //Change!
double Pitch_KI = 0;//Change!
double Pitch_KD = 0;//Change!

//Roll
double Roll_Error;
double Roll_Previous_Error = 0;
double Roll_Integral = 0;
double Roll_KP = 1; //Change!
double Roll_KI = 0; //Change!
double Roll_KD = 0; //Change!

int k = 0;

double GyroX;
double GyroY;
double GyroZ;

double KalYaw;
double KalPitch;
double KalRoll;

double KalmanAngleY;
double KalmanAngleX;

double AccelX;
double AccelY;
double AccelZ;

double GyroRateX;
double GyroRateY;

float Q_angle = 0.001; //0.001
float Q_gyro = 0.003;  //0.003
float R_angle = 0.03;  //0.03

float angle = 0;
float bias = 0;
float K_0, K_1, y, S;
float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;

double hold = 0;
double delta = 0;

int LastServoWrite = 0;

void loop() {
  IMU.readSensor();

  AccelX = IMU.getAccelX_mss();
  AccelY = IMU.getAccelY_mss();
  AccelZ = IMU.getAccelZ_mss();
  GyroRateX = IMU.getGyroX_rads() * RAD_TO_DEG;
  GyroRateY = IMU.getGyroY_rads() * RAD_TO_DEG;

  KalRoll = atan(AccelY / sqrt(AccelX * AccelX + AccelZ * AccelZ)) * RAD_TO_DEG;
  KalPitch = atan2(-AccelX, AccelZ) * RAD_TO_DEG;

  dt = millis() - LastTime;

  GyroX = IMU.getGyroX_rads() * dt * RAD_TO_DEG;
  GyroY = IMU.getGyroY_rads() * dt * RAD_TO_DEG;
  GyroZ = IMU.getGyroZ_rads() * dt * RAD_TO_DEG;

  if ((KalPitch > 90 && KalmanAngleY < -90) || (KalPitch < -90 && KalmanAngleY))
  {
    angle = KalPitch;
    KalmanAngleY = KalPitch;
  }
  else
  {
    KalmanAngleY = KalmanFilter(KalPitch, GyroRateY, dt);
  }
  if (abs(KalmanAngleY) > 90)
  {
    GyroRateX = -GyroRateX;
  }
  KalmanAngleX = KalmanFilter(KalRoll, GyroRateX, dt);

  Pitch_Error = SetPitch - KalmanAngleY;
  Roll_Error = SetRoll - KalmanAngleX;

  Pitch_Integral = Pitch_Integral + Pitch_Error * dt;
  Roll_Integral = Roll_Integral + Roll_Error * dt;

  Pitch_output = Pitch_KP * Pitch_Error + Pitch_KI * Pitch_Integral + Pitch_KD * (Pitch_Error - Pitch_Previous_Error) / dt;
  Roll_output = Roll_KP * Roll_Error + Roll_KI * Roll_Integral + Roll_KD * (Roll_Error - Roll_Previous_Error) / dt;

  Pitch_Written_Output = Pitch_output + 90;
  Roll_Written_Output = Roll_output + 90;

  if (Pitch_output > 89)
  {
    Pitch_Written_Output = 179;
  }
  else if (Pitch_output < -89)
  {
    Pitch_Written_Output = 1;
  }
  if (Roll_output > 89)
  {
    Roll_Written_Output = 179;
  }
  else if (Roll_output < -89)
  {
    Roll_Written_Output = 1;
  }
  
  LastServoWrite = RollServo.read();
  
  RollServo.write(Roll_Written_Output);
  PitchServo.write(Pitch_Written_Output);
  
  LastTime = millis();

  Pitch_Previous_Error = Pitch_Error;
  Roll_Previous_Error = Roll_Error;


    
//  Serial.print(Pitch_output);
//  Serial.print("\t");
//    Serial.print(Roll_output);
    Serial.print("\t\t");
    Serial.print(Pitch_Written_Output );
    Serial.print("\t\t");
    Serial.print(Roll_Written_Output);
    Serial.print("\t\t");
//  Serial.print(KalmanAngleY);
//  Serial.print("\t");
//    Serial.print(KalmanAngleX);
//    Serial.print("\t\t");
//    Serial.print(LastServoWrite);
    Serial.print("\n");


  //	Barometer();
  //  if (k == 10){
  //    GPS();
  //    k = 0;
  //  }
  //  k = k+1;
}


void Barometer()
{
  //============================================================
  //   TEMP PRESSURE ALT
  char status;
  double T, P, p0, a;

  // Loop here getting pressure readings every 10 seconds.

  // If you want sea-level-compensated pressure, as used in weather reports,
  // you will need to know the altitude at which your measurements are taken.
  // We're using a constant called ALTITUDE in this sketch:

  //  Serial.println();
  //  Serial.print("provided altitude: ");
  //  Serial.print(ALTITUDE, 0);
  //  Serial.print(" meters, ");

  // If you want to measure altitude, and not pressure, you will instead need
  // to provide a known baseline pressure. This is shown at the end of the sketch.

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      //      Serial.println();
      //      Serial.print("temperature: ");
      //      Serial.print(T, 2);
      //      Serial.print(" deg C, ");

      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P, T);
        if (status != 0)
        {
          // Print out the measurement:
          //          Serial.println();
          //          Serial.print("absolute pressure: ");
          //          Serial.print(P*1000, 2);
          //          Serial.print(" Pa, ");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P, ALTITUDE); // we're at 1655 meters (Boulder, CO)
          //          Serial.println();
          //          Serial.print("relative (sea-level) pressure: ");
          //          Serial.print(p0*1000, 2);
          //          Serial.print(" Pa, ");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.
          p0 = 1021.80;
          a = pressure.altitude(P, p0);
          //          Serial.println();
          //          Serial.print("computed altitude: ");
          //          Serial.print(a, 0);
          //          Serial.print(" meters, ");

        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

}
/*
  void GPS()
  {
  //GPS
  gps.encode(ss.read());
    //if (gps.location.isUpdated()){
      // Latitude in degrees (double)
		Serial.println();
		Serial.print("Latitude= ");
		Serial.print(gps.location.lat(), 6);
		// Longitude in degrees (double)
		Serial.print(" Longitude= ");
		Serial.println(gps.location.lng(), 6);

		// Raw latitude in whole degrees
		Serial.print("Raw latitude = ");
		Serial.print(gps.location.rawLat().negative ? "-" : "+");
		Serial.println(gps.location.rawLat().deg);
		// ... and billionths (u16/u32)
		Serial.println(gps.location.rawLat().billionths);

		// Raw longitude in whole degrees
		Serial.print("Raw longitude = ");
		Serial.print(gps.location.rawLng().negative ? "-" : "+");
		Serial.println(gps.location.rawLng().deg);
		// ... and billionths (u16/u32)
		Serial.println(gps.location.rawLng().billionths);

		// Raw date in DDMMYY format (u32)
		Serial.print("Raw date DDMMYY = ");
		Serial.println(gps.date.value());

		// Raw time in HHMMSSCC format (u32)
		Serial.print("Raw time in HHMMSSCC = ");
		Serial.println(gps.time.value());

		// Speed in meters per second (double)
		Serial.print("Speed in m/s = ");
		Serial.println(gps.speed.mps());

		// Raw course in 100ths of a degree (i32)
		Serial.print("Raw course in degrees = ");
		Serial.println(gps.course.value());
		// Course in degrees (double)
		Serial.print("Course in degrees = ");
		Serial.println(gps.course.deg());

		// Altitude in meters (double)
		Serial.print("Altitude in meters = ");
		Serial.println(gps.altitude.meters());

		// Number of satellites in use (u32)
		Serial.print("Number os satellites in use = ");
		Serial.println(gps.satellites.value());

		// Horizontal Dim. of Precision (100ths-i32)
		Serial.print("HDOP = ");
		Serial.println(gps.hdop.value());
    //}
  //delay(5000);
  }
*/

void GPS()
{
  Serial.println();
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT,
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT,
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();

  //	smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }

  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
  smartDelay(0);
}

float KalmanFilter(double NewAngle, double NewRate, double dt) {

  angle += dt + (NewRate - bias);
  P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
  P_01 += -dt * P_11;
  P_10 += -dt * P_11;
  P_11 += +Q_gyro * dt;

  y = NewAngle - angle;
  S = P_00 + R_angle;
  K_0 = P_00 / S;
  K_1 = P_10 / S;

  angle += K_0 * y;
  bias += K_1 * y;
  P_00 -= K_0 * P_00;
  P_01 -= K_0 * P_01;
  P_10 -= K_1 * P_00;
  P_11 -= K_1 * P_01;

  return angle;
}
//
//void Takeoff()
//{
//	runway_start = [gps.location.lat(), gps.location.lon()]
//	runway_end = []// POPULATE
//	startalt = gps.altitude.meters()
//	Takeoff == true;
//	unitvector0 = runway_end/((runway_start(0)^2+runway_start(1)^2)^0.5);
//	T = 1990; // SET THRUST TO MAX
//	while (Takeoff == true)
//		unitvector1 = ;
//		angle = Angled(unitvector0,unitvector1);
//		unitvector0 = unitvector1;
//}
//
//void Angled(unitvector0, unitvector1)
//{
//	angle = atan(unitvector0(0)*unitvector1(1) - unitvector1(0)*unitvector0(1),unitvector0(0)*unitvector0(1) + unitvector1(0)*unitvector1(1));
//	return angle;
//}
