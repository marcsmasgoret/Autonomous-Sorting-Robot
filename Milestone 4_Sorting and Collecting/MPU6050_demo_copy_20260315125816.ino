#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 imu;

// FILTERS
struct AxisFilter {
  float x1 = 0, x2 = 0;
  float y  = 0;
};

static inline float median3(float a, float b, float c){
  if(a>b){ float t=a;a=b;b=t; }
  if(b>c){ float t=b;b=c;c=t; }
  if(a>b){ float t=a;a=b;b=t; }
  return b;
}

static inline float deadzone(float x, float d){ return (fabs(x) < d) ? 0.0f : x; }

static inline float ema(float x, float &y, float alpha){
  y = (1.0f - alpha)*y + alpha*x;
  return y;
}

static inline float filterAxis(AxisFilter &f, float raw, float dz, float alpha){
  float m = median3(raw, f.x1, f.x2);
  f.x2 = f.x1; f.x1 = raw;
  m = deadzone(m, dz);
  return ema(m, f.y, alpha);
}

// CALIBRATION
float gbz = 0; // gyro Z bias only

void calibrate(uint16_t N = 1500, uint16_t dtMs = 3){
  for(int i = 0; i < 100; i++){
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    delay(5);
  }

  double sGZ = 0;
  for(uint16_t i = 0; i < N; i++){
    sensors_event_t a, g, t;
    imu.getEvent(&a, &g, &t);
    sGZ += g.gyro.z;
    delay(dtMs);
  }

  gbz = sGZ / N;
}

// STATE

float yaw = 0;
unsigned long lastUs = 0;

AxisFilter gzF;

// Tuning
const float GYR_DZ    = 0.01f;  // rad/s  — raise if yaw drifts while still
const float GYR_ALPHA = 0.15f;  // EMA smoothing

void wrapPi(float &a){
  if(a >  3.14159265f) a -= 6.28318530f;
  if(a < -3.14159265f) a += 6.28318530f;
}

void setup(){
  Serial.begin(115200);
  while(!Serial) delay(10);

  if(!imu.begin()) while(1) delay(10);

  imu.setAccelerometerRange(MPU6050_RANGE_8_G);
  imu.setGyroRange(MPU6050_RANGE_500_DEG);
  imu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(200);

  calibrate();
  lastUs = micros();

  Serial.println("Yaw_deg YawRate_dps");
}

void loop(){
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if(dt <= 0 || dt > 0.1f) dt = 0.01f;

  sensors_event_t a, g, t;
  imu.getEvent(&a, &g, &t);

  float gz = g.gyro.z - gbz;
  float fgz = filterAxis(gzF, gz, GYR_DZ, GYR_ALPHA);

  yaw += fgz * dt;
  wrapPi(yaw);

  const float RAD2DEG = 57.2957795f;
  Serial.print(yaw * RAD2DEG);   Serial.print(" ");
  Serial.println(fgz * RAD2DEG);

  delay(10);
}