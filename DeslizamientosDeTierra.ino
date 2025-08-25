/*
  ESP32 – Alerta de deslizamientos (IMU calibrada, lluvia intensidad/binario,
  vibración, humedad, LEDs, y BUZZER SOLO en ROJO con control activo-bajo/alto)
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//===================== CONFIGURACIÓN =====================//
#define RAIN_MODE_BINARY 0             // 0=intensidad (0..100%), 1=binario
enum SoilType { CAPACITIVE, RESISTIVE };
const SoilType SOIL_SENSOR_TYPE = CAPACITIVE;

//---- Pines (ESP32 DevKit V1)
const int PIN_RAIN_AO   = 34;   // ADC1_CH6
const int PIN_SOIL_AO   = 35;   // ADC1_CH7
const int PIN_VIB_DO    = 4;    // Vibración (digital)
const int PIN_LED_RED   = 25;
const int PIN_LED_YELLOW= 26;
const int PIN_LED_GREEN = 27;
const int PIN_BUZZER    = 14;   // Buzzer

//---- Buzzer: si tu buzzer/módulo/transistor es activo-bajo, deja 1
#define BUZZER_ACTIVE_LOW 1

// Utilidades buzzer
inline void BUZZER_WRITE(bool on){
  // si es activo-bajo: ON -> LOW, OFF -> HIGH
  digitalWrite(PIN_BUZZER, BUZZER_ACTIVE_LOW ? !on : on);
}
inline void BUZZER_OFF(){ BUZZER_WRITE(false); }
inline void BUZZER_ON(){  BUZZER_WRITE(true);  }

//---- ADC / Atenuación
#include "driver/adc.h"
#include "esp_adc_cal.h"

//---- MPU6050
Adafruit_MPU6050 mpu;

//---- Promedios simples
template <size_t N>
struct MovingAvg {
  int buf[N]; size_t idx=0; bool filled=false; long sum=0;
  void add(int v){ sum -= filled ? buf[idx] : 0; buf[idx]=v; sum += v; idx=(idx+1)%N; if(idx==0) filled=true; }
  float avg() const { size_t n = filled?N:idx; return n? (float)sum/n : 0.0f; }
};
MovingAvg<10> rainAvg;
MovingAvg<10> soilAvg;

//---- Calibración lluvia (intensidad)
int RAIN_ADC_DRY = 4000;     // seco ≈ alto
int RAIN_ADC_WET = 1200;     // muy mojado ≈ bajo
const uint32_t RAIN_HOLD_MS = 4000;

//---- Lluvia binario
const int   RAIN_WET_ADC_THRESH  = 1600;

//---- Humedad de suelo
const int SOIL_WET_ADC_CAP  = 1800;  // capacitivo: <1800 -> muy húmedo
const int SOIL_WET_ADC_RES  = 2800;  // resistivo:  >2800 -> muy húmedo
const uint32_t SOIL_HOLD_MS = 5000;

//---- Vibración
volatile uint32_t vibPulses = 0;
void IRAM_ATTR vibISR(){ vibPulses++; }
const uint32_t VIB_WINDOW_MS   = 2000;
const uint32_t VIB_PULSES_RISK = 3;
const uint32_t VIB_HOLD_MS     = 3000;

//==== Inclinación / aceleración (con baseline y filtros) ====//
float ROLL_OFFS = 0.0f, PITCH_OFFS = 0.0f;
const float TILT_ABS_DEG_THRESH = 12.0f;
const float TILT_RATE_DEG_PER_H  = 0.20f;
const float SHOCK_G_THRESH       = 0.80f;
const uint32_t IMU_WARMUP_MS     = 800;
const uint32_t IMU_BASELINE_MS   = 2000;
const uint32_t RATE_MIN_DT_MS    = 1000;

//===================== ESTADO =====================//
unsigned long lastReport = 0;
unsigned long lastTiltRateTick = 0;
float lastRollRel=0, lastPitchRel=0;
bool haveLastAngles = false;

unsigned long rainWetSince = 0;
unsigned long soilWetSince = 0;
unsigned long vibActiveSince = 0;
uint32_t lastVibPulseSnapshot = 0;
unsigned long lastVibSnapshotMs = 0;

// Buzzer (solo ROJO)
bool buzzerOn = false;
unsigned long lastBuzzToggle = 0;
const uint32_t BUZZ_PERIOD_MS = 300;

//===================== UTILIDADES =====================//
float clampf(float v, float lo, float hi){ return v<lo?lo:(v>hi?hi:v); }
void computeAngles(const sensors_event_t& a, float& rollDeg, float& pitchDeg){
  float ax=a.acceleration.x, ay=a.acceleration.y, az=a.acceleration.z;
  rollDeg  = atan2f(ay, az) * 180.0f / PI;
  pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;
}
float mapWetPercentFromADC(int adc, int dryADC, int wetADC){
  if(wetADC >= dryADC) wetADC = dryADC - 1;
  adc = constrain(adc, wetADC, dryADC);
  float pct = (float)(dryADC - adc) / (float)(dryADC - wetADC) * 100.0f;
  return clampf(pct, 0, 100);
}
void calibrateIMU() {
  Serial.println("Calibrando IMU (quieto)...");
  unsigned long t0 = millis();
  while(millis() - t0 < IMU_WARMUP_MS) { delay(5); }
  const int N = 200;
  double sumR=0, sumP=0;
  for(int i=0;i<N;i++){
    sensors_event_t a, g, temp; mpu.getEvent(&a,&g,&temp);
    float r,p; computeAngles(a,r,p); sumR += r; sumP += p;
    delay(IMU_BASELINE_MS / N);
  }
  ROLL_OFFS  = (float)(sumR / N);
  PITCH_OFFS = (float)(sumP / N);
  haveLastAngles = false;
  Serial.printf("Baseline -> roll0=%.2f°, pitch0=%.2f°\n", ROLL_OFFS, PITCH_OFFS);
}

//===================== SETUP =====================//
void setup() {
  Serial.begin(115200); delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_RAIN_AO, ADC_11db);
  analogSetPinAttenuation(PIN_SOIL_AO, ADC_11db);

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  BUZZER_OFF(); // MUY IMPORTANTE: apaga según polaridad

  pinMode(PIN_VIB_DO, INPUT_PULLDOWN);
  attachInterrupt(digitalPinToInterrupt(PIN_VIB_DO), vibISR, RISING);
  lastVibSnapshotMs = millis();

  Wire.begin(21,22);
  if(!mpu.begin()){
    Serial.println("ERROR: MPU6050 no detectado.");
    while(1){ digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED)); delay(200); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_10_HZ);

  calibrateIMU();

  Serial.println("\nEscribe 'c' + Enter para recalibrar IMU cuando esté quieto.");
  Serial.println("Nota: 4095 en seco (lluvia/suelo) es normal con estos módulos.");
}

//===================== LOOP =====================//
void loop() {
  // Recalibración por Serial
  if(Serial.available()){
    char ch = Serial.read();
    if(ch=='c' || ch=='C'){ calibrateIMU(); }
  }

  // 1) Lecturas
  int rainRaw = analogRead(PIN_RAIN_AO);
  int soilRaw = analogRead(PIN_SOIL_AO);
  rainAvg.add(rainRaw);
  soilAvg.add(soilRaw);

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float rollAbs, pitchAbs; computeAngles(a, rollAbs, pitchAbs);
  float rollRel  = rollAbs  - ROLL_OFFS;
  float pitchRel = pitchAbs - PITCH_OFFS;

  // Tasa de inclinación (deg/h)
  unsigned long tnow = millis();
  static float rateLP = 0.0f;
  float tiltRateDegPerHour = 0.0f;
  if(haveLastAngles && (tnow - lastTiltRateTick) >= RATE_MIN_DT_MS){
    float dRoll  = fabsf(rollRel  - lastRollRel);
    float dPitch = fabsf(pitchRel - lastPitchRel);
    float dAng   = max(dRoll, dPitch);
    float dt_s   = (tnow - lastTiltRateTick)/1000.0f;
    float rate_deg_per_s = dAng / dt_s;
    rateLP = 0.90f*rateLP + 0.10f*rate_deg_per_s;
    tiltRateDegPerHour = rateLP * 3600.0f;
    lastRollRel=rollRel; lastPitchRel=pitchRel; lastTiltRateTick=tnow;
  }
  if(!haveLastAngles){ lastRollRel=rollRel; lastPitchRel=pitchRel; lastTiltRateTick=tnow; haveLastAngles=true; }

  // Shock
  float amag = sqrtf(a.acceleration.x*a.acceleration.x + a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z);
  float gDev = fabsf(amag - 9.80665f);
  bool shock = (gDev > (SHOCK_G_THRESH * 9.80665f));

  // 2) Lluvia
  bool rainRisk = false; float rainPct = 0.0f;
  if(RAIN_MODE_BINARY){
    bool rainWetNow = (rainAvg.avg() < RAIN_WET_ADC_THRESH);
    static unsigned long rainSince=0;
    if(rainWetNow){ if(rainSince==0) rainSince=tnow; } else { rainSince=0; }
    rainRisk = (rainSince>0 && tnow - rainSince >= RAIN_HOLD_MS);
    rainPct = rainWetNow ? 100.0f : 0.0f;
  } else {
    rainPct = mapWetPercentFromADC((int)rainAvg.avg(), RAIN_ADC_DRY, RAIN_ADC_WET);
    const float RAIN_RISK_PERCENT = 70.0f;
    static unsigned long rainHighSince=0;
    bool highNow = (rainPct >= RAIN_RISK_PERCENT);
    if(highNow){ if(rainHighSince==0) rainHighSince=tnow; } else { rainHighSince=0; }
    rainRisk = (rainHighSince>0 && tnow - rainHighSince >= RAIN_HOLD_MS);
  }

  // 3) Suelo
  bool soilWetNow = (SOIL_SENSOR_TYPE==CAPACITIVE) ? (soilAvg.avg() < SOIL_WET_ADC_CAP)
                                                   : (soilAvg.avg() > SOIL_WET_ADC_RES);
  if(soilWetNow){ if(soilWetSince==0) soilWetSince=tnow; } else { soilWetSince=0; }
  bool soilRisk = (soilWetSince>0) && (tnow - soilWetSince >= SOIL_HOLD_MS);

  // 4) Vibración
  if(tnow - lastVibSnapshotMs >= VIB_WINDOW_MS){
    uint32_t pulses = vibPulses - lastVibPulseSnapshot;
    lastVibPulseSnapshot = vibPulses;
    lastVibSnapshotMs = tnow;
    if(pulses >= VIB_PULSES_RISK){ vibActiveSince = tnow; }
  }
  bool vibRisk = (vibActiveSince>0) && (tnow - vibActiveSince <= VIB_HOLD_MS);

  // 5) Inclinación
  bool tiltRisk = (fabsf(rollRel)  >= TILT_ABS_DEG_THRESH) ||
                  (fabsf(pitchRel) >= TILT_ABS_DEG_THRESH) ||
                  (tiltRateDegPerHour >= TILT_RATE_DEG_PER_H) ||
                  shock;

  // 6) Fusión
  uint8_t riskCount = (rainRisk?1:0) + (soilRisk?1:0) + (vibRisk?1:0) + (tiltRisk?1:0);
  bool red    = (riskCount >= 3);
  bool yellow = (!red && riskCount == 2);
  bool green  = (!red && !yellow);

  digitalWrite(PIN_LED_RED,    red);
  digitalWrite(PIN_LED_YELLOW, yellow);
  digitalWrite(PIN_LED_GREEN,  green);

  // 7) BUZZER SOLO EN ROJO (con polaridad correcta)
  if(red){
    if(millis() - lastBuzzToggle >= BUZZ_PERIOD_MS){
      lastBuzzToggle = millis();
      buzzerOn = !buzzerOn;
      if(buzzerOn) BUZZER_ON(); else BUZZER_OFF();
    }
  } else {
    buzzerOn = false;
    BUZZER_OFF();
  }

  // 8) Telemetría
  if(millis() - lastReport > 700){
    lastReport = millis();
    Serial.println("------");
    Serial.printf("Rain raw(avg): %d (%.0f)\n", rainRaw, rainAvg.avg());
    if(RAIN_MODE_BINARY){
      Serial.printf("Rain bin: %s (th=%d, hold=%lus)\n",
                    (rainRisk?"RIESGO lluvia":"no"), RAIN_WET_ADC_THRESH, RAIN_HOLD_MS/1000);
    } else {
      Serial.printf("Rain intensity: %.1f %% (dry=%d, wet=%d)\n",
                    rainPct, RAIN_ADC_DRY, RAIN_ADC_WET);
    }
    Serial.printf("Soil raw(avg): %d (%.0f) | Risk: %s\n",
                  soilRaw, soilAvg.avg(), soilRisk?"YES":"no");
    Serial.printf("Tilt REL roll/pitch: %.2f / %.2f deg | rate: %.3f deg/h | shock:%s\n",
                  rollRel, pitchRel, tiltRateDegPerHour, shock?"YES":"no");
    Serial.printf("Vib pulses total: %lu | window risk:%s\n",
                  (unsigned long)vibPulses, vibRisk?"ACTIVE":"idle");
    Serial.printf("Risks -> rain:%d soil:%d vib:%d tilt:%d | COUNT=%u\n",
                  rainRisk, soilRisk, vibRisk, tiltRisk, riskCount);
    Serial.printf("ALERTA: %s %s\n",
                  red?"ROJA": (yellow?"AMARILLA":"VERDE"),
                  red? "(BUZZER)": "");
  }

  delay(10);
}
