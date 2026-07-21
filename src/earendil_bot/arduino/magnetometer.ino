#include <Wire.h>
#include <math.h>

#if defined(__AVR__)
#include <avr/wdt.h>
#endif

#define QMC5883L_ADDR 0x0D


// ============================================================
// 3D KALIBRASYON DEGERLERI
// ============================================================

const float MAG_X_OFFSET = 0.000000f;
const float MAG_Y_OFFSET = -270.000000f;
const float MAG_Z_OFFSET = 16.000000f;

const float MAG_X_SCALE = 0.986372f;
const float MAG_Y_SCALE = 0.988798f;
const float MAG_Z_SCALE = 1.025794f;


// ============================================================
// SENSOR MONTAJ YONU
// ============================================================

const int8_t X_SIGN = 1;
const int8_t Y_SIGN = -1;

const float HEADING_FINE_OFFSET_DEG = 0.0f;


// ============================================================
// BIRINCI ASAMA HEADING REFERANSLARI
// ============================================================

const float MEASURED_AT_0   = 128.0f;
const float MEASURED_AT_90  = 210.0f;
const float MEASURED_AT_180 = 306.0f;
const float MEASURED_AT_270 = 42.0f;


// ============================================================
// SON HEADING CIKTISI ICIN 5 NOKTALI DUZELTME
// ============================================================
//
// Gercek   0 derece -> Mevcut cikti 359 derece
// Gercek  90 derece -> Mevcut cikti  85 derece
// Gercek 180 derece -> Mevcut cikti 181 derece
// Gercek 270 derece -> Mevcut cikti 257 derece
// Gercek 330 derece -> Mevcut cikti 313 derece
//

const float FINAL_MEASURED_AT_0   = 359.0f;
const float FINAL_MEASURED_AT_90  = 85.0f;
const float FINAL_MEASURED_AT_180 = 181.0f;
const float FINAL_MEASURED_AT_270 = 257.0f;
const float FINAL_MEASURED_AT_330 = 313.0f;


// ============================================================
// SISTEM AYARLARI
// ============================================================

// Yaklasik 8.3 Hz
const unsigned long READ_INTERVAL_MS = 120UL;

// I2C maksimum bekleme suresi
const unsigned long I2C_TIMEOUT_US = 25000UL;

// Dusuk geciren filtre
const float FILTER_ALPHA = 0.20f;

// Ardisik hata esikleri
const uint8_t RECONFIGURE_ERROR_COUNT = 8;
const uint8_t FULL_RESTART_ERROR_COUNT = 30;


// ============================================================
// CALISMA DEGISKENLERI
// ============================================================

unsigned long lastReadMillis = 0;

unsigned long validReadCount = 0;
unsigned long failedReadCount = 0;

uint8_t consecutiveErrors = 0;

float filteredX = 0.0f;
float filteredY = 0.0f;
float filteredZ = 0.0f;

bool filterInitialized = false;


// ============================================================
// WATCHDOG
// ============================================================

void feedWatchdog()
{
#if defined(__AVR__)
  wdt_reset();
#endif
}


// ============================================================
// WIRE BASLATMA
// ============================================================

void startWire()
{
  Wire.begin();

  // Parazitli veya uzun kablolarda daha kararlidir.
  Wire.setClock(100000UL);

  // Timeout durumunda AVR I2C donanimini sifirla.
  Wire.setWireTimeout(I2C_TIMEOUT_US, true);
  Wire.clearWireTimeoutFlag();
}


// ============================================================
// WIRE TIMEOUT KONTROLU
// ============================================================

bool wireTimedOut()
{
  if (Wire.getWireTimeoutFlag())
  {
    Wire.clearWireTimeoutFlag();
    return true;
  }

  return false;
}


// ============================================================
// WIRE TAMPONUNU TEMIZLE
// ============================================================

void clearWireBuffer()
{
  while (Wire.available() > 0)
  {
    Wire.read();
  }
}


// ============================================================
// REGISTER YAZMA
// ============================================================

bool writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(QMC5883L_ADDR);

  Wire.write(reg);
  Wire.write(value);

  uint8_t result = Wire.endTransmission(true);

  if (wireTimedOut())
  {
    return false;
  }

  return result == 0;
}


// ============================================================
// SENSOR VAR MI?
// ============================================================

bool qmcExists()
{
  Wire.beginTransmission(QMC5883L_ADDR);

  uint8_t result = Wire.endTransmission(true);

  if (wireTimedOut())
  {
    return false;
  }

  return result == 0;
}


// ============================================================
// QMC5883L BASLATMA
// ============================================================

bool initializeQMC5883L()
{
  if (!qmcExists())
  {
    return false;
  }

  // Software reset
  if (!writeRegister(0x0A, 0x80))
  {
    return false;
  }

  delay(20);

  // Set/reset period
  if (!writeRegister(0x0B, 0x01))
  {
    return false;
  }

  /*
     Control Register 1 = 0x11

     OSR   = 512
     Range = +/-8 Gauss
     ODR   = 10 Hz
     Mode  = Continuous
  */
  if (!writeRegister(0x09, 0x11))
  {
    return false;
  }

  delay(150);

  return true;
}


// ============================================================
// VERI GECERLILIK KONTROLU
// ============================================================

bool sampleLooksValid(int16_t x, int16_t y, int16_t z)
{
  if (x == -1 && y == -1 && z == -1)
  {
    return false;
  }

  if ((x == -1 && y == -1) ||
      (x == -1 && z == -1) ||
      (y == -1 && z == -1))
  {
    return false;
  }

  if (x == 0 && y == 0 && z == 0)
  {
    return false;
  }

  if (x == 32767 || x == -32768 ||
      y == 32767 || y == -32768 ||
      z == 32767 || z == -32768)
  {
    return false;
  }

  return true;
}


// ============================================================
// TEK QMC5883L OKUMASI
// ============================================================

bool readQMC5883LOnce(
  int16_t &x,
  int16_t &y,
  int16_t &z)
{
  clearWireBuffer();

  Wire.beginTransmission(QMC5883L_ADDR);
  Wire.write(0x00);

  uint8_t result = Wire.endTransmission(true);

  if (wireTimedOut() || result != 0)
  {
    return false;
  }

  delayMicroseconds(100);

  uint8_t received = Wire.requestFrom(
    (uint8_t)QMC5883L_ADDR,
    (uint8_t)6,
    (uint8_t)true
  );

  if (wireTimedOut())
  {
    clearWireBuffer();
    return false;
  }

  if (received != 6)
  {
    clearWireBuffer();
    return false;
  }

  uint8_t data[6];

  for (uint8_t i = 0; i < 6; i++)
  {
    if (Wire.available() <= 0)
    {
      clearWireBuffer();
      return false;
    }

    data[i] = Wire.read();
  }

  x = (int16_t)(
        ((uint16_t)data[1] << 8) |
        data[0]
      );

  y = (int16_t)(
        ((uint16_t)data[3] << 8) |
        data[2]
      );

  z = (int16_t)(
        ((uint16_t)data[5] << 8) |
        data[4]
      );

  return sampleLooksValid(x, y, z);
}


// ============================================================
// TEKRAR DENEMELI OKUMA
// ============================================================

bool readQMC5883L(
  int16_t &x,
  int16_t &y,
  int16_t &z)
{
  for (uint8_t attempt = 0; attempt < 3; attempt++)
  {
    feedWatchdog();

    if (readQMC5883LOnce(x, y, z))
    {
      return true;
    }

    delay(3);
  }

  return false;
}


// ============================================================
// WIRE VE SENSORU YENIDEN BASLATMA
// ============================================================

bool restartWireAndSensor()
{
  Serial.println(
    F("# Wire ve QMC yeniden baslatiliyor.")
  );

  Wire.end();
  delay(50);

  startWire();
  delay(50);

  for (uint8_t attempt = 0; attempt < 5; attempt++)
  {
    feedWatchdog();

    if (initializeQMC5883L())
    {
      Serial.println(
        F("# QMC5883L yeniden baslatildi.")
      );

      consecutiveErrors = 0;
      filterInitialized = false;

      return true;
    }

    delay(200);
  }

  Serial.println(
    F("# QMC5883L yeniden baslatilamadi.")
  );

  return false;
}


// ============================================================
// ACIYI 0-360 ARASINA GETIR
// ============================================================

float normalizeHeading(float angle)
{
  while (angle < 0.0f)
  {
    angle += 360.0f;
  }

  while (angle >= 360.0f)
  {
    angle -= 360.0f;
  }

  return angle;
}


// ============================================================
// DOGRUSAL INTERPOLASYON
// ============================================================

float interpolateHeading(
  float input,
  float inputMin,
  float inputMax,
  float outputMin,
  float outputMax)
{
  if (inputMax == inputMin)
  {
    return outputMin;
  }

  float ratio =
    (input - inputMin) /
    (inputMax - inputMin);

  return outputMin +
         ratio * (outputMax - outputMin);
}


// ============================================================
// BIRINCI ASAMA HEADING DUZELTMESI
// ============================================================

float correctHeading(float measuredHeading)
{
  float measured =
    normalizeHeading(measuredHeading);

  /*
     0-42 arasindaki degerleri 360-402
     bolgesine tasiyoruz.
  */
  if (measured < MEASURED_AT_270)
  {
    measured += 360.0f;
  }

  float correctedHeading;

  // Ham 42 -> Gercek 270
  // Ham 128 -> Gercek 360 yani 0
  if (measured <= MEASURED_AT_0)
  {
    correctedHeading = interpolateHeading(
      measured,
      MEASURED_AT_270,
      MEASURED_AT_0,
      270.0f,
      360.0f
    );
  }

  // Ham 128 -> Gercek 360
  // Ham 210 -> Gercek 450 yani 90
  else if (measured <= MEASURED_AT_90)
  {
    correctedHeading = interpolateHeading(
      measured,
      MEASURED_AT_0,
      MEASURED_AT_90,
      360.0f,
      450.0f
    );
  }

  // Ham 210 -> Gercek 450
  // Ham 306 -> Gercek 540 yani 180
  else if (measured <= MEASURED_AT_180)
  {
    correctedHeading = interpolateHeading(
      measured,
      MEASURED_AT_90,
      MEASURED_AT_180,
      450.0f,
      540.0f
    );
  }

  // Ham 306 -> Gercek 540
  // Ham 402 -> Gercek 630 yani 270
  else
  {
    correctedHeading = interpolateHeading(
      measured,
      MEASURED_AT_180,
      MEASURED_AT_270 + 360.0f,
      540.0f,
      630.0f
    );
  }

  correctedHeading += HEADING_FINE_OFFSET_DEG;

  return normalizeHeading(correctedHeading);
}


// ============================================================
// SON CIKTIYA 5 NOKTALI HEADING DUZELTMESI
// ============================================================

float correctFinalHeading(float currentHeading)
{
  float measured =
    normalizeHeading(currentHeading);

  /*
     Kesintisiz referans sirasi:

     359       -> Gercek 360 yani 0
     85 + 360  -> Gercek 450 yani 90
     181 + 360 -> Gercek 540 yani 180
     257 + 360 -> Gercek 630 yani 270
     313 + 360 -> Gercek 690 yani 330
     359 + 360 -> Gercek 720 yani 360/0
  */

  if (measured < FINAL_MEASURED_AT_0)
  {
    measured += 360.0f;
  }

  const float measuredAt0 =
    FINAL_MEASURED_AT_0;

  const float measuredAt90 =
    FINAL_MEASURED_AT_90 + 360.0f;

  const float measuredAt180 =
    FINAL_MEASURED_AT_180 + 360.0f;

  const float measuredAt270 =
    FINAL_MEASURED_AT_270 + 360.0f;

  const float measuredAt330 =
    FINAL_MEASURED_AT_330 + 360.0f;

  const float measuredAt360 =
    FINAL_MEASURED_AT_0 + 360.0f;

  float correctedHeading;

  // Mevcut 359 -> 445
  // Gercek 360 -> 450
  if (measured <= measuredAt90)
  {
    correctedHeading = interpolateHeading(
      measured,
      measuredAt0,
      measuredAt90,
      360.0f,
      450.0f
    );
  }

  // Mevcut 445 -> 541
  // Gercek 450 -> 540
  else if (measured <= measuredAt180)
  {
    correctedHeading = interpolateHeading(
      measured,
      measuredAt90,
      measuredAt180,
      450.0f,
      540.0f
    );
  }

  // Mevcut 541 -> 617
  // Gercek 540 -> 630
  else if (measured <= measuredAt270)
  {
    correctedHeading = interpolateHeading(
      measured,
      measuredAt180,
      measuredAt270,
      540.0f,
      630.0f
    );
  }

  // Mevcut 617 -> 673
  // Gercek 630 -> 690
  else if (measured <= measuredAt330)
  {
    correctedHeading = interpolateHeading(
      measured,
      measuredAt270,
      measuredAt330,
      630.0f,
      690.0f
    );
  }

  // Mevcut 673 -> 719
  // Gercek 690 -> 720
  else
  {
    correctedHeading = interpolateHeading(
      measured,
      measuredAt330,
      measuredAt360,
      690.0f,
      720.0f
    );
  }

  return normalizeHeading(correctedHeading);
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  startWire();

  if (initializeQMC5883L())
  {
    Serial.println(F("# QMC5883L basariyla baslatildi."));
  }
  else
  {
    Serial.println(F("# HATA: QMC5883L baslatilamadi!"));
  }

  lastReadMillis = millis();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  feedWatchdog();

  unsigned long currentMillis = millis();

  if (currentMillis - lastReadMillis >= READ_INTERVAL_MS)
  {
    lastReadMillis = currentMillis;

    int16_t rx = 0, ry = 0, rz = 0;

    if (readQMC5883L(rx, ry, rz))
    {
      validReadCount++;
      consecutiveErrors = 0;

      // 1. 3D Kalibrasyon & Yon Duzeltme
      float x_cal = ((float)rx - MAG_X_OFFSET) * MAG_X_SCALE * (float)X_SIGN;
      float y_cal = ((float)ry - MAG_Y_OFFSET) * MAG_Y_SCALE * (float)Y_SIGN;
      float z_cal = ((float)rz - MAG_Z_OFFSET) * MAG_Z_SCALE;

      // 2. Dusuk Geciren Filtre (EMA)
      if (!filterInitialized)
      {
        filteredX = x_cal;
        filteredY = y_cal;
        filteredZ = z_cal;
        filterInitialized = true;
      }
      else
      {
        filteredX += FILTER_ALPHA * (x_cal - filteredX);
        filteredY += FILTER_ALPHA * (y_cal - filteredY);
        filteredZ += FILTER_ALPHA * (z_cal - filteredZ);
      }

      // 3. Ham Heading Hesabi (atan2)
      float rawHeadingRad = atan2(filteredY, filteredX);
      float rawHeadingDeg = rawHeadingRad * (180.0f / (float)M_PI);
      rawHeadingDeg = normalizeHeading(rawHeadingDeg);

      // 4. Birinci Asama Duzeltme
      float stage1Heading = correctHeading(rawHeadingDeg);

      // 5. Ikinci Asama (5 Noktali) Duzeltme
      float finalHeading = correctFinalHeading(stage1Heading);

      // ROS 2 Hardware Bridge tarafindan okunan seri cikti
      Serial.print(F("HEADING:"));
      Serial.println(finalHeading, 2);
    }
    else
    {
      failedReadCount++;
      consecutiveErrors++;

      if (consecutiveErrors >= FULL_RESTART_ERROR_COUNT)
      {
        restartWireAndSensor();
      }
      else if (consecutiveErrors >= RECONFIGURE_ERROR_COUNT)
      {
        initializeQMC5883L();
      }
    }
  }
}
