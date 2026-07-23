/*
  1902A / HD44780 LCD & LED ArUco Gösterici
  -----------------------------------------
  Pin Bağlantıları (Arduino Uno / Mega):
    RS   -> D12
    E    -> D11
    DB4  -> D5
    DB5  -> D4
    DB6  -> D3
    DB7  -> D2
    
    VSS  -> GND
    VDD  -> +5V
    V0   -> Potansiyometre Orta Bacağı
    RW   -> GND
    
    Kırmızı LED -> D7 (Python Çalışıyor / Aranıyor - Kırmızı YANAR, Mavi SÖNER)
    Mavi LED    -> D6 (25 cm Altında ArUco Algılanınca Sabit 3 Saniye YANAR)
*/

#include <LiquidCrystal.h>

// LCD Pinleri: RS, E, DB4, DB5, DB6, DB7
const int pin_RS  = 12;
const int pin_E   = 11;
const int pin_DB4 = 5;
const int pin_DB5 = 4;
const int pin_DB6 = 3;
const int pin_DB7 = 2;

// LED Pinleri
const int pin_RED_LED  = 7;
const int pin_BLUE_LED = 6;

LiquidCrystal lcd(pin_RS, pin_E, pin_DB4, pin_DB5, pin_DB6, pin_DB7);

String inputString = "";
bool stringComplete = false;
unsigned long lastMsgTime = 0;
int lastDisplayedID = -999;

// 3 Saniye Mavi LED Zamanlayıcı Değişkenleri
unsigned long blueHoldStartTime = 0;
bool isBlueHolding = false;
const unsigned long BLUE_HOLD_MS = 3000; // 3 saniye (3000 ms)

// Metni 16 karakterlik satırda tam ortalayarak yazdıran fonksiyon
void printCentered(String text, int row = 0) {
  int len = text.length();
  int col = (16 - len) / 2;
  if (col < 0) col = 0;
  lcd.setCursor(col, row);
  lcd.print(text);
}

void setLeds(bool redOn, bool blueOn) {
  digitalWrite(pin_RED_LED, redOn ? HIGH : LOW);
  digitalWrite(pin_BLUE_LED, blueOn ? HIGH : LOW);
}

void setup() {
  Serial.begin(115200);
  inputString.reserve(64);

  // LED Pin Ayarları
  pinMode(pin_RED_LED, OUTPUT);
  pinMode(pin_BLUE_LED, OUTPUT);

  // Başlangıçta tüm LED'ler sönük
  setLeds(false, false);

  // 1902A LCD 16 sütun x 2 satır modunda başlatılır
  lcd.begin(16, 2);
  lcd.clear();
  
  printCentered("BEKLENIYOR", 0);
  lastMsgTime = millis();
}

void loop() {
  unsigned long now = millis();

  // Seri porttan veri oku
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n' || inChar == '\r') {
      if (inputString.length() > 0) {
        stringComplete = true;
      }
    } else {
      inputString += inChar;
    }
  }

  // Mavi LED 3 saniyelik zamanlayıcı kontrolü
  if (isBlueHolding) {
    if (now - blueHoldStartTime < BLUE_HOLD_MS) {
      setLeds(false, true); // Kırmızı SÖNER, Mavi YANAR
    } else {
      isBlueHolding = false; // 3 saniye doldu
    }
  }

  // Seri komut işleme
  if (stringComplete) {
    inputString.trim();
    lastMsgTime = now;

    if (inputString.startsWith("ID:")) {
      int idVal = inputString.substring(3).toInt();
      
      // 25 cm altında ArUco algılandı -> Mavi LED'i 3 saniye sabitle
      isBlueHolding = true;
      blueHoldStartTime = now;
      setLeds(false, true);

      if (idVal != lastDisplayedID) {
        lastDisplayedID = idVal;
        
        lcd.clear();
        String dispText = "ID: " + String(idVal);
        printCentered(dispText, 0); // Ekranda ortalanmış ID
      }
    } 
    else if (inputString == "NONE" || inputString == "ID:-1") {
      // ArUco yoksa ancak 3 saniyelik mavi tutma süresi bittiyse kırmızıya geç
      if (!isBlueHolding) {
        setLeds(true, false);

        if (lastDisplayedID != -1) {
          lastDisplayedID = -1;
          
          lcd.clear();
          printCentered("---", 0);
        }
      }
    }

    inputString = "";
    stringComplete = false;
  }

  // 4 saniyedir seri veri gelmediyse (Python kapalıysa) bağlantı uyarısı ver ve LED'leri söndür
  if (now - lastMsgTime > 4000) {
    isBlueHolding = false;
    setLeds(false, false);
    if (lastDisplayedID != -999) {
      lastDisplayedID = -999;
      lcd.clear();
      printCentered("BAGLANTI YOK", 0);
    }
  }
}
