// =====================================================
// ARDUINO MEGA - ROS2 ROVER MOTOR KONTROLCUSU
// =====================================================
//
// Desteklenen komutlar:
//
// MOTOR:FWD:80
// MOTOR:BACK:80
// MOTOR:LEFT:80
// MOTOR:RIGHT:80
// MOTOR:STOP
//
// Raspberry Pi belirli aral?klarla komut göndermelidir.
// KOMUT_TIMEOUT_MS süresince komut gelmezse motorlar durur.
// =====================================================


// =====================================================
// MOTOR SÜRÜCÜ P?NLER?
// =====================================================

// Sol BTS7960
const int L_RPWM = 5;
const int L_LPWM = 6;
const int L_REN  = 7;
const int L_LEN  = 12;

// Sa? BTS7960
const int R_RPWM = 13;
const int R_LPWM = 10;
const int R_REN  = 11;

// ?ki BTS7960 LEN ba?lant?s? ortak pin 12'ye ba?l?.
const int R_LEN  = 12;


// =====================================================
// GÜVENL?K AYARLARI
// =====================================================

// Bu süre boyunca yeni hareket komutu gelmezse motorlar? durdur.
const unsigned long KOMUT_TIMEOUT_MS = 750UL;

unsigned long sonHareketKomutuMs = 0;

bool hareketAktif = false;


// =====================================================
// MOTOR KONTROL FONKS?YONLARI
// =====================================================

void dur()
{
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);

  hareketAktif = false;
}


void hareketWatchdogBaslat()
{
  sonHareketKomutuMs = millis();
  hareketAktif = true;
}


void ileri(int pwm)
{
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  hareketWatchdogBaslat();
}


void geri(int pwm)
{
  pwm = constrain(pwm, 0, 255);

  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  hareketWatchdogBaslat();
}


void sagaDon(int pwm)
{
  pwm = constrain(pwm, 0, 255);

  // Sol motor ileri
  analogWrite(L_RPWM, pwm);
  analogWrite(L_LPWM, 0);

  // Sa? motor geri
  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, pwm);

  hareketWatchdogBaslat();
}


void solaDon(int pwm)
{
  pwm = constrain(pwm, 0, 255);

  // Sol motor geri
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, pwm);

  // Sa? motor ileri
  analogWrite(R_RPWM, pwm);
  analogWrite(R_LPWM, 0);

  hareketWatchdogBaslat();
}


// =====================================================
// PWM DE?ER?N? KOMUTTAN AYIR
// =====================================================

bool pwmOku(
  const String &komut,
  const String &prefix,
  int &pwm)
{
  if (!komut.startsWith(prefix))
  {
    return false;
  }

  String pwmMetni = komut.substring(prefix.length());
  pwmMetni.trim();

  if (pwmMetni.length() == 0)
  {
    return false;
  }

  // PWM bölümünde yaln?zca rakam bulunmal?.
  for (unsigned int i = 0; i < pwmMetni.length(); i++)
  {
    if (!isDigit(pwmMetni.charAt(i)))
    {
      return false;
    }
  }

  long deger = pwmMetni.toInt();

  if (deger < 0 || deger > 255)
  {
    return false;
  }

  pwm = (int)deger;

  return true;
}


// =====================================================
// SER? KOMUTU ??LE
// =====================================================

void komutuIsle(String komut)
{
  komut.trim();

  if (komut.length() == 0)
  {
    return;
  }

  int pwm = 0;


  // ---------------------------------------------------
  // DUR
  // ---------------------------------------------------

  if (komut == "MOTOR:STOP")
  {
    dur();

    Serial.println(F("ACK,MOTOR:STOP"));

    return;
  }


  // ---------------------------------------------------
  // ?LER?
  // ---------------------------------------------------

  if (pwmOku(komut, "MOTOR:FWD:", pwm))
  {
    ileri(pwm);

    Serial.print(F("ACK,MOTOR:FWD,"));
    Serial.println(pwm);

    return;
  }


  // ---------------------------------------------------
  // GER?
  // ---------------------------------------------------

  if (pwmOku(komut, "MOTOR:BACK:", pwm))
  {
    geri(pwm);

    Serial.print(F("ACK,MOTOR:BACK,"));
    Serial.println(pwm);

    return;
  }


  // ---------------------------------------------------
  // SA?A DÖN
  // ---------------------------------------------------

  if (pwmOku(komut, "MOTOR:RIGHT:", pwm))
  {
    sagaDon(pwm);

    Serial.print(F("ACK,MOTOR:RIGHT,"));
    Serial.println(pwm);

    return;
  }


  // ---------------------------------------------------
  // SOLA DÖN
  // ---------------------------------------------------

  if (pwmOku(komut, "MOTOR:LEFT:", pwm))
  {
    solaDon(pwm);

    Serial.print(F("ACK,MOTOR:LEFT,"));
    Serial.println(pwm);

    return;
  }


  // Tan?nmayan veya bozuk bir komut geldiyse güvenlik için dur.
  dur();

  Serial.print(F("ERR,INVALID_COMMAND,"));
  Serial.println(komut);
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  // Eksik seri sat?r?nda uzun süre beklememesi için.
  Serial.setTimeout(30);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  // BTS7960 enable pinlerini aktif et.
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  dur();

  delay(500);

  Serial.println(F("BOOT,MOTOR_CONTROLLER_READY"));
}


// =====================================================
// LOOP
// =====================================================

void loop()
{
  // Yeni komut gelmediyse motorlar? otomatik durdur.
  if (
    hareketAktif &&
    (unsigned long)(millis() - sonHareketKomutuMs)
      >= KOMUT_TIMEOUT_MS)
  {
    dur();

    Serial.println(F("WARN,COMMAND_TIMEOUT,MOTOR_STOPPED"));
  }


  // Raspberry Pi veya Serial Monitor üzerinden komut oku.
  if (Serial.available() > 0)
  {
    String komut = Serial.readStringUntil('\n');

    komutuIsle(komut);
  }
}
