#include <EEPROM.h>
#define EEPROM_SIZE 12 // Buat EEPROM 12 byte

// ----------- DEKLARASI VARIABEL--------------- //
// VARIABEL SWITCH PENGENDALI
int ISRpin = 18;
bool state = false;
bool flag = false;

// VARIABEL INTERUPSI TIMER
volatile int sample;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// VARIABEL PENGENDALI
float zeroSig = 0; // Nilai nol untuk awal
float refSig = 11.1; // Nilai referensi
float dataCont = 0; // Nilai sinyal kontrol, diatur sama dengan referensi di awal
float feedbackSig = 0; // Sinyal umpan balik dari plant
float error_P = 0, error_I = 0, error_D = 0, lastError = 0;

// VARIABEL PID
float Kp = 1;
float Ki = 0;
float Kd = 0.1;

int addKp = 0;
int addKi = 4; // float 4 byte
int addKd = 8;

// ----------- DEFINISI FUNGSI INTERUPSI --------------- //
// Interupsi untuk switch on/off pengendali
void IRAM_ATTR ISRbutton() {
  flag = true;
  detachInterrupt(digitalPinToInterrupt(ISRpin));
}

void IRAM_ATTR ISRtimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  sample = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ----------- SETUP --------------- //
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  EEPROM.begin(EEPROM_SIZE);
  
  // button untuk switch on/off pengendali
  pinMode(ISRpin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ISRpin), ISRbutton, FALLING);

  nyalakanTimer();
  
  delay(5000); // Tunggu 5s sebelum loop dimulai untuk membuka Serial Monitor
}

// ----------- MAIN LOOP --------------- //
void loop() {
  
// Switch on/off pengendali
  if (flag) {
    state = !state;
    if (state) {
      dataCont = refSig;
    }
    else {
      dataCont = 0;
    }
    matikanTimer();
    delay(1000);
    nyalakanTimer();
    flag = false;
    attachInterrupt(digitalPinToInterrupt(ISRpin), ISRbutton, FALLING);
  }

  if(sample) {
    // Baca dan proses PID dari serial setiap 100 ms (interupsi timer)
    while(Serial2.available() > 0){
      delay(10);
      feedbackSig = readData();
      PIDProcess();
    }
  }

  if (Serial.available()) { // Ada data dari serial port
    rewriteConstants();
  }
  
// Kirim ke serial setiap 100 ms
  writeData(dataCont);
  delay(100);
}

// ----------- DEFINISI FUNGSI --------------- //
// FUNGSI PID
void PIDProcess(){
  if(state) {
    error_P = refSig - feedbackSig;
    error_I += error_P;
    error_D = (error_P - lastError);

    dataCont = (Kp * error_P) + (Ki * error_I) + (Kd * error_D);

    lastError = error_P;
  }
  // do nothing, controller off
}

// FUNGSI BACA DATA SERIAL
float readData() {
  // Variabel lokal untuk membaca data string ke float
  char temp;
  char myData[10] = "";
  int intData = 0;
  float floData = 0.0;

  for (int i = 0; i < 10; i++) {
    temp = Serial2.read();
    if (isDigit(temp)) {
      myData[i] = temp;
    }
    else if (temp == 0x0A) {
      break;
    }
    else {
      i--; // not a digit, decrement once
    }
  }
  intData = atoi(myData);
  floData = (float)intData * 0.01;
  return floData;
}

// FUNGSI KIRIM DATA SERIAL
void writeData(float myData) {
  Serial2.print(myData,2);
  Serial2.println(); // print 0x0A sebagai tanda terminasi data
}

// FUNGSI TIMER ON
void nyalakanTimer() {
  timer = timerBegin(0, 80, true); // Pilih timer 0, prescaler 80Mhz/80= 10Mhz, counts up
  timerAttachInterrupt(timer, &ISRtimer, true);  // Fungsi handler, tipe timer edge
  timerAlarmWrite(timer, 100000, true); // Timer 1000000 us = 1 s, counter reload true
  timerAlarmEnable(timer);
}

// FUNGSI TIMER OFF
void matikanTimer() {
  timerAlarmDisable(timer);    // stop alarm
  timerDetachInterrupt(timer);  // detach interrupt
  timerEnd(timer);
}

void rewriteConstants() {
  // Variabel lokal untuk membaca data string ke float
  char temp;
  char myData[10] = "";
  int intData = 0;
  float floData = 0.0;
  bool rewriteKp = false, rewriteKi = false, rewriteKd = false;

  for (int i = 0; i < 10; i++) {
    temp = Serial.read();
    
    if (isDigit(temp)) {
      myData[i] = temp;
    }
    else if (temp == 0x0A) {
      break;
    }
    else if (temp == 'p') {
      rewriteKp = true;
      i--;
    }
    else if (temp == 'i') {
      rewriteKi = true;
      i--;
    }
    else if (temp == 'd') {
      rewriteKd = true;
      i--;
    }
    else {
      i--; // not a digit, decrement once
    }
  }
  
  intData = atoi(myData);
  floData = (float)intData * 0.01;

  if (rewriteKp) {
    Serial.println("Rewriting data Kp...");
    EEPROM.put(addKp, floData);
    EEPROM.get(addKp, Kp);
  }
  else if (rewriteKi) {
    Serial.println("Rewriting data Ki...");
    EEPROM.put(addKi, floData);
    EEPROM.get(addKi, Ki);
  }
  else if (rewriteKd) {
    Serial.println("Rewriting data Kd...");
    EEPROM.put(addKd, floData);
    EEPROM.get(addKd, Kd);
  }

  // Tampilkan data di Serial Monitor 
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(" Ki: ");
  Serial.print(Ki);
  Serial.print(" Kd: ");
  Serial.println(Kd);
}
