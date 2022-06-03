// ----------- DEKLARASI VARIABEL--------------- //
// VARIABEL PLANT
float y, x, xn1, yn1;
float refSig = 11.1;

// VARIABEL INTERUPSI TIMER
volatile int sample;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// ----------- DEFINISI FUNGSI INTERUPSI --------------- //
void IRAM_ATTR ISRtimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  sample = 1;
  portEXIT_CRITICAL_ISR(&timerMux);
}

// ----------- SETUP --------------- //
void setup() {
 Serial.begin(115200);
 Serial2.begin(115200);
  
  x = 0;
  y = 0;
  xn1 = 0;
  yn1 = 0;

  // timer interrupt
  timer = timerBegin(0, 80, true); // Pilih timer 0, prescaler 80Mhz/80= 10Mhz, counts up
  timerAttachInterrupt(timer, &ISRtimer, true);  // Fungsi handler, tipe timer edge
  timerAlarmWrite(timer, 100000, true); // Timer 1000000 us = 1 s, counter reload true
  timerAlarmEnable(timer);
  
  delay(5000); // Tunggu 5s sebelum loop dimulai untuk membuka Serial Monitor
}


// ----------- MAIN LOOP --------------- //
void loop() {
// Kirim data ke pengendali
  writeData(y); // Kirim output fungsi plant

  if (sample) {
    // Baca dan proses data dari pengendali setiap 100 ms
    while(Serial2.available() > 0){
      delay(10);
      x = readData();
      plantProcess(); // Hitung output plant berdasarkan input pengendali
    }
  }

// Tampilkan data di Serial Monitor
  Serial.print("Ref Signal: ");
  Serial.println(refSig);
  Serial.print("Output: ");
  Serial.println(y);
  delay(100);
}

// ----------- DEFINISI FUNGSI --------------- //
// FUNGSI PLANT (MOTOR)
void plantProcess() {
  y =  0.04676 * x + 0.04676 * xn1 + 0.9048 * yn1;
  yn1 = y;
  xn1 = x;
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
