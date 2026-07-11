#include "arduinoFFT.h"
#define SAMPLES 128             
#define SAMPLING_FREQ 1000.0

double vRealX[SAMPLES];
double vImagX[SAMPLES];

ArduinoFFT<double> FFT_X = ArduinoFFT<double>(vRealX, vImagX, SAMPLES, SAMPLING_FREQ);
unsigned int sampling_period_us;
unsigned long microseconds;

void setup() {
  Serial.begin(115200);
  sampling_period_us = round(1000000.0 / SAMPLING_FREQ);
  
}

void loop() {
  // ambil data sensor
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();

    // dummy data: simulasi frekuensi dominan 50Hz pada sumbu x
    vRealX[i] = 2.0 * sin(2 * PI * 50.0 * (i / SAMPLING_FREQ)) + random(-10, 10) / 100.0; 
    vImagX[i] = 0.0;

    while (micros() - microseconds < sampling_period_us) { }
  }

  //fft 
  FFT_X.windowing(FFTWindow::Hamming, FFTDirection::Forward); 
  FFT_X.compute(FFTDirection::Forward);
  FFT_X.complexToMagnitude();

  //ekstraksi fitur data
  double peakFrequencyX = FFT_X.majorPeak();
  
  Serial.print("peak freq: "); //dalam Hz
  Serial.print(peakFrequencyX);

  // Format JSON untuk Node-RED
  Serial.print("{\"axis\":\"X\", \"peak_freq\":");
  Serial.print(peakFrequencyX);
  Serial.println("}");

  delay(2000); 
}