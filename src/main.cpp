#include <Arduino.h>
#include "driver/rtc_io.h"
// #include "ESP_I2S.h"
#include "ESP32RotaryEncoder.h"
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

#ifndef LED_POWER
#define LED_POWER GPIO_NUM_13
#endif

#ifndef BUTTON_SLEEP
#define BUTTON_SLEEP GPIO_NUM_14
#endif

#ifndef DI_ENCODER_A
#define DI_ENCODER_A 35  // volume up
#endif

#ifndef DI_ENCODER_B
#define DI_ENCODER_B 32  // volume down
#endif

#ifndef I2S_BCK
#define I2S_BCK 25
#endif

#ifndef I2S_WS
#define I2S_WS 26
#endif

#ifndef I2S_DATA_OUT
#define I2S_DATA_OUT 33
#endif

RotaryEncoder rotaryEncoder(DI_ENCODER_A, DI_ENCODER_B);

RTC_DATA_ATTR int sleep_count = 0;

// Output to the internal DAC
/* 
AnalogAudioStream out;
BluetoothA2DPSink a2dp_sink(out);
*/

AudioInfo info(44100, 2, 32);
I2SStream out;
NumberFormatConverterStream convert(out);
BluetoothA2DPSink a2dp_sink(convert);
// BluetoothA2DPSink a2dp_sink(out);

esp_a2d_connection_state_t last_connection_state;
uint16_t minutes = 10;
unsigned long shutdown_ms = millis() + 1000 * 60 * minutes;

void on_data() {
  shutdown_ms = millis() + 1000 * 60 * minutes;
}

void show_wake_reason() {
  auto cause = esp_sleep_get_wakeup_cause();
  switch (cause) {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup reason: EXT0");
      break;
    default:
      Serial.printf("Wakeup reason: %d\n", cause);
  }
  Serial.printf("Count %d\n", ++sleep_count);
}

volatile bool button_pressed = false;

void IRAM_ATTR button_isr() {
  button_pressed = true;
}

void knobCallback(long value) {
  Serial.printf("Value: %ld\n", value);
  int newVol = a2dp_sink.get_volume() + value * 5;
  Serial.printf("newVol: %d\n", newVol);
  if (newVol < 0) {
    newVol = 0;
  } else if (newVol > 127) {
    newVol = 127;
  }
  a2dp_sink.set_volume(newVol);
}

void enter_sleep() {
  detachInterrupt(digitalPinToInterrupt(BUTTON_SLEEP));
  Serial.println("Going to sleep...");
  esp_sleep_enable_ext0_wakeup(BUTTON_SLEEP, LOW);
  esp_deep_sleep_start();
}

void setup() {

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

  pinMode(LED_POWER, OUTPUT);
  pinMode(BUTTON_SLEEP, INPUT_PULLUP);

  Serial.begin(115200);
  delay(500);

  show_wake_reason();

  attachInterrupt(digitalPinToInterrupt(BUTTON_SLEEP), button_isr, RISING);

  //Output using the ESP32 I2S API
  /*
  I2S.setSckPin(14);
  I2S.setFsPin(15);
  I2S.setDataPin(22);

  if (!I2S.begin(I2S_PHILIPS_MODE, 44100, 16)) {
    Serial.println("Failed to initialize I2S");
    while (1);
  }
  a2dp_sink.start("ESP32_Bluetooth");
  */

  // Default I2S pins can be changed
  // Note: PCM5102 I2S won't work without setting RxTxMode!!!
  auto cfg = out.defaultConfig(TX_MODE);
  // Note: pin 14 and 15 are not working for MAX98357A
  cfg.pin_bck = I2S_BCK;
  cfg.pin_ws = I2S_WS;
  cfg.pin_data = I2S_DATA_OUT;
  cfg.i2s_format = I2S_STD_FORMAT;
  // i2s.begin(cfg);
  cfg.copyFrom(info);
  out.begin(cfg);

  // Convert from 16 to 32 bits
  convert.begin(16, 32);

  // a2dp_sink.set_auto_reconnect(true);
  a2dp_sink.set_on_data_received(on_data);
  a2dp_sink.start("JBL Pebbles");
  
  rotaryEncoder.setEncoderType(EncoderType::FLOATING);
  rotaryEncoder.setBoundaries(-1, 1, false);
  rotaryEncoder.onTurned(&knobCallback);
  rotaryEncoder.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (button_pressed) {
    Serial.println("Rotary Encoder's switch is pressed!");
    button_pressed = false;
    enter_sleep();
  }

  if (millis() > shutdown_ms) {
    Serial.println("No data for a while, going to sleep now");
    enter_sleep();
  }

  esp_a2d_connection_state_t state = a2dp_sink.get_connection_state();
  if (last_connection_state != state) {
    bool is_connected = state == ESP_A2D_CONNECTION_STATE_CONNECTED;
    Serial.println(is_connected ? "Connected" : "Disconnected");
    digitalWrite(LED_POWER, is_connected);
    last_connection_state = state;
  }
}
