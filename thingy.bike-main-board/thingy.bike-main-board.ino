#include <esp_now.h>
#include <WiFi.h>
#include "VescUart.h"

/** Initiate VescUart class */
VescUart UART;
time_t last_message_time = 0;

void new_message_received(const uint8_t *mac, const uint8_t *incomingData, int len) {
  /*Serial.print("Bytes received: ");
  Serial.println(len);*/

  float gyro_z;
  memcpy (&gyro_z, incomingData, 4);

  /*Serial.println("Gyro z:");
  Serial.println(gyro_z);*/

  if (gyro_z < -40){
    /** Call the function setCurrent() to set the motor current */
    UART.setCurrent(2);
  }else{
    UART.setCurrent(0);
  }

    last_message_time = millis();

 
}

void init_vesc_uart(){
  
    /** Setup UART port (Serial1 on Atmega32u4) */
  Serial1.begin(115200, SERIAL_8N1, 4, 5);

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial1);

}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  Serial.println(WiFi.macAddress());

  esp_now_register_recv_cb(new_message_received);

  init_vesc_uart();
}

void loop() {

  /** Call the function getVescValues() to acquire data from VESC */
  /*if ( UART.getVescValues() ) {

    Serial.println(UART.data.rpm);
    Serial.println(UART.data.inpVoltage);
    Serial.println(UART.data.ampHours);
    Serial.println(UART.data.tachometerAbs);

  }
  else
  {
    Serial.println("Failed to get data!");
  }*/

  delay(10);

  if (  millis() - last_message_time > 500){
    UART.setCurrent(0);
  }

}
