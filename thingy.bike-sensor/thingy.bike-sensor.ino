#include "RAK12033-IIM42652.h"

#include <esp_now.h>
#include <WiFi.h>

IIM42652 IMU;
#define ACCEL_X_THR 100
#define ACCEL_Y_THR 100
#define ACCEL_Z_THR 100
#define INT_PIN 3
#define GPIO_WAKE_UP false

//Sleep settings
#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 0.2         /* Time ESP32 will go to sleep (in seconds) */

bool sleep_scheduled = false;

//Replace this address with the address of a main board in your kit
uint8_t main_board_address[] = { 0xA0, 0x76, 0x4E, 0x7E, 0x44, 0x74 };

esp_now_peer_info_t car_controller_info;

RTC_DATA_ATTR time_t last_active_time = 0;

void i_sent_data_to_main_board(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Message has been delivered" : "Upps. Cannot deliver a message");
  sleep_scheduled = false;
  if (status == ESP_NOW_SEND_SUCCESS) {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}

void setup_ESP_NOW() {

  if (WiFi.getMode() == WIFI_STA) {
    return;
  }

  WiFi.mode(WIFI_STA);

  WiFi.setTxPower(WIFI_POWER_2dBm);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(i_sent_data_to_main_board);

  memcpy(car_controller_info.peer_addr, main_board_address, 6);
  car_controller_info.channel = 0;
  car_controller_info.encrypt = false;

  if (esp_now_add_peer(&car_controller_info) != ESP_OK) {
    Serial.println("Cannot connect to a main board");
    return;
  }
}

void float_2_bytes(uint8_t bytes_temp[4], float float_variable) {
  memcpy(bytes_temp, (uint8_t *)(&float_variable), 4);
}

void setup() {
  setCpuFrequencyMhz(80);

  time_t timeout = millis();
  // Initialize Serial for debug output.
  Serial.begin(115200);
  while (!Serial) {
    if ((millis() - timeout) < 5000) {
      delay(100);
    } else {
      break;
    }
  }
  Serial.println("RAK12033 Basic Reading example.");

  Wire.begin(4, 5);

  if (IMU.begin(Wire, 0x68) == false) {
    while (1) {
      Serial.println("IIM-42652 is not connected.");
      delay(5000);
    }
  }

  IMU.ex_idle();
  //IMU.accelerometer_enable();
  IMU.gyroscope_enable();
  //IMU.temperature_enable();

  delay(50);
}

void loop() {

  if (sleep_scheduled == false) {
    IIM42652_axis_t accel_data;
    IIM42652_axis_t gyro_data;
    float temp;

    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;


    //IMU.get_accel_data(&accel_data);
    IMU.get_gyro_data(&gyro_data);
    //IMU.get_temperature(&temp);

    /*
   * ±16 g  : 2048  LSB/g
   * ±8 g   : 4096  LSB/g
   * ±4 g   : 8192  LSB/g
   * ±2 g   : 16384 LSB/g
   */
    /*acc_x = (float)accel_data.x / 2048;
  acc_y = (float)accel_data.y / 2048;
  acc_z = (float)accel_data.z / 2048;

  Serial.print("Accel X: ");
  Serial.print(acc_x);
  Serial.print("[g]  Y: ");
  Serial.print(acc_y);
  Serial.print("[g]  Z: ");
  Serial.print(acc_z);
  Serial.println("[g]");*/

    /*
   * ±2000 º/s    : 16.4   LSB/(º/s)
   * ±1000 º/s    : 32.8   LSB/(º/s)
   * ±500  º/s    : 65.5   LSB/(º/s)
   * ±250  º/s    : 131    LSB/(º/s)
   * ±125  º/s    : 262    LSB/(º/s)
   * ±62.5  º/s   : 524.3  LSB/(º/s)
   * ±31.25  º/s  : 1048.6 LSB/(º/s)
   * ±15.625 º/s  : 2097.2 LSB/(º/s)
   */
    /*gyro_x = (float)gyro_data.x / 16.4;
    gyro_y = (float)gyro_data.y / 16.4;*/
    gyro_z = (float)gyro_data.z / 16.4;

    /*Serial.print("Gyro  X:");
  Serial.print(gyro_x);
  Serial.print("º/s  Y: ");
  Serial.print(gyro_y);*/
    /*Serial.print("º/s  Z: ");
    Serial.print(gyro_z);
    Serial.println("º/s");*/

    /*Serial.print("Temper : ");
  Serial.print(temp);
  Serial.println("[ºC]");

  IMU.accelerometer_disable();
  IMU.gyroscope_disable();
  IMU.temperature_disable();
  IMU.idle();*/

    uint8_t message[4] = { 0 };
    float_2_bytes(message, gyro_z);

    bool msg_should_be_sent = false;
    if (gyro_z > 40 || gyro_z < -40) {
      last_active_time = millis();
      msg_should_be_sent = true;
    } else {
      if (millis() - last_active_time > 1000 || millis() < 1000) {
        msg_should_be_sent = false;
      } else {
        msg_should_be_sent = true;
      }
    }

    // Send message via ESP-NOW
    if (msg_should_be_sent == true) {
      setup_ESP_NOW();
      sleep_scheduled = true;
      esp_err_t result = esp_now_send(main_board_address, message, sizeof(message));
    } else {
      sleep_scheduled = true;

      if (GPIO_WAKE_UP == true) {
        //pinMode(INT_PIN, INPUT_PULLUP);  // setup for interrupt
        //  attachInterrupt(digitalPinToInterrupt(INT_PIN), int1_ISR, FALLING);
        esp_deep_sleep_enable_gpio_wakeup(1 << INT_PIN, ESP_GPIO_WAKEUP_GPIO_LOW);

        IMU.accelerometer_enable();
        IMU.gyroscope_disable();
        IMU.temperature_disable();

        IMU.set_accel_fsr(IIM42652_ACCEL_CONFIG0_FS_SEL_16g);
        IMU.set_accel_frequency(IIM42652_ACCEL_CONFIG0_ODR_50_HZ);

        IMU.enable_accel_low_power_mode();
        IMU.wake_on_motion_configuration(ACCEL_X_THR, ACCEL_Y_THR, ACCEL_Z_THR);

      } else {
        esp_sleep_enable_timer_wakeup(0.6 * uS_TO_S_FACTOR);
      }

      esp_deep_sleep_start();
    }
  }
}