#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <EEPROM.h>
#include "MPU9250.h"
#include "config.hpp"


#ifdef MPU_ON
MPU9250 IMU(Wire, 0x68);
#endif
uint8_t serial_buf[BYTES_TO_READ] = {0,};
bool uart_flag = false;
int sensors[6]; // - array for saving sensors signal values

struct_message myData; // structure for transmitting with esp_now
charge_state_msg charge = {DEVICE_ID, 0};
unsigned long ground_zeros = 0;
volatile uint8_t bracelet_mode = 0;
volatile uint8_t gesture_mode = 1;
volatile bool is_pressed = false;
unsigned long switch_off_start_time = millis();
esp_now_peer_info_t peerInfo; // creating peer interface
uint8_t broadcastAddress_pc[] = {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF4};
/*1 версия приемника: {0x80, 0x7D, 0x3A, 0x99, 0x85, 0x14}
  2 версия приемника (с оригинальным stm): {0x30, 0xc6, 0xf7, 0xb0, 0xf6, 0xf0}
  2 версия приемника (в корпусе): {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xF4}
  машинка: {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xEC}
*/
uint8_t broadcastAddress_car[] = {0x30, 0xC6, 0xF7, 0xB0, 0xF6, 0xEC}; // car's ESP MAC-address


void setup() {
#ifndef OLD_VERSION
    enable_supply();
#endif
    setCpuFrequencyMhz(80);
    init_serials();
    read_last_state();
    init_base_pins();
    init_wireless_connection();
#if defined(MPU_ON)
    mpu_init();
#endif
    set_all_signal_coeff(30);
    delay(1000);
#ifdef WIFI_OFF_TEST
    WiFi.mode(WIFI_OFF);
#endif
    ground_zeros = millis();
}


void loop() {
    change_mode_by_button();
    check_switch_off_by_button();
#if !defined(TEST_SIGNALS_READING) && !defined(MPU_TESTING) && defined(MPU_ON)
    /* MAIN */
    inform_about_charge();
    get_signals();
    state_machine();
#elif defined(MPU_TESTING)
    // mpu_get_all_data();
    mpu_get_print_accelerations();
    // mpu_get_roll_pitch_angles();
#elif defined(ANALOG_SIGNALS_PRINTING)
    Serial.print("Voltage: ");
    Serial.print(analogRead(IN_VOLTAGE));
    Serial.print("\tTemperature: ");
    Serial.println(analogRead(TERM));
    delay(500);
#else
    get_signals();
    Serial.print(sensors[0]);
    Serial.print('\t');
    Serial.print(sensors[1]);
    Serial.print('\t');
    Serial.print(sensors[2]);
    Serial.print('\t');
    Serial.print(sensors[3]);
    Serial.print('\t');
    Serial.print(sensors[4]);
    Serial.print('\t');
    Serial.println(sensors[5]);
#endif
}


/********************************************************************************/
void enable_supply(void) {
    pinMode(26, OUTPUT);
    digitalWrite(26, 1);
}

void init_serials(void) {
    Serial.begin(115200);
    while (!Serial);
    Serial2.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN, false);
    while (!Serial2);
}

void WiFi_enable(void) {
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) error_clbck();

    memcpy(peerInfo.peer_addr, broadcastAddress_pc, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) error_clbck();

    memcpy(peerInfo.peer_addr, broadcastAddress_car, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) error_clbck();

    if (WiFi.setTxPower(WIFI_POWER_2dBm) != 1) error_clbck();
    WiFi.disconnect();
}

void init_wireless_connection(void) {
    myData.id = DEVICE_ID;
    WiFi_enable();
    Serial.println("Device is ready");
    delay(100);
}

void change_mode_by_button(void) {
    if (!digitalRead(WKUP)) {
        digitalWrite(RX_1_LED, bracelet_mode);
        delay(1000);
        bracelet_mode = (bracelet_mode) ? 0 : 1;
        EEPROM.write(0, bracelet_mode);
        EEPROM.commit();
    }
}

void check_switch_off_by_button(void) {
    if (!digitalRead(MC_DIN)) {
        if (!is_pressed) {
            is_pressed = true;
            switch_off_start_time = millis();
        } else {
            unsigned long current_time = millis();
            if (switch_off_start_time > current_time) {
                current_time += (0xFFFFFFFF - switch_off_start_time);
                switch_off_start_time = 0;
            }
            
            if (current_time - switch_off_start_time >= SWITCH_OFF_TIME) {
                esp_deep_sleep_start();
            }
        }
    } else if (is_pressed) {
        is_pressed = false;
    }
}

void set_signal_coefficient(uint8_t cell_addr, uint8_t value) {
    /*
    potentiometer addresses:
    AD5206 | number in the line
    ---------------------------
    addr 0 |    4 (4-old)
    addr 1 |    6 (5-old)
    addr 2 |    3 (3-old)
    addr 3 |    5 (6-old)
    addr 4 |    2 (2-old)
    addr 5 |    1 (1-old)
    */
    uint8_t tmp_buf[2] = {cell_addr, value};
    Serial2.write((char *) tmp_buf, 2);
}

void set_all_signal_coeff(uint8_t unit_val) {
    for (uint8_t i = 0; i < 6; i++) {
        set_signal_coefficient(i, unit_val);
        delay(250);
    }
}

void get_signals(void) {
    if (Serial2.read() == 'S') {
        uart_flag = true;
        Serial2.readBytes(serial_buf, BYTES_TO_READ);
        uint8_t tmp_count = 0;
        for (uint8_t i = 0; i < 6; i++) {
            sensors[i] = serial_buf[tmp_count] | (serial_buf[tmp_count+1] << 8);
            tmp_count += 2;
        }
    }
}

void send_data(uint8_t *addr_ptr) {
    if (uart_flag) {
        uart_flag = false;
#if (DEVICE_ID == 3)
        gesture_recognizing();
#endif
#ifdef MPU_ON
        mpu_data_processing();
#endif
#ifdef WIFI_OFF_TEST
        WiFi_enable();
#endif
        esp_now_send(addr_ptr, (uint8_t *)&myData, sizeof(myData));
#ifdef WIFI_OFF_TEST
        WiFi.mode(WIFI_OFF);
#endif
    }
}

void gesture_recognizing(void) {
    switch (gesture_mode) {
        case 0:
            if ((1500 <= sensors[1]) && (1500 <= sensors[2]) && (1500 <= sensors[4])) {
                myData.w = 1; // есть жест
            } else {
                myData.w = 0; // расслабленная рука
            }
            break;
        case 1:
            if (2000 <= sensors[4]) {
                myData.w = 1; // есть жест
            } else {
                myData.w = 0; // расслабленная рука
            }
            break;
    }
}

void init_base_pins(void) {
#ifdef OLD_VERSION
    pinMode(LED3, OUTPUT);
    pinMode(LED4, OUTPUT);
    digitalWrite(LED3, 1);
    digitalWrite(LED4, 1);
#else
    pinMode(RX_1_LED, OUTPUT); // led
    pinMode(RX_3_LED, OUTPUT); // led
    pinMode(RX_4_LED, OUTPUT); // led
    digitalWrite(RX_1_LED, 1);
    digitalWrite(RX_3_LED, 1);
    digitalWrite(RX_4_LED, 1);
#endif
    pinMode(ENABLE, OUTPUT);
    pinMode(MC_DIN, INPUT);
    digitalWrite(ENABLE, 1);
    pinMode(ON_VOLTAGE_MEAS, OUTPUT);
    pinMode(IN_VOLTAGE, ANALOG);
    pinMode(TERM, ANALOG);
}

void read_last_state(void) {
    EEPROM.begin(EEPROM_SIZE);
    bracelet_mode = EEPROM.read(0);
    if (bracelet_mode == 1) {
        digitalWrite(RX_1_LED, 0);
    } else {
        digitalWrite(RX_1_LED, 1);
        bracelet_mode = 0;
    }
}

void state_machine(void) {
    switch (bracelet_mode) {
        case 0:
            send_data(broadcastAddress_pc);
            break;
        case 1:
            send_data(broadcastAddress_car);
            break;
    }
}

/* calculating percentage of the charge */
uint8_t get_charge(void) {
    digitalWrite(ON_VOLTAGE_MEAS, 1);
    delay(5);
    uint8_t vtg = (analogRead(IN_VOLTAGE) - 2052) / 4.26; // 1981<->2052 - digital value that equal 3.3V !! in denominator: 5.14 <-> 4.26
    digitalWrite(ON_VOLTAGE_MEAS, 0);
    if (vtg > 100) vtg = 100;
    return vtg; 
}

/* periodically sending data about charge status */
void inform_about_charge(void) {
    unsigned long current_ticks = millis();
    if (ground_zeros > current_ticks) {
        current_ticks += (0xFFFFFFFF - ground_zeros);
        ground_zeros = 0;
    }

    if (current_ticks - ground_zeros >= CHARGE_INFO_TIME) {
        charge.val = get_charge();
#ifdef WIFI_OFF_TEST
        WiFi_enable();
#endif
        esp_now_send(broadcastAddress_pc, (uint8_t *)&charge, sizeof(charge));
#ifdef WIFI_OFF_TEST
        WiFi.mode(WIFI_OFF);
#endif
        ground_zeros = millis();
    }
}

void error_clbck(void) {
    while (1) {
        digitalWrite(RX_1_LED, 0);
        digitalWrite(RX_3_LED, 0);
        digitalWrite(RX_4_LED, 0);
        delay(250);
        digitalWrite(RX_1_LED, 1);
        digitalWrite(RX_3_LED, 1);
        digitalWrite(RX_4_LED, 1);
        delay(250);
        if (!digitalRead(WKUP)) esp_deep_sleep_start();
    }
}

#ifdef MPU_ON
void mpu_init(void) {
    delay(1000);
    // pinMode(INT_MPU9250, INPUT);
    int status = IMU.begin();

    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        Serial.println("Check IMU wiring or try cycling power");
        Serial.print("Status: ");
        Serial.println(status);
    } else {
        Serial.print("MPU init status: ");
        Serial.println(status);
        // setting the accelerometer full scale range to +/-8G
        IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
        // setting the gyroscope full scale range to +/-500 deg/s
        IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
        // setting DLPF bandwidth to 20 Hz
        IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
        // setting SRD to 19 for a 50 Hz update rate
        IMU.setSrd(19);
        IMU.setAccelCalX(ACC_X_BIAS, ACC_X_SCALE);
        IMU.setAccelCalY(ACC_X_BIAS, ACC_Y_SCALE);
    }
}

void mpu_data_processing(void) {
    IMU.readSensor();
    float ax = IMU.getAccelX_mss();

#if (DEVICE_ID == 3)
    float ay = IMU.getAccelY_mss() + 4;
#else
    float ay = IMU.getAccelY_mss();
#endif

    if (ax > 9) {
        ax = 9;
    } else if (ax < -9) {
        ax = -9;
    }

    if (ay > 9) {
        ay = 9;
    } else if (ay < -9) {
        ay = -9;
    }

    if (ax < 0) {
        myData.y = -ax;
        myData.x = 1;
    } else {
        myData.y = ax;
        myData.x = 0;
    }

    if (ay < 0) {
        myData.q = -ay;
        myData.t = 1;
    } else {
        myData.q = ay;
        myData.t = 0;
    }
}

void mpu_get_all_data(void) {
    IMU.readSensor();

    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);

    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroZ_rads(), 6);

    Serial.print("\t");
    Serial.print(IMU.getMagX_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagY_uT(), 6);
    Serial.print("\t");
    Serial.print(IMU.getMagZ_uT(), 6);

    Serial.print("\t");
    Serial.println(IMU.getTemperature_C(), 6);
    delay(250);
}

void mpu_get_print_accelerations(void) {
    IMU.readSensor();
    Serial.print(IMU.getAccelX_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelY_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getAccelZ_mss(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroX_rads(), 6);
    Serial.print("\t");
    Serial.print(IMU.getGyroY_rads(), 6);
    Serial.print("\t");
    Serial.println(IMU.getGyroZ_rads(), 6);
    delay(50);
}

void mpu_get_roll_pitch_angles(void) {
    IMU.readSensor();
    float a_x = IMU.getAccelX_mss();
    float a_y = IMU.getAccelY_mss();
    float a_z = IMU.getAccelZ_mss();
    Serial.print(atan(a_y / a_z) * 57.3, 6);
    Serial.print("\t");
    Serial.println((atan(-a_x / pow(a_y*a_y + a_z*a_z, 1/2))) * 57.3, 6);
    delay(50);
}
#endif /* MPU_ON */
