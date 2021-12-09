#pragma once
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/api/custom_api_device.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <list>
#include <map>
#include <queue>
#include <vector>

namespace esphome {
namespace bentel_kyo32 {

static const uint8_t MAX_ZONE = 32;
static const uint8_t MAX_AREE = 8;
static const char *TAG = "bentel_kyo32";

class Bentel_Kyo32 : public PollingComponent, public uart::UARTDevice, public api::CustomAPIDevice {
 public:
  Bentel_Kyo32() : uart::UARTDevice() {}

  using Sensormap = std::map<uint8_t, std::unique_ptr<binary_sensor::BinarySensor>>;

  // BinarySensor *zona = new BinarySensor[MAX_ZONE];
  Sensormap zona;
  void add_zona_sensor(uint8_t zone, const char *name) {
    zona.emplace(std::make_pair(zone, std::unique_ptr<binary_sensor::BinarySensor>(new binary_sensor::BinarySensor(name))));
//    App.register_component(zona[zone].get());
    App.register_binary_sensor(zona[zone].get());
  }
  //  BinarySensor *zona_sabotaggio = new BinarySensor[MAX_ZONE];
  Sensormap zona_sabotaggio;
  void add_zona_sabotaggio_sensor(uint8_t zone, const char *name = nullptr) {
    if (name == nullptr) {
      name = str_sprintf("Zona Sabotaggio %d", zone).c_str();
    }
    zona_sabotaggio.emplace(std::make_pair(zone, std::unique_ptr<binary_sensor::BinarySensor>(new binary_sensor::BinarySensor(name))));
  //  App.register_component(zona_sabotaggio[zone].get());
    App.register_binary_sensor(zona_sabotaggio[zone].get());
  }
  // BinarySensor *zona_esclusa = new BinarySensor[MAX_ZONE];
  Sensormap zona_esclusa;

  // BinarySensor *memoria_allarme_zona = new BinarySensor[MAX_ZONE];
  Sensormap memoria_allarme_zona;

  // BinarySensor *memoria_sabotaggio_zona = new BinarySensor[MAX_ZONE];
  Sensormap memoria_sabotaggio_zona;

  //  BinarySensor *allarme_area = new BinarySensor[MAX_AREE];
  Sensormap allarme_area;

  //  BinarySensor *inserimento_totale_area = new BinarySensor[MAX_AREE];
  Sensormap inserimento_totale_area;

  // BinarySensor *inserimento_parziale_area = new BinarySensor[MAX_AREE];
  Sensormap inserimento_parziale_area;

  // BinarySensor *inserimento_parziale_ritardo_0_area = new BinarySensor[MAX_AREE];
  Sensormap inserimento_parziale_ritardo_0_area;

  // BinarySensor *disinserita_area = new BinarySensor[MAX_AREE];
  Sensormap disinserita_area;

  // BinarySensor *stato_uscita = new BinarySensor[MAX_AREE];
  Sensormap stato_uscita;

  binary_sensor::BinarySensor *kyo_comunication{nullptr};
  void set_communication_sensor(binary_sensor::BinarySensor *sensor) { this->kyo_comunication = sensor; }
  // TODO - use the same pattern as above for these settings
  binary_sensor::BinarySensor *warn_mancanza_rete = new binary_sensor::BinarySensor("Warning Mancanza Rete");
  binary_sensor::BinarySensor *warn_scomparsa_bpi = new binary_sensor::BinarySensor("Warning Scomparsa BPI");
  binary_sensor::BinarySensor *warn_fusibile = new binary_sensor::BinarySensor("Warning Fusibile");
  binary_sensor::BinarySensor *warn_batteria_bassa = new binary_sensor::BinarySensor("Warning Batteria Bassa");
  binary_sensor::BinarySensor *warn_guasto_linea_telefonica = new binary_sensor::BinarySensor("Warning Guasto Linea Telefonica");
  binary_sensor::BinarySensor *warn_codici_default = new binary_sensor::BinarySensor("Warning Codici Default");
  binary_sensor::BinarySensor *warn_wireless = new binary_sensor::BinarySensor("Warning Wireless");

  binary_sensor::BinarySensor *stato_sirena = new binary_sensor::BinarySensor("stato sirena");
  binary_sensor::BinarySensor *sabotaggio_zona = new binary_sensor::BinarySensor("Sabotaggio di Zona");
  binary_sensor::BinarySensor *sabotaggio_chiave_falsa = new binary_sensor::BinarySensor("Sabotaggio Chiave Falsa");
  binary_sensor::BinarySensor *sabotaggio_bpi = new binary_sensor::BinarySensor("Sabotaggio BPI");
  binary_sensor::BinarySensor *sabotaggio_sistema = new binary_sensor::BinarySensor("Sabotaggio Sistema");
  binary_sensor::BinarySensor *sabotaggio_jam = new binary_sensor::BinarySensor("Sabotaggio JAM");
  binary_sensor::BinarySensor *sabotaggio_wireless = new binary_sensor::BinarySensor("Sabotaggio Wireless");

  uint8_t cmdGetSensorStatus[6] = {0xf0, 0x04, 0xf0, 0x0a, 0x00, 0xee};  // Read Realtime Status and Trouble Status
  uint8_t cmdGetPartitionStatus[6] = {0xf0, 0x02, 0x15,
                                      0x12, 0x00, 0x19};  // Partitions Status (305) - Outputs Status - Tamper Memory -
                                                          // Bypassed Zones - Zone Alarm Memory - Zone Tamper Memory
  uint8_t cmqGetSoftwareVersion[6] = {0xf0, 0x00, 0x00, 0x0b, 0x00, 0xfb};  // f0 00 00 0b 00 fb
  uint8_t cmdResetAllarms[9] = {0x0F, 0x05, 0xF0, 0x01, 0x00, 0x05, 0xFF, 0x00, 0xFF};

  enum class PollingStateEnum { Init = 1, Status };

  bool serialTrace = false;
  bool logTrace = false;
  PollingStateEnum pollingState;
  int centralInvalidMessageCount = 0;

  int sendMessageToKyo(uint8_t *cmd, int lcmd, uint8_t ReadByes[], int waitForAnswer = 0) {
    // clean rx buffer
    while (available() > 0)
      read();

    delay(10);

    int index = 0;
    uint8_t RxBuff[255];
    memset(ReadByes, 0, 254);

    write_array(cmd, lcmd);
    delay(waitForAnswer);

    // Read a single Byte
    while (available() > 0)
      RxBuff[index++] = read();

    if (this->serialTrace) {
      int i;
      char txString[255];
      char rxString[255];
      memset(txString, 0, 255);
      memset(rxString, 0, 255);

      for (i = 0; i < lcmd; i++)
        sprintf(txString, "%s %02X", txString, cmd[i]);

      for (i = 0; i < index; i++)
        sprintf(rxString, "%s %02X", rxString, RxBuff[i]);

      ESP_LOGD("sendMessageToKyo", "TX [%d] '%s', RX [%d] '%s'", lcmd, txString, index, rxString);
    }

    if (index <= 0)
      return -1;

    memcpy(ReadByes, RxBuff, index);
    return index;
  }

  void setup() override {
    register_service(&Bentel_Kyo32::arm_area, "arm_area", {"area", "arm_type", "specific_area"});
    register_service(&Bentel_Kyo32::disarm_area, "disarm_area", {"area", "specific_area"});
    register_service(&Bentel_Kyo32::reset_alarms, "reset_alarms");
    register_service(&Bentel_Kyo32::activate_output, "activate_output", {"output_number"});
    register_service(&Bentel_Kyo32::deactivate_output, "deactivate_output", {"output_number"});
    register_service(&Bentel_Kyo32::debug_command, "debug_command", {"serial_trace", "log_trace"});

    // register_service(&Bentel_Kyo32::on_clock_setting, "clock_setting", {"pin", "day", "month", "year", "hour",
    // "minutes", "seconds", "data_format"}); register_service(&Bentel_Kyo32::on_bypass_zone, "bypass_zone", {"pin",
    // "zone_number"}); register_service(&Bentel_Kyo32::on_unbypass_zone, "unbypass_zone", {"pin", "zone_number"});

    this->pollingState = PollingStateEnum::Init;
    if (this->kyo_comunication)
      this->kyo_comunication->publish_state(false);
    this->set_update_interval(500);
  }

  uint8_t calculateCRC(uint8_t *cmd, int lcmd) {
    int sum = 0x00;
    for (int i = 0; i <= lcmd; i++)
      sum += cmd[i];

    return (0x203 - sum);
  }

  void arm_area(int area, int arm_type, int specific_area) {
    if (area > MAX_AREE) {
      ESP_LOGD("arm_area", "Invalid Area %i, MAX %i", area, MAX_AREE);
      return;
    }

    ESP_LOGD("arm_area", "request arm type %d area %d", arm_type, area);
    uint8_t cmdArmPartition[11] = {0x0F, 0x00, 0xF0, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0xCC, 0xFF};

    uint8_t total_insert_area_status = 0x00, partial_insert_area_status = 0x00;

    if (specific_area == 1) {
      for (int i = 0; i < MAX_AREE; i++) {
        total_insert_area_status |= (this->inserimento_totale_area[i]->state) << i;
        partial_insert_area_status |= (this->inserimento_parziale_area[i]->state) << i;
      }
    }

    if (arm_type == 2)
      partial_insert_area_status |= 1 << (area - 1);
    else
      total_insert_area_status |= 1 << (area - 1);

    cmdArmPartition[6] = total_insert_area_status;
    cmdArmPartition[7] = partial_insert_area_status;
    cmdArmPartition[9] = calculateCRC(cmdArmPartition, 8);

    uint8_t Rx[255];
    int Count = sendMessageToKyo(cmdArmPartition, sizeof(cmdArmPartition), Rx, 100);
    ESP_LOGD("arm_area", "arm_area kyo respond %i", Count);
  }

  void disarm_area(int area, int specific_area) {
    if (area > MAX_AREE) {
      ESP_LOGD("arm_area", "invalid Area %i, MAX %i", area, MAX_AREE);
      return;
    }

    ESP_LOGD("disarm_area", "request disarm area %d", area);
    uint8_t cmdDisarmPartition[11] = {0x0F, 0x00, 0xF0, 0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xFF};

    uint8_t total_insert_area_status = 0x00, partial_insert_area_status = 0x00;
    // TODO 
    // Because this is now a map and not a hardocded array add check if the area was created otherwise code may crash
    //  
    if (specific_area == 1) {
      for (int i = 0; i < MAX_AREE; i++) {
        total_insert_area_status |= (this->inserimento_totale_area[i]->state) << i;
        partial_insert_area_status |= (this->inserimento_parziale_area[i]->state) << i;
      }

      if (this->inserimento_totale_area[area - 1]->state)
        total_insert_area_status |= 0 << (area - 1);
      else
        partial_insert_area_status |= 1 << (area - 1);
    }

    cmdDisarmPartition[6] = total_insert_area_status;
    cmdDisarmPartition[7] = partial_insert_area_status;
    cmdDisarmPartition[9] = calculateCRC(cmdDisarmPartition, 8);

    uint8_t Rx[255];
    int Count = sendMessageToKyo(cmdDisarmPartition, sizeof(cmdDisarmPartition), Rx, 80);
    ESP_LOGD("disarm_area", "kyo respond %i", Count);
  }

  void reset_alarms() {
    ESP_LOGD("reset_alarms", "Reset Alarms.");

    uint8_t Rx[255];
    int Count = sendMessageToKyo(cmdResetAllarms, sizeof(cmdResetAllarms), Rx, 80);
    ESP_LOGD("reset_alarms", "kyo respond %i", Count);
  }

  void debug_command(int serial_trace, int log_trace) {
    this->serialTrace = (serial_trace == 1);
    this->logTrace = (log_trace == 1);

    ESP_LOGD("debug_command", "serial_trace %i log_trace %i", this->serialTrace, this->logTrace);
  }

  void loop() override {}

  void activate_output(int output_number) {
    if (output_number > MAX_AREE) {
      ESP_LOGD("activate_output", "invalid output %i, MAX %i", output_number, MAX_AREE);
      return;
    }

    ESP_LOGD("activate_output", "activate Output Number: %d", output_number);

    uint8_t cmdActivateOutput[9] = {0x0f, 0x06, 0xf0, 0x01, 0x00, 0x06, 0x00, 0x00, 0x00};

    cmdActivateOutput[6] |= 1 << (output_number - 1);
    cmdActivateOutput[8] = cmdActivateOutput[6];

    uint8_t Rx[255];
    int Count = sendMessageToKyo(cmdActivateOutput, sizeof(cmdActivateOutput), Rx, 80);
    ESP_LOGD("activate_output", "kyo respond %i", Count);
  }

  void deactivate_output(int output_number) {
    if (output_number > MAX_AREE) {
      ESP_LOGD("deactivate_output", "invalid output %i, MAX %i", output_number, MAX_AREE);
      return;
    }

    ESP_LOGD("deactivate_output", "deactivate Output Number: %d", output_number);

    uint8_t cmdDeactivateOutput[9] = {0x0f, 0x06, 0xf0, 0x01, 0x00, 0x06, 0x00, 0x00, 0xCC};

    cmdDeactivateOutput[7] |= 1 << (output_number - 1);
    cmdDeactivateOutput[8] = cmdDeactivateOutput[7];

    uint8_t Rx[255];
    int Count = sendMessageToKyo(cmdDeactivateOutput, sizeof(cmdDeactivateOutput), Rx, 80);
    ESP_LOGD("deactivate_output", "kyo respond %i", Count);
  }

  /*
  void on_bypass_zone(int pin, int zone_number)
  {
    ESP_LOGD("custom", "Bypass Zone. PIN: %d, Zone Number: %d", pin, zone_number);
  }

  void on_unbypass_zone(int pin, int zone_number)
  {
    ESP_LOGD("custom", "UnBypass Zone. PIN: %d, Zone Number: %d", pin, zone_number);
  }

  void on_clock_setting(int pin, int day, int month, int year, int hour, int minutes, int seconds, int data_format)
  {
    ESP_LOGD("custom", "Clock Setting. PIN: %d, Day: %d, Month: %d, Year: %d, Hour: %d, Minutes: %d, Seconds: %d, Data
  Format: %d", pin, day, month, year, hour, minutes, seconds, data_format);
  }
  */

  void update() override {
    switch (this->pollingState) {
      case PollingStateEnum::Init:
        if (this->update_kyo_status()) {
          this->pollingState = PollingStateEnum::Status;
          this->centralInvalidMessageCount = 0;
        } else {
          this->centralInvalidMessageCount++;
        }

        break;

      case PollingStateEnum::Status:
        if (this->update_kyo_partitions()) {
          this->pollingState = PollingStateEnum::Init;
          this->centralInvalidMessageCount = 0;
        } else
          this->centralInvalidMessageCount++;

        break;
    }

    if (kyo_comunication) {
      if (this->centralInvalidMessageCount == 0 && !this->kyo_comunication->state)
        this->kyo_comunication->publish_state(true);
      else if (centralInvalidMessageCount > 3)
        this->kyo_comunication->publish_state(false);
    }
  }

  bool update_kyo_partitions() {
    uint8_t Rx[255];
    int Count = 0;

    Count = sendMessageToKyo(cmdGetPartitionStatus, sizeof(cmdGetPartitionStatus), Rx, 100);
    if (Count != 26) {
      if (this->logTrace)
        ESP_LOGD("update_kyo_partitions", "invalid message length %i", Count);

      return (false);
    }

    int StatoZona;

    // Ciclo AREE INSERITE
    for (auto &iter : inserimento_totale_area) {
      StatoZona = (Rx[6] >> iter.first) & 1;
      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "Area %d - Stato %d", iter.first, StatoZona);
      iter.second->publish_state(StatoZona == 1);
    }

    // Ciclo AREE INSERITE PARZIALI
    for (auto &iter : inserimento_parziale_area) {
      StatoZona = (Rx[7] >> iter.first) & 1;
      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "aree_parziale Area %d - Stato %d", iter.first, StatoZona);
      iter.second->publish_state(StatoZona == 1);
    }

    // Ciclo AREE INSERITE PARZIALI RITARDO 0
    for (auto &iter : inserimento_parziale_ritardo_0_area) {
      StatoZona = (Rx[8] >> iter.first) & 1;
      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "aree_parzial _ritardo_0:  Area %d - Stato %d", iter.first, StatoZona);
      iter.second->publish_state(StatoZona == 1);
    }

    // Ciclo AREE DISINSERITE
    for (auto &iter : disinserita_area) {
      StatoZona = (Rx[9] >> iter.first) & 1;
      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "AREE DISINSERITE:  Area %d - Stato %d", iter.first, StatoZona);
      iter.second->publish_state(StatoZona == 1);
    }

    // STATO SIRENA
    stato_sirena->publish_state(((Rx[10] >> 5) & 1) == 1);

    // CICLO STATO USCITE
    for (auto &iter : stato_uscita) {
      uint8_t i = iter.first;
      StatoZona = 0;
      if (i >= 8 && i <= 15)
        StatoZona = (Rx[11] >> (i - 8)) & 1;
      else if (i <= 7)
        StatoZona = (Rx[12] >> i) & 1;

      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "stato_uscita Uscita %d - Stato %d", iter.first, StatoZona);
      iter.second->publish_state(StatoZona == 1);
    }

    // CICLO ZONE ESCLUSE
    for (auto &iter : zona_esclusa) {
      uint8_t i = iter.first;
      StatoZona = 0;
      if (i >= 24)
        StatoZona = (Rx[13] >> (i - 24)) & 1;
      else if (i >= 16 && i <= 23)
        StatoZona = (Rx[14] >> (i - 16)) & 1;
      else if (i >= 8 && i <= 15)
        StatoZona = (Rx[15] >> (i - 8)) & 1;
      else if (i <= 7)
        StatoZona = (Rx[16] >> i) & 1;

      iter.second->publish_state(StatoZona == 1);
    }

    // CICLO MEMORIA ALLARME ZONE
    for (auto &iter : memoria_allarme_zona) {
      uint8_t i = iter.first;
      StatoZona = 0;
      if (i >= 24)
        StatoZona = (Rx[17] >> (i - 24)) & 1;
      else if (i >= 16 && i <= 23)
        StatoZona = (Rx[18] >> (i - 16)) & 1;
      else if (i >= 8 && i <= 15)
        StatoZona = (Rx[19] >> (i - 8)) & 1;
      else if (i <= 7)
        StatoZona = (Rx[20] >> i) & 1;

      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "memoria_allarme_zona : Zona %d - Stato %d", i, StatoZona);

      iter.second->publish_state(StatoZona == 1);
    }

    // CICLO MEMORIA SABOTAGGIO ZONE
    for (auto &iter : memoria_sabotaggio_zona) {
      uint8_t i = iter.first;
      StatoZona = 0;
      if (i >= 24)
        StatoZona = (Rx[21] >> (i - 24)) & 1;
      else if (i >= 16 && i <= 23)
        StatoZona = (Rx[22] >> (i - 16)) & 1;
      else if (i >= 8 && i <= 15)
        StatoZona = (Rx[23] >> (i - 8)) & 1;
      else if (i <= 7)
        StatoZona = (Rx[24] >> i) & 1;

      if (this->logTrace && (StatoZona == 1) != iter.second->state)
        ESP_LOGD(TAG, "memoria_sabotaggio_zona Zona %d - Stato %d", i, StatoZona);

      iter.second->publish_state(StatoZona == 1);
    }

    return true;
  }

  bool update_kyo_status() {
    uint8_t Rx[255];
    int Count = 0;

    Count = sendMessageToKyo(cmdGetSensorStatus, sizeof(cmdGetSensorStatus), Rx, 100);
    if (Count != 18) {
      if (this->logTrace)
        ESP_LOGD("update_kyo_status", "invalid message length %i", Count);

      return false;
    }

    int StatoZona, i;

    // Ciclo ZONE
    for (auto &iter : zona) {
      uint8_t i = iter.first;
      StatoZona = 0;
      if (i >= 24)
        StatoZona = (Rx[6] >> (i - 24)) & 1;
      else if (i >= 16 && i <= 23)
        StatoZona = (Rx[7] >> (i - 16)) & 1;
      else if (i >= 8 && i <= 15)
        StatoZona = (Rx[8] >> (i - 8)) & 1;
      else if (i <= 7)
        StatoZona = (Rx[9] >> i) & 1;

      ESP_LOGD(TAG, "Zona - The value of sensor is: %d", StatoZona);
      iter.second->publish_state(StatoZona == 1);
    }

    // Ciclo SABOTAGGIO ZONE
    for (auto &iter : zona_sabotaggio) {
      uint8_t i = iter.first;
      StatoZona = 0;
      if (i >= 24)
        StatoZona = (Rx[10] >> (i - 24)) & 1;
      else if (i >= 16 && i <= 23)
        StatoZona = (Rx[11] >> (i - 16)) & 1;
      else if (i >= 8 && i <= 15)
        StatoZona = (Rx[12] >> (i - 8)) & 1;
      else if (i <= 7)
        StatoZona = (Rx[13] >> i) & 1;

      iter.second->publish_state(StatoZona == 1);
    }

    // Ciclo ALLARME AREA
    for (auto &iter : allarme_area) {
      StatoZona = (Rx[15] >> iter.first) & 1;
      iter.second->publish_state(StatoZona == 1);
    }

    // Ciclo WARNINGS
    for (i = 0; i < 8; i++) {
      StatoZona = (Rx[14] >> i) & 1;
      switch (i) {
        case 0:
          warn_mancanza_rete->publish_state(StatoZona == 1);
          break;

        case 1:
          warn_scomparsa_bpi->publish_state(StatoZona == 1);
          break;

        case 2:
          warn_fusibile->publish_state(StatoZona == 1);
          break;

        case 3:
          warn_batteria_bassa->publish_state(StatoZona == 1);
          break;

        case 4:
          warn_guasto_linea_telefonica->publish_state(StatoZona == 1);
          break;

        case 5:
          warn_codici_default->publish_state(StatoZona == 1);
          break;

        case 6:
          warn_wireless->publish_state(StatoZona == 1);
          break;
      }
    }

    // Ciclo SABOTAGGI
    for (i = 0; i < 8; i++) {
      StatoZona = (Rx[16] >> i) & 1;
      switch (i) {
        case 2:
          sabotaggio_zona->publish_state(StatoZona == 1);
          break;

        case 3:
          sabotaggio_chiave_falsa->publish_state(StatoZona == 1);
          break;

        case 4:
          sabotaggio_bpi->publish_state(StatoZona == 1);
          break;

        case 5:
          sabotaggio_sistema->publish_state(StatoZona == 1);
          break;

        case 6:
          sabotaggio_jam->publish_state(StatoZona == 1);
          break;

        case 7:
          sabotaggio_wireless->publish_state(StatoZona == 1);
          break;
      }
    }

    return true;
  }
};
}  // namespace bentel_kyo32
}  // namespace esphome
