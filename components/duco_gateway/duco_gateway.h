#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#if defined(USE_ESP32)
#include <driver/gpio.h>
#endif

namespace esphome {
namespace duco_gateway {

// ─────────────────────────────────────────────────────────────────────────────
// Ventilation mode helpers
// ─────────────────────────────────────────────────────────────────────────────
static const char *vent_mode_to_str(int mode) {
  switch (mode) {
    case 0:  return "AUTO";
    case 1:  return "MAN1";
    case 2:  return "MAN2";
    case 3:  return "MAN3";
    case 4:  return "EMPT";
    case 5:  return "ALRM";
    case 11: return "CNT1";
    case 12: return "CNT2";
    case 13: return "CNT3";
    case 20: return "AUT0";
    case 21: return "AUT1";
    case 22: return "AUT2";
    case 23: return "AUT3";
    default: return "UNKN";
  }
}

// Map stat string to number (matches DucoStatusModes in P151)
static int vent_mode_from_str(const char *s) {
  const char *names[]   = {"AUTO","MAN1","MAN2","MAN3","EMPT","ALRM","CNT1","CNT2","CNT3","AUT0","AUT1","AUT2","AUT3"};
  const int   numbers[] = {0,1,2,3,4,5,11,12,13,20,21,22,23};
  for (int i = 0; i < 13; i++) {
    if (strstr(s, names[i]) != nullptr) return numbers[i];
  }
  return -1;
}

// ─────────────────────────────────────────────────────────────────────────────
// Phase state machine
//
// IDLE → LOGLEVEL → NETWORK → SENSORINFO → NODEPARA_CO2 → NODEPARA_TEMP →
//        NODEPARA_RH → FANSPEED → IDLE
//
// Phases that have no sensor configured are skipped automatically.
// ─────────────────────────────────────────────────────────────────────────────
enum SerialPhase {
  PH_IDLE,
  PH_LOGLEVEL,      // coreloglevel 0x10
  PH_NETWORK,       // Network  → tabel met stat / %dbt / cval / cntdwn
  PH_SENSORINFO,    // sensorinfo → CO2 / RH / TEMP (boxsensor)
  PH_NODEPARA_CO2,  // nodeparaget <ext_node> 74  → CO2 ppm
  PH_NODEPARA_TEMP, // nodeparaget <ext_node> 73  → temp /10
  PH_NODEPARA_RH,   // nodeparaget <ext_node> 75  → RH /100
  PH_CUSTOM,        // command entered from HA text entity
  PH_FANSPEED,      // fanspeed → RPM
};

// ─────────────────────────────────────────────────────────────────────────────
class DucoGateway : public Component, public uart::UARTDevice {
 public:
  // ── Sensor setters ────────────────────────────────────────────────────────
  void set_serial_switch_pin(uint8_t pin)                    { serial_switch_pin_ = pin; serial_switch_configured_ = true; }
  void set_ext_sensor_node(uint8_t n)                        { ext_node_ = n; }
  void set_rh_sensor_node(uint8_t n)                         { rh_node_  = n; }
  void set_vent_mode_sensor(sensor::Sensor *s)               { vent_mode_ = s; }
  void set_vent_percentage_sensor(sensor::Sensor *s)         { vent_pct_ = s; }
  void set_fan_speed_sensor(sensor::Sensor *s)               { fan_speed_ = s; }
  void set_countdown_sensor(sensor::Sensor *s)               { countdown_ = s; }
  void set_box_temperature_sensor(sensor::Sensor *s)         { box_temp_ = s; }
  void set_box_humidity_sensor(sensor::Sensor *s)            { box_rh_ = s; }
  void set_ext_co2_sensor(sensor::Sensor *s)                 { ext_co2_ = s; }
  void set_ext_temperature_sensor(sensor::Sensor *s)         { ext_temp_ = s; }
  void set_ext_humidity_sensor(sensor::Sensor *s)            { ext_rh_   = s; }
  void set_vent_mode_text_sensor(text_sensor::TextSensor *s) { vent_mode_txt_ = s; }
  void set_poll_interval_ms(uint32_t ms)                     { poll_interval_ms_ = ms; }

  // Request an immediate network dump with all discovered nodes in logs.
  void log_all_nodes() {
    log_all_nodes_requested_ = true;
  }

  // Queue a custom command to be sent when the gateway is idle.
  void send_custom_command(const std::string &command) {
    if (phase_ != PH_IDLE || custom_command_requested_) {
      ESP_LOGW(TAG, "Cannot queue custom command while busy (phase %d)", (int) phase_);
      return;
    }

    size_t start = command.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) {
      ESP_LOGW(TAG, "Ignoring empty custom command");
      return;
    }

    size_t end = command.find_last_not_of(" \t\r\n");
    std::string trimmed = command.substr(start, end - start + 1);

    if (trimmed.size() >= sizeof(custom_command_)) {
      ESP_LOGW(TAG, "Custom command too long (%u >= %u)", (unsigned) trimmed.size(), (unsigned) sizeof(custom_command_));
      return;
    }

    strncpy(custom_command_, trimmed.c_str(), sizeof(custom_command_) - 1);
    custom_command_[sizeof(custom_command_) - 1] = '\0';
    custom_command_requested_ = true;
    ESP_LOGI(TAG, "Queued custom command: %s", custom_command_);
  }

  // ── ESPHome lifecycle ─────────────────────────────────────────────────────
  float get_setup_priority() const override { return setup_priority::DATA; }

  void setup() override {
    ESP_LOGI(TAG, "DucoGateway setup()");

#if defined(USE_ESP32)
    if (serial_switch_configured_) {
      // HIGH = ESP UART connected to Ducobox (normal operation)
      // LOW  = external UART/USB connected to Ducobox (service mode)
      auto gpio = static_cast<gpio_num_t>(serial_switch_pin_);
      gpio_reset_pin(gpio);
      gpio_set_direction(gpio, GPIO_MODE_OUTPUT);
      gpio_set_level(gpio, 1);
      ESP_LOGI(TAG, "Serial switch GPIO%u = HIGH (ESP UART -> Ducobox)", serial_switch_pin_);
    } else {
      ESP_LOGI(TAG, "No serial switch pin configured; skipping UART mux control");
    }
#else
    ESP_LOGE(TAG, "duco_gateway requires ESP32 platform");
#endif

    delay(200);
    flush_rx();
    setup_done_ = true;
  }

  void loop() override {
    // First loop() call: send coreloglevel here (not in setup) so rx_pump() runs.
    if (setup_done_ && !loglevel_sent_) {
      loglevel_sent_ = true;
      ESP_LOGD(TAG, "Sending coreloglevel 0x10");
      send_command("coreloglevel 0x10\r\n", PH_LOGLEVEL);
      return;
    }

    rx_pump();

    if (phase_ == PH_IDLE) {
      if (custom_command_requested_) {
        custom_command_requested_ = false;
        char cmd[136];
        snprintf(cmd, sizeof(cmd), "%s\r\n", custom_command_);
        send_command(cmd, PH_CUSTOM);
        return;
      }

      if (log_all_nodes_requested_) {
        log_all_nodes_requested_ = false;
        log_all_nodes_active_ = true;
        send_command("Network\r\n", PH_NETWORK);
        return;
      }

      if ((millis() - last_poll_ms_) >= poll_interval_ms_) {
        last_poll_ms_ = millis();
        log_all_nodes_active_ = false;
        send_command("Network\r\n", PH_NETWORK);
      }
    }
  }

 protected:
  static constexpr const char *TAG               = "duco_serial";
  static constexpr uint32_t    CMD_TIMEOUT_MS    = 8000;   // Should be enough even for slow responses, but short enough to recover from errors. Ducobox is usually very fast (responds in <100ms) but can be slow when the network table grows large or communication is bad with one of the nodes. Duco fails the command after roughly 5 seconds, but we set a slightly longer timeout here to also cover the case where we miss the "Failed" response.

  uint8_t serial_switch_pin_ {0};
  bool    serial_switch_configured_ {false};

  SerialPhase phase_        {PH_IDLE};
  uint32_t    cmd_sent_ms_  {0};
  uint32_t    last_rx_ms_   {0};
  uint32_t    last_poll_ms_ {0};
  bool        got_done_     {false};
  bool        had_rx_       {false};
  bool        setup_done_   {false};
  bool        loglevel_sent_{false};
  bool        log_all_nodes_requested_{false};
  bool        log_all_nodes_active_{false};
  bool        nodeparaget_in_flight_{false};
  bool        phase_timed_out_{false};
  bool        custom_command_requested_{false};
  uint32_t    poll_interval_ms_{15000};
  char        custom_command_[128] {0};

  // Network table column indices (resolved from header row)
  uint8_t col_stat_   {255};
  uint8_t col_dbt_    {255};
  uint8_t col_cval_   {255};
  uint8_t col_cntdwn_ {255};

  char     line_buf_[256];
  uint16_t line_pos_ {0};

  uint8_t ext_node_ {3};
  uint8_t rh_node_  {4};

  sensor::Sensor          *vent_mode_    {nullptr};
  sensor::Sensor          *vent_pct_     {nullptr};
  sensor::Sensor          *fan_speed_    {nullptr};
  sensor::Sensor          *countdown_    {nullptr};
  sensor::Sensor          *box_temp_     {nullptr};
  sensor::Sensor          *box_rh_       {nullptr};
  sensor::Sensor          *ext_co2_      {nullptr};
  sensor::Sensor          *ext_temp_     {nullptr};
  sensor::Sensor          *ext_rh_       {nullptr};
  text_sensor::TextSensor *vent_mode_txt_{nullptr};

  // ── Serial helpers ────────────────────────────────────────────────────────
  bool is_nodeparaget_phase_(SerialPhase p) const {
    return p == PH_NODEPARA_CO2 || p == PH_NODEPARA_TEMP || p == PH_NODEPARA_RH;
  }

  void flush_rx() {
    while (available())
      read();
  }

  void send_command(const char *cmd, SerialPhase next_phase) {
    if (is_nodeparaget_phase_(next_phase) && nodeparaget_in_flight_) {
      ESP_LOGW(TAG, "Blocked overlapping nodeparaget command in phase %d", (int) next_phase);
      return;
    }

    flush_rx();
    line_pos_    = 0;
    got_done_    = false;
    had_rx_      = false;
    phase_timed_out_ = false;
    phase_       = next_phase;
    cmd_sent_ms_ = millis();
    last_rx_ms_  = millis();
    // Send byte-by-byte with small delay — Ducobox RX buffer is very small
    for (const char *p = cmd; *p; p++) {
      write_byte((uint8_t)*p);
      delay(5);
    }
    char log_cmd[64];
    strncpy(log_cmd, cmd, sizeof(log_cmd) - 1);
    log_cmd[sizeof(log_cmd)-1] = '\0';
    for (char *p = log_cmd; *p; p++) if (*p=='\r'||*p=='\n') { *p='\0'; break; }
    if (is_nodeparaget_phase_(next_phase)) {
      nodeparaget_in_flight_ = true;
    }
    ESP_LOGD(TAG, "TX >> \"%s\"  (phase %d)", log_cmd, (int)next_phase);
  }

  void rx_pump() {
    if (phase_ == PH_IDLE) return;

    if ((millis() - cmd_sent_ms_) > CMD_TIMEOUT_MS) {
      ESP_LOGW(TAG, "Timeout in phase %d (had_rx=%d)", (int)phase_, (int)had_rx_);
      phase_timed_out_ = true;
      advance_phase();
      return;
    }

    while (available()) {
      char c = (char)read();
      had_rx_     = true;
      last_rx_ms_ = millis();

      // Ducobox uses \r as line terminator (sometimes \r\n, sometimes just \r)
      if (c == '\r' || c == '\n') {
        if (line_pos_ > 0) {
          line_buf_[line_pos_] = '\0';
          process_line(line_buf_);
          line_pos_ = 0;
        }
      } else {
        if (line_pos_ < (sizeof(line_buf_) - 1))
          line_buf_[line_pos_++] = c;
      }
    }

    if (got_done_) advance_phase();
  }

  // ── Line parser ───────────────────────────────────────────────────────────
  void process_line(const char *line) {
    ESP_LOGD(TAG, "RX << \"%s\"", line);

    if (phase_ == PH_CUSTOM) {
      if (strstr(line, "Done") != nullptr) {
        ESP_LOGI(TAG, "CMD << Done");
        got_done_ = true;
        return;
      }
      if (strstr(line, "Failed") != nullptr) {
        ESP_LOGW(TAG, "CMD << Failed");
        got_done_ = true;
        return;
      }

      ESP_LOGI(TAG, "CMD << %s", line);
      return;
    }

    if (strstr(line, "Done") != nullptr) { got_done_ = true; return; }
    if (strstr(line, "Failed") != nullptr) {
      ESP_LOGW(TAG, "Command failed in phase %d", (int) phase_);
      got_done_ = true;
      return;
    }
    if (phase_ == PH_LOGLEVEL) { return; }

    // ── Network table ─────────────────────────────────────────────────────
    if (phase_ == PH_NETWORK) {
      if (strstr(line, "--- end list ---") != nullptr) {
        got_done_ = true;
        return;
      }

      // Header row looks like: "Node| Zone| ...| stat| ...| %dbt| ...| cval| ...| cntdwn|..."
      // Data row for ducobox (node 1): "   1|  --|  --|AUTO|  --|  25|  --|  30|  --|   0|..."
      if (strstr(line, "stat") != nullptr && strstr(line, "cval") != nullptr) {
        // This is the column header row — find column indices
        parse_network_header(line);
        if (log_all_nodes_active_) {
          ESP_LOGI(TAG, "Header: %s", line);
        }
        return;
      }
      // Node data rows
      int node = -1;
      if (sscanf(line, " %d|", &node) == 1) {
        if (log_all_nodes_active_) {
          ESP_LOGI(TAG, "Node %d: %s", node, line);
        }
        // Keep existing behavior: only node 1 updates box-level sensors
        if (node == 1) {
          parse_network_row(line);
        }
      }
      return;
    }

    // ── sensorinfo ────────────────────────────────────────────────────────
    if (phase_ == PH_SENSORINFO) {
      unsigned int raw = 0;
      // "  CO2 :  627 [ppm]" or "  CO2: 627 [ppm]"
      if (strncmp(line, "  CO", 4) == 0) {
        const char *colon = strchr(line, ':');
        if (colon && sscanf(colon+1, " %u", &raw) == 1) {
          ESP_LOGD(TAG, "sensorinfo CO2=%u ppm", raw);
          // box sensor CO2 — not used separately; ext_co2_ is for external node
        }
      } else if (strncmp(line, "  RH", 4) == 0) {
        const char *colon = strchr(line, ':');
        if (colon && sscanf(colon+1, " %u", &raw) == 1) {
          float rh = (float)raw / 100.0f;
          ESP_LOGD(TAG, "sensorinfo RH raw=%u -> %.2f%%", raw, rh);
          if (box_rh_) box_rh_->publish_state(rh);
        }
      } else if (strncmp(line, "  TEMP", 6) == 0) {
        const char *colon = strchr(line, ':');
        if (colon && sscanf(colon+1, " %u", &raw) == 1) {
          float temp = (float)raw / 10.0f;
          ESP_LOGD(TAG, "sensorinfo TEMP raw=%u -> %.1f C", raw, temp);
          if (box_temp_) box_temp_->publish_state(temp);
        }
      }
      return;
    }

    // ── nodeparaget response: "  --> 492" ─────────────────────────────────
    if (phase_ == PH_NODEPARA_CO2 || phase_ == PH_NODEPARA_TEMP || phase_ == PH_NODEPARA_RH) {
      unsigned int raw = 0;
      if (sscanf(line, "  --> %u", &raw) == 1) {
        if (phase_ == PH_NODEPARA_CO2) {
          ESP_LOGD(TAG, "ext CO2=%u ppm", raw);
          if (ext_co2_) ext_co2_->publish_state((float)raw);
        } else if (phase_ == PH_NODEPARA_TEMP) {
          float temp = (float)raw / 10.0f;
          ESP_LOGD(TAG, "ext TEMP raw=%u -> %.1f C", raw, temp);
          if (ext_temp_) ext_temp_->publish_state(temp);
        } else if (phase_ == PH_NODEPARA_RH) {
          float rh = (float)raw / 100.0f;
          ESP_LOGD(TAG, "ext RH raw=%u -> %.2f%%", raw, rh);
          if (ext_rh_) ext_rh_->publish_state(rh);
        }
      }
      return;
    }

    // ── fanspeed ──────────────────────────────────────────────────────────
    if (phase_ == PH_FANSPEED) {
      unsigned int filtered = 0;
      // "  FanSpeed: Actual 389 [rpm] - Filtered 391 [rpm]"
      if (sscanf(line, "  FanSpeed: Actual %*u %*s %*s %*s %u", &filtered) == 1) {
        ESP_LOGD(TAG, "FanSpeed filtered=%u rpm", filtered);
        if (fan_speed_) fan_speed_->publish_state((float)filtered);
      }
      return;
    }
  }

  // ── Network header parser ─────────────────────────────────────────────────
  // Finds column positions of stat / %dbt / cval / cntdwn in the pipe-delimited header
  void parse_network_header(const char *line) {
    col_stat_ = col_dbt_ = col_cval_ = col_cntdwn_ = 255;
    char buf[256];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    uint8_t col = 0;
    char *tok = strtok(buf, "|");
    while (tok != nullptr) {
      if (strstr(tok, "stat"))   col_stat_   = col;
      if (strstr(tok, "%dbt"))   col_dbt_    = col;
      if (strstr(tok, "cval"))   col_cval_   = col;
      if (strstr(tok, "cntdwn")) col_cntdwn_ = col;
      col++;
      tok = strtok(nullptr, "|");
    }
    ESP_LOGD(TAG, "Network header: stat=%d dbt=%d cval=%d cntdwn=%d",
             col_stat_, col_dbt_, col_cval_, col_cntdwn_);

    if (log_all_nodes_active_) {
      ESP_LOGI(TAG, "==== Duco Node List (Network) ====");
    }
  }

  // ── Network data row parser ───────────────────────────────────────────────
  void parse_network_row(const char *line) {
    char buf[256];
    strncpy(buf, line, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';
    uint8_t col = 0;
    char *tok = strtok(buf, "|");
    while (tok != nullptr) {
      // Trim leading/trailing spaces
      while (*tok == ' ') tok++;
      char *end = tok + strlen(tok) - 1;
      while (end > tok && *end == ' ') { *end = '\0'; end--; }

      if (col == col_stat_ && col_stat_ != 255) {
        int mode = vent_mode_from_str(tok);
        ESP_LOGD(TAG, "Network stat=\"%s\" -> %d", tok, mode);
        if (mode >= 0) {
          if (vent_mode_)     vent_mode_->publish_state((float)mode);
          if (vent_mode_txt_) vent_mode_txt_->publish_state(vent_mode_to_str(mode));
        }
      }
      if (col == col_dbt_ && col_dbt_ != 255) {
        int v = 0;
        if (sscanf(tok, "%d", &v) == 1) {
          ESP_LOGD(TAG, "Network %%dbt=%d", v);
          if (vent_pct_) vent_pct_->publish_state((float)v);
        }
      }
      if (col == col_cval_ && col_cval_ != 255) {
        // cval in Network table is fan % (not mode number)
        int v = 0;
        if (sscanf(tok, "%d", &v) == 1) {
          ESP_LOGD(TAG, "Network cval=%d", v);
          // cval = current fan value (%) — map to countdown if no dedicated sensor
        }
      }
      if (col == col_cntdwn_ && col_cntdwn_ != 255) {
        int v = 0;
        if (sscanf(tok, "%d", &v) == 1) {
          ESP_LOGD(TAG, "Network cntdwn=%d", v);
          if (countdown_) countdown_->publish_state((float)v);
        }
      }
      col++;
      tok = strtok(nullptr, "|");
    }
  }

  // ── Phase sequencer ───────────────────────────────────────────────────────
  void advance_phase() {
    // Clear in-flight marker as soon as a nodeparaget phase completes
    // (either by Done, Failed or timeout) before issuing the next request.
    if (is_nodeparaget_phase_(phase_)) {
      nodeparaget_in_flight_ = false;

      // If a nodeparaget command timed out, abort the chained parameter reads.
      // Late replies are ambiguous ("--> <value>") and can otherwise be
      // misattributed to the next phase (e.g. CO2 interpreted as TEMP).
      if (phase_timed_out_) {
        ESP_LOGW(TAG, "Aborting nodeparaget chain after timeout in phase %d", (int) phase_);
        phase_timed_out_ = false;
        phase_ = PH_IDLE;
        last_poll_ms_ = millis();
        return;
      }
    }

    switch (phase_) {

      case PH_LOGLEVEL:
        if (had_rx_) {
          ESP_LOGD(TAG, "coreloglevel 0x10: got response");
        } else {
          ESP_LOGW(TAG, "coreloglevel 0x10: NO response! Check wiring/GPIO2.");
        }
        last_poll_ms_ = millis() - poll_interval_ms_;  // poll immediately
        phase_ = PH_IDLE;
        break;

      case PH_NETWORK:
        log_all_nodes_active_ = false;
        // Next: sensorinfo (box sensor) if any box sensor is configured
        if (box_temp_ || box_rh_) {
          send_command("sensorinfo\r\n", PH_SENSORINFO);
        } else {
          go_to_ext_sensor();
        }
        break;

      case PH_SENSORINFO:
        go_to_ext_sensor();
        break;

      case PH_NODEPARA_CO2:
        if (ext_temp_) {
          char cmd[32];
          snprintf(cmd, sizeof(cmd), "nodeparaget %u 73\r\n", ext_node_);
          send_command(cmd, PH_NODEPARA_TEMP);
        } else {
          go_to_fanspeed();
        }
        break;

      case PH_NODEPARA_TEMP:
        if (ext_rh_) {
          char cmd[32];
          snprintf(cmd, sizeof(cmd), "nodeparaget %u 75\r\n", rh_node_);
          send_command(cmd, PH_NODEPARA_RH);
        } else {
          go_to_fanspeed();
        }
        break;

      case PH_NODEPARA_RH:
        go_to_fanspeed();
        break;

      case PH_CUSTOM:
        phase_ = PH_IDLE;
        break;

      case PH_FANSPEED:
        ESP_LOGD(TAG, "Poll cycle complete");
        phase_ = PH_IDLE;
        break;

      default:
        phase_ = PH_IDLE;
        break;
    }
  }

  void go_to_ext_sensor() {
    if (ext_co2_) {
      char cmd[32];
      snprintf(cmd, sizeof(cmd), "nodeparaget %u 74\r\n", ext_node_);
      send_command(cmd, PH_NODEPARA_CO2);
    } else if (ext_temp_) {
      char cmd[32];
      snprintf(cmd, sizeof(cmd), "nodeparaget %u 73\r\n", ext_node_);
      send_command(cmd, PH_NODEPARA_TEMP);
    } else if (ext_rh_) {
      char cmd[32];
      snprintf(cmd, sizeof(cmd), "nodeparaget %u 75\r\n", rh_node_);
      send_command(cmd, PH_NODEPARA_RH);
    } else {
      go_to_fanspeed();
    }
  }

  void go_to_fanspeed() {
    if (fan_speed_) {
      send_command("fanspeed\r\n", PH_FANSPEED);
    } else {
      ESP_LOGI(TAG, "Poll cycle complete");
      phase_ = PH_IDLE;
    }
  }
};

}  // namespace duco_gateway
}  // namespace esphome