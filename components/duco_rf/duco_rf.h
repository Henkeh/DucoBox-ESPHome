#pragma once
/**
 * Ducobox RF Gateway — ESPHome component wrapper
 *
 * Wraps the original DucoCC1101 library by Arne Mauer.
 * Source: github.com/arnemauer/Ducobox-ESPEasy-Plugin/tree/master/lib/Duco
 *
 * Place these files from arnemauer's repo into components/duco_rf/:
 *   duco_rf.h DucoCC1101.h  DucoCC1101.cpp
 */

#include "esphome/core/component.h"
#include "esphome/components/cc1101/cc1101.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

#include <cstring>
#include <string>
#include <vector>

#include "DucoCC1101.h"

namespace esphome {
namespace duco_rf {

static const char *const TAG = "duco_rf";
static const int kFixedTemperature = 210;

struct PersistedNetworkState {
  uint8_t version;
  uint8_t device_address;
  uint8_t network_id[4];
};

class DucoRF : public Component, public cc1101::CC1101Listener {
 public:
  // ── Setters (called from generated __init__.py code) ─────────────────────
  void set_cc1101(cc1101::CC1101Component *radio) { cc1101_ = radio; }
  void set_device_address(uint8_t a)  { device_address_ = a; }
  void set_network_id(uint8_t b0, uint8_t b1, uint8_t b2, uint8_t b3) {
    network_id_[0] = b0; network_id_[1] = b1;
    network_id_[2] = b2; network_id_[3] = b3;
  }

  // Sensors
  void set_vent_mode_sensor(sensor::Sensor *s)                { vent_mode_sensor_ = s; }
  void set_vent_mode_text_sensor(text_sensor::TextSensor *s)  { vent_mode_text_sensor_ = s; }
  void set_network_id_text_sensor(text_sensor::TextSensor *s) { network_id_text_sensor_ = s; }
  void set_device_address_text_sensor(text_sensor::TextSensor *s) { device_address_text_sensor_ = s; }
  void set_log_rf_messages(bool log_rf_messages) { log_rf_messages_ = log_rf_messages; }

  void on_packet(const std::vector<uint8_t> &packet, float, float rssi, uint8_t lqi) override {
    this->log_packet("RX", packet, rssi, lqi);

    if (!rf_.ingest_packet(packet, static_cast<int>(rssi), lqi)) {
      return;
    }

    rf_.processNewMessages();
    publish_vent_mode_();
    this->sync_network_id_state_();
    flush_rf_log_();
  }

  // ── ESPHome lifecycle ─────────────────────────────────────────────────────
  void setup() override {
    if (cc1101_ == nullptr) {
      ESP_LOGE(TAG, "cc1101_id is not configured");
      this->mark_failed();
      return;
    }

    this->network_state_pref_ = global_preferences->make_preference<PersistedNetworkState>(0x4455434F, true);
    this->load_persisted_network_state_();

    ESP_LOGI(TAG, "Initialising Duco RF (addr=%u netid=%02X%02X%02X%02X temp=%d)",
             device_address_,
             network_id_[0], network_id_[1], network_id_[2], network_id_[3],
         kFixedTemperature);

    // Configure the library
    rf_.setLogRFMessages(log_rf_messages_);
    rf_.setGatewayAddress(device_address_);
    rf_.setNetworkId(network_id_);
    rf_.setTemperature(kFixedTemperature);
    rf_.set_tx_callback([this](const std::vector<uint8_t> &payload) {
      this->log_packet("TX", payload, 0.0f, 0);
      return this->cc1101_->transmit_packet(payload) == cc1101::CC1101Error::NONE;
    });
    cc1101_->register_listener(this);
    initial_subscribe_pending_ = true;
    initial_subscribe_started_at_ = millis();

    ESP_LOGI(TAG, "CC1101 transport bound; initial subscribe queued");
    this->sync_network_id_state_();
  }

  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  void loop() override {
    if (initial_subscribe_pending_ && millis() - initial_subscribe_started_at_ >= 250) {
      initial_subscribe_pending_ = false;
      ESP_LOGI(TAG, "Sending initial subscribe to Ducobox");
      rf_.sendSubscribeMessage();
      flush_rf_log_();
    }

    // Check pending ACKs / retransmissions.
    rf_.checkForAck();

    // Poll for completed join / disjoin.
    if (rf_.pollNewDeviceAddress()) {
      if (!this->is_unpaired_state_()) {
        ESP_LOGI(TAG, "Join OK — new network_id=%s, device address=%u",
                 this->network_id_to_string_(rf_.getnetworkID()).c_str(), rf_.getDeviceAddress());
      } else {
        ESP_LOGI(TAG, "Disjoin complete — network_id=%s, device address=%u",
                 this->network_id_to_string_(rf_.getnetworkID()).c_str(), rf_.getDeviceAddress());
      }
      this->save_persisted_network_state_();
      this->sync_network_id_state_();
      flush_rf_log_();
    }

    // Keep the UI in sync even when mode updates arrive through paths
    // that do not immediately trigger a publish in the IRQ handling block.
    publish_vent_mode_();
  }

  // ── Commands callable from HA (e.g. via ESPHome API or button entities) ──

  /** Request a ventilation mode change.
   *  mode: 0=AUTO, 4=LOW, 5=MIDDLE, 6=HIGH, 7=NOTHOME
   *  permanent: true = stays until changed, false = timed override
   *  percentage: sensor demand percentage (0 = not used)
   */
  void request_ventilation_mode(uint8_t mode, bool permanent = false, uint8_t percentage = 0) {
    // AWAY/NOTHOME (mode 7) does not support a permanent flag in Duco packets.
    if (mode == 7 && permanent) {
      ESP_LOGD(TAG, "Mode 7 (Away) ignores permanent=true; forcing permanent=false");
      permanent = false;
    }

    ESP_LOGI(TAG, "Requesting mode=%s, permanent=%s, percentage=%s", mode_to_string_(map_vent_mode_(mode, permanent)), permanent ? "yes" : "no", percentage_to_string_(percentage));
    rf_.requestVentilationMode(mode, permanent, percentage);
    // // Immediate local feedback; RF confirmations can still update this later.
    // publish_vent_mode_from_raw_(mode, permanent);
    flush_rf_log_();
  }

  void pair() { ESP_LOGI(TAG, "Pair requested"); rf_.sendJoinPacket(); flush_rf_log_(); }
  void unpair() { ESP_LOGI(TAG, "Unpair requested"); rf_.sendDisjoinPacket();        flush_rf_log_(); }
  void enable_installer_mode() { ESP_LOGI(TAG, "Enable installer mode requested"); rf_.enableInstallerMode();  flush_rf_log_(); }
  void disable_installer_mode() { ESP_LOGI(TAG, "Disable installer mode requested"); rf_.disableInstallerMode(); flush_rf_log_(); }

 protected:
  // ── Config ────────────────────────────────────────────────────────────────
  cc1101::CC1101Component *cc1101_{nullptr};
  uint8_t device_address_{0};
  uint8_t network_id_[4] {0x00, 0x00, 0x00, 0x00};

  // ── Sensors ───────────────────────────────────────────────────────────────
  sensor::Sensor           *vent_mode_sensor_      {nullptr};
  text_sensor::TextSensor  *vent_mode_text_sensor_ {nullptr};
  text_sensor::TextSensor  *network_id_text_sensor_ {nullptr};
  text_sensor::TextSensor  *device_address_text_sensor_ {nullptr};

  // ── Library instance ──────────────────────────────────────────────────────
  DucoCC1101 rf_;
  uint8_t    last_mode_ {0xFF};
  uint8_t last_network_id_[4] {0xFF, 0xFF, 0xFF, 0xFF};
  uint8_t last_device_address_ {0xFF};
  ESPPreferenceObject network_state_pref_;
  bool log_rf_messages_{false};
  bool initial_subscribe_pending_{false};
  uint32_t initial_subscribe_started_at_{0};

  // ── Helpers ───────────────────────────────────────────────────────────────

  void log_packet(const char *direction, const std::vector<uint8_t> &packet, float rssi, uint8_t lqi) {
    if (packet.size() < 8) {
      ESP_LOGD(TAG, "%s len=%u (too short)", direction, static_cast<unsigned>(packet.size()));
      return;
    }

    const uint8_t message_type = packet[0];
    const uint8_t source_address = (packet[5] >> 3);
    const uint8_t destination_address = ((packet[5] & 0b00000111) << 2) | (packet[6] >> 6);
    const uint8_t original_destination_address = ((packet[6] & 0b00000001) << 4) | (packet[7] >> 4);
    const uint8_t counter = packet[7] & 0x0F;

    if (strcmp(direction, "RX") == 0) {
      ESP_LOGD(TAG, "%s type=0x%02X src=%u dst=%u orig_dst=%u ctr=%u len=%u rssi=%.1f lqi=%u", direction,
               message_type, source_address, destination_address, original_destination_address, counter,
               static_cast<unsigned>(packet.size()), rssi, lqi);
    } else {
      ESP_LOGD(TAG, "%s type=0x%02X src=%u dst=%u orig_dst=%u ctr=%u len=%u", direction, message_type,
               source_address, destination_address, original_destination_address, counter,
               static_cast<unsigned>(packet.size()));
    }
  }

  /** Publish current ventilation mode if it changed.
   *
   *  DucoCC1101 internal mode encoding (currentVentilationMode):
   *    0 = AUTO
   *    4 = LOW   (MAN1 if not permanent, CNT1 if permanent)
   *    5 = MID   (MAN2 / CNT2)
   *    6 = HIGH  (MAN3 / CNT3)
   *    7 = NOTHOME (EMPT)
   *
   *  We map these to the same numbers used by the serial gateway sensor
   *  (0=AUTO 1=MAN1 2=MAN2 3=MAN3 4=EMPT 11=CNT1 12=CNT2 13=CNT3).
   */
  void publish_vent_mode_() {
    uint8_t raw   = rf_.getCurrentVentilationMode();
    bool    perm  = rf_.getCurrentPermanentMode();

    publish_vent_mode_from_raw_(raw, perm);
  }

  uint8_t map_vent_mode_(uint8_t raw, bool perm) const {

    uint8_t mapped;
    if (!perm) {
      switch (raw) {
        case 0: mapped =  0; break;  // AUTO
        case 4: mapped =  1; break;  // MAN1
        case 5: mapped =  2; break;  // MAN2
        case 6: mapped =  3; break;  // MAN3
        case 7: mapped =  4; break;  // EMPT
        default: mapped = raw; break;
      }
    } else {
      switch (raw) {
        case 4: mapped = 11; break;  // CNT1
        case 5: mapped = 12; break;  // CNT2
        case 6: mapped = 13; break;  // CNT3
        case 7: mapped =  4; break;  // EMPT (Away is never a permanent mode)
        default: mapped = raw; break;
      }
    }

    return mapped;
  }

  void publish_vent_mode_from_raw_(uint8_t raw, bool perm) {
    const uint8_t mapped = map_vent_mode_(raw, perm);

    if (mapped == last_mode_) return;
    last_mode_ = mapped;

    const char *s = mode_to_string_(mapped);
    ESP_LOGD(TAG, "Ventilation mode → %u (%s)", mapped, s);

    if (vent_mode_sensor_)      vent_mode_sensor_->publish_state((float)mapped);
    if (vent_mode_text_sensor_) vent_mode_text_sensor_->publish_state(s);
  }

  static const char *mode_to_string_(uint8_t mode) {
    switch (mode) {
      case 0:  return "AUTO";
      case 1:  return "MAN1";
      case 2:  return "MAN2";
      case 3:  return "MAN3";
      case 4:  return "EMPT";
      case 11: return "CNT1";
      case 12: return "CNT2";
      case 13: return "CNT3";
      default: {
        ESP_LOGI(TAG, "Unknown ventilation mode: %u", mode);
        return "Unknown";
      }
    }
  }

  static const char *percentage_to_string_(uint8_t percentage) {
    if (percentage == 0) {
      return "Not used";
    } else {
      static char buf[16];
      snprintf(buf, sizeof(buf), "%u%%", percentage);
      return buf;
    }
  }

  std::string network_id_to_string_(const uint8_t *id) const {
    char buf[9];
    snprintf(buf, sizeof(buf), "%02X%02X%02X%02X", id[0], id[1], id[2], id[3]);
    return std::string(buf);
  }

  std::string device_address_to_string_(uint8_t address) const {
    char buf[4];
    snprintf(buf, sizeof(buf), "%u", address);
    return std::string(buf);
  }

  bool is_unpaired_state_() {
    const uint8_t *id = rf_.getnetworkID();
    return rf_.getDeviceAddress() == 0 && id[0] == 0 && id[1] == 0 && id[2] == 0 && id[3] == 0;
  }

  bool load_persisted_network_state_() {
    PersistedNetworkState state{};
    if (!this->network_state_pref_.load(&state)) {
      ESP_LOGI(TAG, "No persisted network state found; using configured values");
      return false;
    }

    if (state.version != 1) {
      ESP_LOGW(TAG, "Persisted network state version mismatch (%u); ignoring", state.version);
      return false;
    }

    this->device_address_ = state.device_address;
    memcpy(this->network_id_, state.network_id, sizeof(this->network_id_));
    ESP_LOGI(TAG, "Loaded persisted network state: network_id=%s, device address=%u",
             this->network_id_to_string_(this->network_id_).c_str(), this->device_address_);
    return true;
  }

  void save_persisted_network_state_() {
    PersistedNetworkState state{};
    state.version = 1;
    state.device_address = rf_.getDeviceAddress();
    memcpy(state.network_id, rf_.getnetworkID(), sizeof(state.network_id));

    if (!this->network_state_pref_.save(&state)) {
      ESP_LOGW(TAG, "Failed to save persisted network state");
      return;
    }

    if (!global_preferences->sync()) {
      ESP_LOGW(TAG, "Failed to sync persisted network state");
      return;
    }

    ESP_LOGI(TAG, "Persisted network state: network_id=%s, device address=%u",
             this->network_id_to_string_(state.network_id).c_str(), state.device_address);
  }

  void sync_network_id_state_() {
    uint8_t *current = rf_.getnetworkID();
    uint8_t current_device_address = rf_.getDeviceAddress();

    memcpy(this->network_id_, current, sizeof(this->network_id_));
    this->device_address_ = current_device_address;

    bool network_changed = memcmp(current, this->last_network_id_, sizeof(this->last_network_id_)) != 0;
    bool device_address_changed = current_device_address != this->last_device_address_;

    if (!network_changed && !device_address_changed) {
      return;
    }

    if (network_changed) {
      memcpy(this->last_network_id_, current, sizeof(this->last_network_id_));
      const std::string network_id_str = this->network_id_to_string_(current);

      if (this->network_id_text_sensor_ != nullptr) {
        this->network_id_text_sensor_->publish_state(network_id_str);
      }
    }

    if (device_address_changed) {
      this->last_device_address_ = current_device_address;
      const std::string device_address_str = this->device_address_to_string_(current_device_address);

      if (this->device_address_text_sensor_ != nullptr) {
        this->device_address_text_sensor_->publish_state(device_address_str);
      }
    }
  }

  /** Drain internal log ring-buffer from DucoCC1101 into ESPHome logger. */
  void flush_rf_log_() {
    uint8_t n = rf_.getNumberOfLogMessages();
    for (uint8_t i = 0; i < n; i++) {
      ESP_LOGD(TAG, rf_.logMessages[i]);
    }
  }
};

}  // namespace duco_rf
}  // namespace esphome
