# DucoBox-ESPHome
DucoBox ESPHome component based on the work of [arnemauer/Ducobox-ESPEasy-Plugin](https://github.com/arnemauer/Ducobox-ESPEasy-Plugin/). It uses native ESPHome components where possible, e.g. UART, SPI, CC1101. It was tested with a DucoBox Silent. It should work with other DucoBoxes that were supported by the ESPEasy plugin.

## Example configurations
- `ducobox.yaml`: full gateway + RF example (ESP32 active, ESP8266 alternative commented in-file).
- `ducobox-simple.yaml`: simplified full gateway + RF example (ESP32 active, ESP8266 alternative commented in-file).
- `ducobox-rf.yaml`: RF-only example (ESP32 active, ESP8266 alternative commented in-file).
- `ducobox-rf-simple.yaml`: simplified RF-only example (ESP32 active, ESP8266 alternative commented in-file).
- `ducobox-gateway.yaml`: serial gateway-only example for ESP32(-S3) (read-only ventilation telemetry).
- `ducobox-diag.yaml`: diagnostic variant of `ducobox.yaml` with extra logging and a custom command text entity for troubleshooting.

## Board selection
The examples use ESP32 by default and include commented ESP8266 alternatives in the same file.

To switch one of the combined examples from ESP32 to ESP8266:
1. Comment the `esp32:` block and uncomment the `esp8266:` block.
2. Update the pin values in the `substitutions:` block at the top of the file.

## Pin configuration via substitutions
Pinning is configured through substitutions near the top of each example so hardware changes are made in one place.

Typical substitutions look like this:

```yaml
substitutions:
    uart_tx_pin: GPIO43
    uart_rx_pin: GPIO44
    cc1101_cs_pin: GPIO07
    cc1101_gdo0_pin: GPIO06
    spi_clk_pin: GPIO02
    spi_miso_pin: GPIO04
    spi_mosi_pin: GPIO01
    # ESP8266 alternative pins:
    # uart_tx_pin: GPIO1
    # uart_rx_pin: GPIO3
    # cc1101_cs_pin: GPIO15
    # cc1101_gdo0_pin: GPIO02
    # spi_clk_pin: GPIO14
    # spi_miso_pin: GPIO12
    # spi_mosi_pin: GPIO13
```

The gateway-only example only uses the UART substitutions. The RF-only examples only use the CC1101 and SPI substitutions.

# Usage
In order to use the component perform the following steps:
1. Flash one of the example ESPHome configurations onto your device.
2. Put your ventilation system into installer mode:
    1. **Via the DucoBox itself:** Press the 'INST' button on the unit.
    2. **Via a Control/Sensor:** Long-press two diagonal buttons simultaneously on a paired user controller.
3. Click the "Pair" button, either in HA or on the web server of the ESP itself.
    1. Your ESP will save the network ID and node address automatically and persistently, so no update of the YAML config after pairing is needed.
4. Click on "Disable Installer Mode" to exit the installer mode and restart normal operation of the ventilation unit.

# Determine node IDs
In order to determine the node IDs of other devices in the network, click "Log All Duco Nodes" after pairing to show the available nodes, e.g. BOX, UCCO2, UCBAT, SWITCH, UCRH. These can be used to configure the ESP to request additional information for each node via the serial gateway, e.g. temperature, humidity or CO2 concentration. The ESP will also show up as a CO2 remote (UCCO2).

In order to reconfigure these in the ESP, the configuration file needs to be updated with the right node ID and the ESP reprogrammed. Afterwards the information from the nodes should become available in the ESP as well.
