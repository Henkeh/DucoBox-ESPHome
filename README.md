# DucoBox-ESPHome
DucoBox ESPHome component based on the work of [arnemauer/Ducobox-ESPEasy-Plugin](https://github.com/arnemauer/Ducobox-ESPEasy-Plugin/). It uses native ESPHome components where possible, e.g. UART, SPI, CC1101. It was tested with a DucoBox Silent. It should work with other DucoBoxes that were supported by the ESPEasy plugin.

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
