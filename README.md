# RF_triggered_meas

This software was developed to support a wireless sensor network (WSN) for use in snow dynamics research. This project was presented at the ISSW 2023 conference in Bend, OR. The proceedings paper, detailing the WSN, can be found at: https://arc.lib.montana.edu/snow-science/item.php?id=3080

Overview: "Controller", "Radio", and "Datalogger" programs comprise the software belonging to a set of microcontrollers working in unison to remote trigger high speed data-logging of acceleration. The commands are user entered from the controller (central point in the network). LoRa radio transmission is used to communicate between the controller and the 'nodes' (sensing units/endpoints in the network). Each node is comprised of a Feather 32u4 RF95 ("Radio") and a Feather 32u4 Adalogger equipped with an on-board microSD ("Datalogger"). The Feather 32u4 RF95 is also used by the controller, additionally, an LCD screen is used for viewing the sent commands and responses from the nodes. Data is logged from an analog accelerometer, Analog Devices ADXL356CZ, at a rate of 10 kHz with 8-bit precision or 5 kHz with 10-bit precision.

Persistent Identifier: https://zenodo.org/badge/latestdoi/292366180
