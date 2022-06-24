# RF_triggered_meas

Controller, Radio, and Datalogger programs comprise the software belonging to a measurement system capable of data-logging acceleration values. The commands are user entered from the Controller, packet radio transmission is used to communicate between the Controller and the acompanying 'Node' or 'Nodes'. A Node is comprised of a Feather 32u4 RF9X (Radio) and a 32u4 with an on board microSD (Datalogger) connected via the i2c communication lines. The Feather 32u4 RF9X is the board used for the Controller, it is mounted with an LCD screen for viewing the sent commands and response from the Nodes.

This series of programs utilizes many libraries, the main libraries that were employed, and serve as the backbone of this system are the SdFat library https://github.com/greiman/SdFat and the RadioHead library http://www.airspayce.com/mikem/arduino/RadioHead/. 


