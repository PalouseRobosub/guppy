Situations handled on a scale of 0-4:
- 4 = Logged (Not that Bad)
- 3 = LEDS blinking warning
- 2 = Take situational action (Ex: restart or stop program/s)
- 1 = Estop state
- 0 = Cut power (Bad)


This subsystem checks for and responds to the following situations:
| Response | Subsystem | Field | Value | Additional Comments  |
| --- | --- | --- | --- | --- |
| [3 ]| Battery | voltage | low |
| [4] | Battery | voltage | high |
|[ 2] | Battery | temp | approaching high |
|[ 0] | Battery | temp | high |
| [4] | Battery | temp | low |
| [2] | Battery | temp | extreme |
| [2] | Battery | voltage | extreme |
| [1] | Battery | current | high | with no motors running |
| | | | | |
| [2] | CAN | utilization | high |
| [3] | BMS module | connection | loss | no can messages in x time |
| [3] | LED module | connection | loss | no can messages in x time |
| [2] | Arm module | connection | loss | no can messages in x time |
| [1] | Motor controller | connection | loss | no can messages in x time |
| | | | | |
| [1] | Motor | current | exceedingly low (~0) | When in use, Motor disconnected or another big issue |
| [1] | Motor controller | temp | high |
| [3] | Motor controller voltage | differs | between others / bms
| [1] | Motor | current | high |
| [2] | Motor | voltage | extreme |
| [2] | Motor controller | temp | extreme |
| [2] | Motor | current | extreme |
| | | | | |
| [1] | Teleop | heartbeat | stopped | Connection lost |
| [3] | Teleop | heartbeat | slow/inconsistent | Poor connection |
| | | | | |
| [3] | Nav | accuracy | low |
| | | | | |
| [2] | Barometer | reading | extreme |
| [2] | IMU | reading | extreme | 
| | | | | |
| [2] | Moisture sensor | connection | loss |
| [0] | Moisture sensors| reading | wet |
| [1] | Moisture sensor | reading | extereme |
| | | | | |
| [2] | ros | non-essential program | unhealthy | Ex: led, barometer |
| [1] | ros | esential program | unhealthy | Ex: state, safety, control, localization, tasks, vision
| | | | | |
| [2] | ros | action | update timeout | possible to check? |
| [2] | ros | service | no repsonse | possible to check? |
| | | | | |
| [3] | ros | logs | filling quickly | Something broke? |
| | | | | |
| [2] | Panda | memory | low | 
| [3] | Panda| CPU utilization | high (~=100%) | for extended time |
| [3] | Panda | CPU temp | high |
| [4] | Panda | Network | disconnected |





