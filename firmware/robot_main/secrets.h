// =====================================================================
//  secrets.h - your Wi-Fi and over-the-air (OTA) update password.
//  Edit THIS file (not robot_main.ino) and keep it to yourself: don't paste it into chats
//  and don't share it. robot_main.ino reads these three values from here.
// =====================================================================
#pragma once

#define SECRET_WIFI_SSID     "MiFi_021208"     // must offer 2.4 GHz - the ESP32 has no 5 GHz
#define SECRET_WIFI_PASS     "livinglabsnuc123"
#define SECRET_OTA_PASSWORD  "change-me"       // needed to update the robot over Wi-Fi. Pick your own:
                                               // on a shared network (competition venue) anyone who
                                               // knows it could re-flash the robot.
