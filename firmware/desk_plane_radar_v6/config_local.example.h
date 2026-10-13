// Desk Plane Radar v6: local configuration. Copy to config_local.h (gitignored) and edit. Everything here is optional;
// the sketch falls back to its defaults for anything you leave out.
#pragma once

// Your house: the "Home" entry of the weather picker and the centre of the look-up alert.
#define WX_HOME_NAME "Home"
#define WX_HOME_LAT  13.1989
#define WX_HOME_LON  77.7056

// Time zone (POSIX string and IANA name, both for the same place) and the city shown on the World clock face.
#define LOCAL_TZ      "IST-5:30"
#define LOCAL_TZ_IANA "Asia/Kolkata"
#define LOCAL_CITY    "BENGALURU"

// Greeting name (also editable on the web page) and the mDNS host name (http://<name>.local).
#define USER_NAME_DEFAULT ""
#define OTA_HOSTNAME      "deskradar6"

// Panel wired mirrored (left and right TFT CS, OLED buses and encoders swapped)? Uncomment.
// #define PANEL_SWAP 1
