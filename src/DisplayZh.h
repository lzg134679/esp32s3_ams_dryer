#ifndef DISPLAYZH_H
#define DISPLAYZH_H
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

typedef struct {
    const unsigned char *bitmap;
    int width;
    int height;
} Bitmap;

extern Bitmap continuous_running_60x13;
extern Bitmap overtemp_96x32;
extern Bitmap failed_MQTT_connect_80x32;
extern Bitmap no_enough_cycle_96x16;
extern Bitmap PID_auto_timeout_112x16;
extern Bitmap out_temp_error_80x32;
extern Bitmap PID_auto_fail_112x16;
extern Bitmap PID_auto_done_110x40;
extern Bitmap PID_auto_39x26;
extern Bitmap is_disable_60x20;
extern Bitmap mqtt_server_80x20;
extern Bitmap to_config_net_60x12;
extern Bitmap connect_and_visit_61x12;
extern Bitmap create_ap_61x12;
extern Bitmap six_points_50x8;
extern Bitmap clear_all_config_120x20;
extern Bitmap factory_reset_120x20;
extern Bitmap will_start_ap_120x20;
extern Bitmap connect_wifi_timeout_120x20;
extern Bitmap connecting_wifi_120x20;
extern Bitmap device_name_125x25;
extern Bitmap starting_65x16;
extern Bitmap mqtt_error_32;
extern Bitmap wifi_disconnected_20;
extern Bitmap temp_30x13;
extern Bitmap mode_30x13;
extern Bitmap time_30x13;
extern Bitmap cangtemp_30x13;
extern Bitmap custom_39x13;
extern Bitmap cang_temp_noconnect_39x13;
extern Bitmap standby_15x30;
extern Bitmap running_15x45;

void drawZh(int x, int y, const Bitmap &bitmap, Adafruit_SSD1306 &display);
#endif
