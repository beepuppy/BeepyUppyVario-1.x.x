//This version has the latest / best version so far of magnetomer control of Dial - still needs work so far as rolling config. 
//and also fastest beep and v/s average so far. 

#define LV_CONF_INCLUDE_SIMPLE
#include <lvgl.h>
#include <lv_conf.h>
#include <TFT_eSPI.h>
#include "CompassDialALPHA.c"
#include "HeadingIndicator.c"
#include "HeadingCone.c"
#include "WindDirectionArrow.c"
#include "clock.h"
#include "gps.h"
#include "barometer.h"
#include <FT6336U.h>
#include "Heading.h"
#include "Beep.h"
#include <math.h>
#include "battery.h"
#include "IMU.h"
#include "IMU_Calibration.h"
#include "Attitude.h"


  

#define BUZZER_PIN 27  // or whatever pin you're using
Beep beepManager(BUZZER_PIN, 0);
#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 480
#define TOUCH_SDA 21
#define TOUCH_SCL 22
#define TOUCH_RST 13
#define TOUCH_INT 14

#define DEBUG_HEADING



TFT_eSPI tft = TFT_eSPI();
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 10];

//Global declarations
// Time label global pointer (optional, used later to update time)
FT6336U touchController(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT);
lv_obj_t *time_label;
GPSHandler gps(34, 4, 9600);       // Initialize GPS module
lv_obj_t *hdop_label;              // Declare GPS HDOP label globally
lv_obj_t *speed_value_label;       // Declare GPS speed label globally
lv_obj_t *alt_value_label;         // ✅ Altitude display label (global)
lv_obj_t *vs_value_label;          // ✅ Altitude display label (global)
lv_obj_t *compass_img;             // ✅ Compass dial image (global)
lv_obj_t *heading_value_label[3];  // ✅ Heading value labels (global)
lv_obj_t *battery_label;           // 🔋 Battery percentage label
lv_obj_t *vs_bar;  // Vertical Speed bar in compass panel
IMUData imu;       // holds the latest accel/gyro/mag readings
lv_obj_t *calib_status_label = NULL; 


static uint32_t lastTick = 0;



// ------ LVGL Touch Read Callback ------
void touchRead(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  int16_t x = 0, y = 0;
  if (touchController.read_td_status() > 0) {
    x = touchController.read_touch1_x();
    y = touchController.read_touch1_y();
    Serial.printf("Touch Detected at X: %d, Y: %d\n", x, y);
    data->point.x = map(x, 0, SCREEN_WIDTH, 0, lv_disp_get_hor_res(NULL));
    data->point.y = map(y, 0, SCREEN_HEIGHT, 0, lv_disp_get_ver_res(NULL));
    data->state = LV_INDEV_STATE_PR;
    Serial.printf("[LVGL] Mapped Touch X: %d, Y: %d\n", data->point.x, data->point.y);
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, area->x2 - area->x1 + 1, area->y2 - area->y1 + 1);
  tft.pushColors((uint16_t *)&color_p->full,
                 (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1), true);
  tft.endWrite();
  lv_disp_flush_ready(disp_drv);
}

void setup() {
  Serial.begin(115200);
  battery::begin();
  initBarometer();  // ✅ Initialize barometer properly at startup
  initHeadingSensors();
  clock_begin();  // Start the software clock
  tft.init();
  tft.invertDisplay(true);  // Fix inverted colors
  tft.setRotation(0);
  tft.fillScreen(TFT_WHITE);  // Set background color
  //setHeadingOffset(-137.0f);  // or whatever offset you need

Serial.begin(115200);

// Initialise IMU sensors
if (!IMU_Init()) {
    Serial.println("IMU init failed!");
    while (1); // Stop here if sensors not found
}

// Initialise Attitude (after IMU is confirmed)
{
    Attitude::Settings aset; // defaults are fine for now
    Attitude::init(aset);
}




  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchRead;
  lv_indev_drv_register(&indev_drv);

  touchController.begin();
  pinMode(TOUCH_INT, INPUT);
  Serial.println("[DEBUG] Touchscreen Initialized");

  // **Top Panel (1)**
  lv_obj_t *top_panel = lv_obj_create(lv_scr_act());
  lv_obj_set_scrollbar_mode(top_panel, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(top_panel, SCREEN_WIDTH, 30);
  lv_obj_set_style_bg_color(top_panel, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(top_panel, LV_ALIGN_TOP_MID, 0, 0);

  // Add Time Label to Top Panel
  time_label = lv_label_create(top_panel);
  lv_obj_set_style_text_font(time_label, &lv_font_montserrat_10, 0);  // font size 10
  lv_label_set_text(time_label, get_datetime_string());               // Initial placeholder
  lv_obj_align(time_label, LV_ALIGN_RIGHT_MID, 3, 0);                 // Align to right with 10px margin

  // 🔋 Battery Label (centered between HDOP and Time)
  battery_label = lv_label_create(top_panel);
  lv_label_set_text(battery_label, "🔋 --%");             // Initial placeholder
  lv_obj_align(battery_label, LV_ALIGN_CENTER, -10, 0);  // Centered in top panel


  // **GPS quality label (reuse existing hdop_label)**
  hdop_label = lv_label_create(top_panel);
  lv_obj_set_style_text_font(hdop_label, &lv_font_montserrat_10, 0);  // font size 10
  lv_label_set_text(hdop_label, "HD:--  VD:--  VA:--");
  lv_obj_align(hdop_label, LV_ALIGN_LEFT_MID, -5, 0);  // keep your current alignment


  // **Three Top Panels (2, 3, 4): Speed, Alt, V/S**
int speed_width = 90;
int alt_width   = 140;
int vs_width    = 90;

lv_obj_t *speed_panel = lv_obj_create(lv_scr_act());  // (2)
lv_obj_set_scrollbar_mode(speed_panel, LV_SCROLLBAR_MODE_OFF);
lv_obj_t *alt_panel = lv_obj_create(lv_scr_act());    // (3)
lv_obj_set_scrollbar_mode(alt_panel, LV_SCROLLBAR_MODE_OFF);
lv_obj_t *vs_panel = lv_obj_create(lv_scr_act());     // (4)
lv_obj_set_scrollbar_mode(vs_panel, LV_SCROLLBAR_MODE_OFF);

lv_obj_set_size(speed_panel, speed_width, 60);
lv_obj_set_size(alt_panel,   alt_width,   60);
lv_obj_set_size(vs_panel,    vs_width,    60);

lv_obj_set_style_bg_color(speed_panel, lv_color_hex(0xFFFACD), 0);
lv_obj_set_style_bg_color(alt_panel,   lv_color_hex(0xFFFACD), 0);
lv_obj_set_style_bg_color(vs_panel,    lv_color_hex(0xFFFACD), 0);

lv_obj_align(speed_panel, LV_ALIGN_TOP_LEFT, 0, 35);
lv_obj_align(alt_panel,   LV_ALIGN_TOP_LEFT, speed_width, 35);
lv_obj_align(vs_panel,    LV_ALIGN_TOP_LEFT, speed_width + alt_width, 35);

// Speed label + value
lv_obj_t *speed_label = lv_label_create(speed_panel);
lv_label_set_text(speed_label, "Speed (mph)");
lv_obj_align(speed_label, LV_ALIGN_TOP_RIGHT, -5, -10);
lv_obj_set_style_text_font(speed_label, &lv_font_montserrat_10, 0);

speed_value_label = lv_label_create(speed_panel);
lv_label_set_text(speed_value_label, "0");
lv_obj_align(speed_value_label, LV_ALIGN_CENTER, 0, 10);
lv_obj_set_style_text_font(speed_value_label, &lv_font_montserrat_40, 0);

// Alt label + value
lv_obj_t *alt_label = lv_label_create(alt_panel);
lv_label_set_text(alt_label, "Alt (m)");
lv_obj_align(alt_label, LV_ALIGN_TOP_RIGHT, -5, -10);
lv_obj_set_style_text_font(alt_label, &lv_font_montserrat_10, 0);

alt_value_label = lv_label_create(alt_panel);
lv_label_set_text(alt_value_label, "0 m");
lv_obj_align(alt_value_label, LV_ALIGN_CENTER, 0, 10);
lv_obj_set_style_text_font(alt_value_label, &lv_font_montserrat_40, 0);

// V/S label + value
lv_obj_t *vs_label = lv_label_create(vs_panel);
lv_label_set_text(vs_label, "V/S (m/s)");
lv_obj_align(vs_label, LV_ALIGN_TOP_RIGHT, -5, -10);
lv_obj_set_style_text_font(vs_label, &lv_font_montserrat_10, 0);

vs_value_label = lv_label_create(vs_panel);
lv_label_set_text(vs_value_label, "0.0");
lv_obj_align(vs_value_label, LV_ALIGN_CENTER, 0, 10);
lv_obj_set_style_text_font(vs_value_label, &lv_font_montserrat_40, 0);


  // **Large Compass Panel (5)**
  lv_obj_t *compass_panel = lv_obj_create(lv_scr_act());
  lv_obj_clear_flag(compass_panel, LV_OBJ_FLAG_SCROLLABLE);  // ✅ Prevent clipping
  lv_obj_set_size(compass_panel, SCREEN_WIDTH, 270);
  lv_obj_set_style_bg_color(compass_panel, lv_color_white(), 0);
  lv_obj_align(compass_panel, LV_ALIGN_TOP_MID, 0, 95);

  calib_status_label = lv_label_create(compass_panel);
lv_label_set_text(calib_status_label, "");
lv_obj_align(calib_status_label, LV_ALIGN_BOTTOM_MID, 0, -5);

  // **Compass Dial (Base Image)**
  compass_img = lv_img_create(compass_panel);
  lv_img_set_src(compass_img, &CompassDialALPHA);
  lv_obj_align(compass_img, LV_ALIGN_CENTER, 0, 20);
  lv_img_set_pivot(compass_img, CompassDialALPHA.header.w / 2, CompassDialALPHA.header.h / 2);  // ✅ Rotate around center
  lv_obj_clear_flag(compass_img, LV_OBJ_FLAG_HIDDEN);                                           // ✅ Make sure it's visible
  lv_obj_set_style_opa(compass_img, LV_OPA_COVER, 0);                                           // ✅ Full opacity


  // **Heading Indicator (Overlay)**
  lv_obj_t *heading_img = lv_img_create(compass_panel);
  lv_img_set_src(heading_img, &HeadingIndicator);
  lv_obj_align(heading_img, LV_ALIGN_CENTER, 0, -75);

  // **Heading Cone (Overlay)**
  lv_obj_t *cone_img = lv_img_create(compass_panel);
  lv_img_set_src(cone_img, &HeadingCone);
  lv_obj_align(cone_img, LV_ALIGN_CENTER, 0, 20);  // tweak offsets if needed
  lv_img_set_pivot(cone_img, HeadingCone.header.w / 2, HeadingCone.header.h / 2);

  // **Wind Direction Arrow**
  lv_obj_t *wind_img = lv_img_create(compass_panel);
  lv_img_set_src(wind_img, &WindDirectionArrow);
  lv_obj_align(wind_img, LV_ALIGN_CENTER, -100, -90);  // tweak offsets if needed

  // === Vertical Speed Bar ===
// === Vertical Speed Bar: Glass Tube ===
vs_bar = lv_bar_create(compass_panel);
lv_obj_set_size(vs_bar, 20, 180);  // width, height
lv_obj_align(vs_bar, LV_ALIGN_RIGHT_MID, -5, 20);
lv_bar_set_range(vs_bar, -30, 30);
lv_bar_set_value(vs_bar, 0, LV_ANIM_OFF);
lv_bar_set_mode(vs_bar, LV_BAR_MODE_SYMMETRICAL);
lv_obj_set_style_anim_time(vs_bar, 80, 0); // smooth interpolation

// Track (main)
lv_obj_set_style_bg_color(vs_bar, lv_color_hex(0x303030), LV_PART_MAIN);
lv_obj_set_style_bg_opa(vs_bar, LV_OPA_COVER, LV_PART_MAIN);
lv_obj_set_style_border_color(vs_bar, lv_color_hex(0x808080), LV_PART_MAIN);
lv_obj_set_style_border_width(vs_bar, 2, LV_PART_MAIN);
lv_obj_set_style_radius(vs_bar, LV_RADIUS_CIRCLE, LV_PART_MAIN);
lv_obj_set_style_pad_all(vs_bar, 2, LV_PART_MAIN); // keeps indicator inside border

// Indicator (fill)
lv_obj_set_style_radius(vs_bar, 0, LV_PART_INDICATOR); // flat ends
lv_obj_set_style_border_width(vs_bar, 0, LV_PART_INDICATOR); // no border






  // **HDG Box inside Compass Panel** — WP1/WP2 removed for now

#define HEADING_BOX_WIDTH 60
#define HEADING_BOX_HEIGHT 25

  // Create the heading value box for HDG
  lv_obj_t *heading_box_hdg = lv_obj_create(compass_panel);
  lv_obj_set_size(heading_box_hdg, HEADING_BOX_WIDTH, HEADING_BOX_HEIGHT);
  lv_obj_set_style_bg_color(heading_box_hdg, lv_color_hex(0xb4eeb4), 0);
  lv_obj_set_style_border_color(heading_box_hdg, lv_color_hex(0x000000), 0);
  lv_obj_set_style_border_width(heading_box_hdg, 1, 0);
  lv_obj_align(heading_box_hdg, LV_ALIGN_TOP_MID, 0, 4);  // top center, 4px down

  // Disable scrollbars for this heading box
  lv_obj_clear_flag(heading_box_hdg, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(heading_box_hdg, LV_SCROLLBAR_MODE_OFF);

  // Inside the box: heading value ("000")
  // Using existing heading_value_label[1] slot to preserve later logic
  heading_value_label[1] = lv_label_create(heading_box_hdg);
  lv_label_set_text(heading_value_label[1], "000");
  lv_obj_center(heading_value_label[1]);  // font stays as‑is

  // Create the "HDG" label above the box
  //lv_obj_t *heading_type_label_hdg = lv_label_create(compass_panel);
  //lv_label_set_text(heading_type_label_hdg, "HDG");
  //lv_obj_align_to(heading_type_label_hdg, heading_box_hdg, LV_ALIGN_OUT_TOP_MID, 0, -2);
  //lv_obj_set_style_text_font(heading_type_label_hdg, &lv_font_montserrat_12, 0); // font size 12

  Serial.println("Compass UI with overlays loaded!");


  // **Six Independent Panels (6 through 11)**
  int small_panel_width = SCREEN_WIDTH / 3;
  int small_panel_height = 36;
  lv_obj_t *panels[6];
  lv_obj_t *panel_labels[6];
  const char *panel_names[6] = { "Time", "Fl. Time", "Dist. T,off", "Cur,GR", "GR, Next", "Max Alt" };
  int label_offset_x[6] = { -30, -20, -18, -20, -20, -20 };
  int label_offset_y[6] = { -20, -20, -20, -20, -20, -20 };

  for (int i = 0; i < 6; i++) {
    panels[i] = lv_obj_create(lv_scr_act());
    lv_obj_set_size(panels[i], small_panel_width, small_panel_height);
    lv_obj_set_style_bg_color(panels[i], lv_color_white(), 0);

    // Position panels just above bottom button panel
    int x = (i % 3) * small_panel_width;
    int y = (i < 3) ? (SCREEN_HEIGHT - 40 - (small_panel_height * 2))  // top row of small panels
                    : (SCREEN_HEIGHT - 40 - small_panel_height);       // bottom row of small panels
    lv_obj_set_pos(panels[i], x, y);

    panel_labels[i] = lv_label_create(panels[i]);
    lv_label_set_text(panel_labels[i], panel_names[i]);
    lv_obj_align(panel_labels[i], LV_ALIGN_TOP_LEFT, -10, -10);
    lv_obj_set_style_text_font(panel_labels[i], &lv_font_montserrat_10, 0);
  }


  // **Bottom Panel for Buttons (12)**
  lv_obj_t *button_panel = lv_obj_create(lv_scr_act());
  lv_obj_set_size(button_panel, SCREEN_WIDTH, 40);
  lv_obj_set_style_bg_color(button_panel, lv_color_white(), 0);
  lv_obj_set_pos(button_panel, 0, SCREEN_HEIGHT - 40);
  lv_obj_clear_flag(button_panel, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_scrollbar_mode(button_panel, LV_SCROLLBAR_MODE_OFF);

  // **Buttons inside Bottom Panel**
  const char *btn_labels[] = { "Settings", "Invert", "Mute", "Beep", "End\nFlight", "Calib" };
  int btn_pos_x[6] = { -12, 41, 94, 147, 200, 253 };    // X positions for 6 buttons
  int btn_pos_y[6] = { -11, -11, -11, -11, -11, -11 };  // uniform Y offset

  lv_color_t btn_colors[6] = {
    lv_color_hex(0xFFCB64),
    lv_color_hex(0xFFBA32),
    lv_color_hex(0xFFAA00),
    lv_color_hex(0xFFAA00),
    lv_color_hex(0xFFBA32),
    lv_color_hex(0xFFCB64)
  };

  int button_width = 52;  // **only declared once**
  int button_height = 30;

  for (int i = 0; i < 6; i++) {
    lv_obj_t *btn = lv_btn_create(button_panel);
    lv_obj_set_size(btn, button_width, button_height);
    lv_obj_set_pos(btn, btn_pos_x[i], btn_pos_y[i]);
    lv_obj_set_style_bg_color(btn, btn_colors[i], 0);
    lv_obj_set_style_bg_opa(btn, LV_OPA_COVER, 0);

    lv_obj_t *btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, btn_labels[i]);
    lv_obj_center(btn_label);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_10, 0);  // Font size


    // **Attach event only to "Invert" button**
    if (strcmp(btn_labels[i], "Invert") == 0) {
      lv_obj_add_event_cb(
        btn, [](lv_event_t *e) {
          static bool invertState = false;  // Toggle state
          invertState = !invertState;
          tft.invertDisplay(invertState);
          Serial.printf("[LVGL] Invert Button Pressed! Display inverted: %s\n", invertState ? "ON" : "OFF");
        },
        LV_EVENT_PRESSED, NULL);
    }

    if (strcmp(btn_labels[i], "Beep") == 0) {
      lv_obj_add_event_cb(
        btn, [](lv_event_t *e) {
          Serial.println("[UI] Beep Settings Button Pressed");
          beepManager.showSettingsUI(lv_scr_act());  // Show settings on current screen
        },
        LV_EVENT_CLICKED, NULL);
    }

   if (strcmp(btn_labels[i], "Calib") == 0) {
    lv_obj_add_event_cb(
        btn, [](lv_event_t *e) {
            Serial.println("[UI] Calib Menu button pressed");
            IMU_Calibration::showCalibMenu();
        },
        LV_EVENT_CLICKED, NULL);
}


  }

  lv_scr_load(lv_scr_act());
  // Serial.println("UI Setup Complete!");
}

static unsigned long lastSpeedUpdate = 0;   //
static unsigned long lastSensorUpdate = 0;  // ms


void loop() {
  uint32_t now = millis();
  lv_tick_inc(now - lastTick);
  lastTick = now;
  lv_timer_handler();

  // 🔹 Update Barometer (Every 30ms for responsiveness)
  if (millis() - lastSensorUpdate > 10) {
    updateBarometer();
    beepManager.update(vSmoothed);
    lastSensorUpdate = millis();

    if (Serial.available()) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd.equalsIgnoreCase("CALMAG")) {
        IMU_Calibration::startMagCalibration();
      }
    }

    lv_label_set_text(vs_value_label, String(vSmoothed, 1).c_str());
    lv_bar_set_value(vs_bar, (int)(vSmoothed * 10), LV_ANIM_ON);

    lv_color_t col;
    if (vSmoothed > 0.0f) {
      col = lv_color_make(0, 255, 0);
    } else if (vSmoothed < 0.0f) {
      col = lv_color_make(255, 0, 255);
    } else {
      col = lv_color_make(128, 128, 128);
    }
    lv_obj_set_style_bg_color(vs_bar, col, LV_PART_INDICATOR);
  }

  // 🔹 Update GPS Data (Every 500ms)
  static unsigned long lastGPSUpdate = 0;
  if (millis() - lastGPSUpdate > 500) {
    gps.update();
    lastGPSUpdate = millis();
  }

  // 🔹 Throttled Heading Update
  static unsigned long lastHeadingUpdate = 0;
  if (millis() - lastHeadingUpdate > 50) {
    static unsigned long lastHeadingUpdate = 0;
if (millis() - lastHeadingUpdate >= 50) {
    updateHeading();
    lastHeadingUpdate = millis();
}

  clock_update();
  update_time_from_gps();

  battery::update();
  char batteryStr[20];
  if (battery::externalPower()) {
    snprintf(batteryStr, sizeof(batteryStr), "USB");
  } else {
    snprintf(batteryStr, sizeof(batteryStr), "🔋 %d%%", battery::getPercentage());
  }
  lv_label_set_text(battery_label, batteryStr);

  lv_label_set_text(time_label, get_datetime_string());

  // 🔹 GPS Quality Label
  char gpsQualStr[48];
  float hd = gps.getHDOP();
  float vd = gps.getVDOP();
  bool haveFix = gps.isValid() && !isnan(hd) && hd > 0.0f;
  bool haveVD = !isnan(vd) && vd > 0.0f;
  bool qnhLocked = haveFix && gps.getSatellites() >= 6 && hd <= 1.5 && haveVD && vd <= 2.5;

  if (!haveFix) {
    snprintf(gpsQualStr, sizeof(gpsQualStr), "HD:--  VD:--  VA:--");
  } else if (!haveVD) {
    snprintf(gpsQualStr, sizeof(gpsQualStr), "HD:%.2f  VD:--  VA:--%s", hd, qnhLocked ? " ✓" : "");
  } else {
    snprintf(gpsQualStr, sizeof(gpsQualStr), "HD:%.2f  VD:%.2f  VA:--%s", hd, vd, qnhLocked ? " ✓" : "");
  }
  lv_label_set_text(hdop_label, gpsQualStr);

  // 🔹 Altitude Label (Every 30ms)
  static unsigned long lastAltUpdate = 0;
  if (millis() - lastAltUpdate > 30) {
    lastAltUpdate = millis();
    lv_label_set_text(alt_value_label, String(altitude, 1).c_str());
  }

  // 🔹 Speed Label (Every 500ms)
  static unsigned long lastSpeedUpdate = 0;
  if (millis() - lastSpeedUpdate > 500) {
    lastSpeedUpdate = millis();
    lv_label_set_text(speed_value_label, String(gps.getSpeed() * 0.621371, 0).c_str());
  }

  // 🔹 Compass Dial + Heading Label
  float hdg_unw = getHeadingUnwrappedDegrees();
  float hdg_draw = fmodf(hdg_unw, 360.0f);
  if (hdg_draw < 0) hdg_draw += 360.0f;
  int16_t angle_0p1 = (int16_t)lrintf((360.0f - hdg_draw) * 10.0f);
  lv_img_set_angle(compass_img, angle_0p1);
  lv_obj_invalidate(compass_img);

  float hdg_norm = getHeadingDegrees();
  char headingStr[4];
  snprintf(headingStr, sizeof(headingStr), "%03d", (int)lrintf(hdg_norm));
  lv_label_set_text(heading_value_label[1], headingStr);
  lv_obj_invalidate(heading_value_label[1]);

  // --- IMU + Attitude (Throttled) ---
  static unsigned long lastIMURead = 0;
  static unsigned long lastAttitudeUpdate = 0;
  if (millis() - lastIMURead >= 20) {
    static unsigned long lastIMURead = 0;
if (millis() - lastIMURead >= 20) {
    if (IMU_Read(imu)) {
        lastIMURead = millis();
        static uint32_t last_us = micros();
        uint32_t now_us = micros();
        float dt = (now_us - last_us) / 1000000.0f;
        last_us = now_us;

        static unsigned long lastAttitudeUpdate = 0;
        if (millis() - lastAttitudeUpdate >= 20) {
            Attitude::update(imu, dt);
            lastAttitudeUpdate = millis();

#ifdef DEBUG_HEADING
            static unsigned long _lastDiag = 0;
            if (millis() - _lastDiag >= 1000) {
                _lastDiag = millis();
                float _yaw = Attitude::getYawUnwrappedDeg();
                bool _magT = Attitude::magTrusted();
                float _hdg = getHeadingDegrees();
                Serial.printf("DIAG RAW_MAG:[%7.1f %7.1f %7.1f] CAL_MAG:[%7.1f %7.1f %7.1f] ATT_YAW:%.2f MAG_TRUST:%d HDG:%.2f\\n",
                    imu_last_raw_mag[0], imu_last_raw_mag[1], imu_last_raw_mag[2],
                    imu.mag[0], imu.mag[1], imu.mag[2],
                    _yaw, (int)_magT, _hdg);
            }
#endif

        }
    }
}


  // --- Calibration Update (Throttled) ---
  static unsigned long lastCalibUpdate = 0;
if (millis() - lastCalibUpdate >= 100) {
    IMU_Calibration::update();
    lastCalibUpdate = millis();
}


  if (IMU_Calibration::isCalibrating()) {
    lv_label_set_text(calib_status_label, "Calibrating... Rotate device");
  } else {
    lv_label_set_text(calib_status_label, "");
  }

  // --- Touch Debug (if GPS invalid) ---
  if (!gps.isValid()) {
    static bool touchPreviouslyDetected = false;
    int touchCount = touchController.read_td_status();
    if (touchCount > 0) {
      touchPreviouslyDetected = true;
    } else if (touchPreviouslyDetected) {
      touchPreviouslyDetected = false;
    }
  }
}  // closes lastHeadingUpdate block
}  
}
