/* Using LVGL with Arduino requires some extra steps...
 *  
 * Be sure to read the docs here: https://docs.lvgl.io/8.3/get-started/platforms/arduino.html
 * but note you should use the lv_conf.h from the repo as it is pre-edited to work.
 * 
 * You can always edit your own lv_conf.h later and exclude the example options once the build environment is working.
 * 
 * Note you MUST move the 'examples' and 'demos' folders into the 'src' folder inside the lvgl library folder 
 * otherwise this will not compile, please see README.md in the repo.
 * 
 */

#include <lvgl.h>
#include <TFT_eSPI.h>
#include "ui.h"

#include <XPT2046_Touchscreen.h>
// A library for interfacing with the touch screen
//
// Can be installed from the library manager (Search for "XPT2046")
//https://github.com/PaulStoffregen/XPT2046_Touchscreen
// ----------------------------
// Touch Screen pins
// ----------------------------

// The CYD touch uses some non default
// SPI pins

#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
SPIClass mySpi = SPIClass(VSPI);
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);
uint16_t touchScreenMinimumX = 200, touchScreenMaximumX = 3700, touchScreenMinimumY = 240,touchScreenMaximumY = 3800;


/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 320;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp_drv );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_drv, lv_indev_data_t * data )
{
    if(ts.touched())
    {
        TS_Point p = ts.getPoint();
        //Some very basic auto calibration so it doesn't go out of range
        if(p.x < touchScreenMinimumX) touchScreenMinimumX = p.x;
        if(p.x > touchScreenMaximumX) touchScreenMaximumX = p.x;
        if(p.y < touchScreenMinimumY) touchScreenMinimumY = p.y;
        if(p.y > touchScreenMaximumY) touchScreenMaximumY = p.y;
        //Map this to the pixel position
        data->point.x = map(p.x,touchScreenMinimumX,touchScreenMaximumX,1,screenWidth); /* Touchscreen X calibration */
        data->point.y = map(p.y,touchScreenMinimumY,touchScreenMaximumY,1,screenHeight); /* Touchscreen Y calibration */
        data->state = LV_INDEV_STATE_PR;

        //Serial.print( "Touch x " );
        //Serial.print( data->point.x );
        //Serial.print( " y " );
        //Serial.println( data->point.y );
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}

#include <esp_now.h>
#include <WiFi.h>
#include <map>

#define DEVICE_1 1
#define DEVICE_2 2
#define DEVICE_3 3

// Timeout for considering a device offline (10 seconds)
#define TIMEOUT_DURATION 2000
#define STATUS_CHECK_INTERVAL 500  // Check every half a second

// Structure to hold the message data
typedef struct {
  int deviceId;
  long timestamp;
} heartbeat_message_t;

// Variables to store online status for each device
bool device1Online = false;
bool device2Online = false;
bool device3Online = false;

bool previousDevice1Online = !device1Online;
bool previousDevice2Online = !device2Online;
bool previousDevice3Online = !device3Online;

// Map to store the last heartbeat for each device
std::map<int, long> lastHeartbeats;

long lastStatusCheckTime = 0;  // Keeps track of when the last status check was performed

void OnDataRecv(const uint8_t* mac, const uint8_t* incomingData, int len) {
  heartbeat_message_t heartbeat;
  memcpy(&heartbeat, incomingData, sizeof(heartbeat));

  int deviceId = heartbeat.deviceId;
  long currentTime = millis();

  // Store the timestamp of the received heartbeat
  lastHeartbeats[deviceId] = currentTime;
}

TaskHandle_t statusCheckTaskHandle;
TaskHandle_t updateStatusesTaskHandle;

void StatusCheckTask(void* parameter) {
  while (true) {
    checkstatus();
    vTaskDelay(STATUS_CHECK_INTERVAL / portTICK_PERIOD_MS);
  }
}

void UpdateStatusesTask(void* parameter) {
  while (true) {
    updateDeviceStatuses();
    vTaskDelay(100); // Run every 100ms
  }
}

void setup()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */

    String LVGL_Arduino = "LVGL version ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );

    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS); /* Start second SPI bus for touchscreen */
    ts.begin(mySpi); /* Touchscreen init */
    ts.setRotation(0); /* Landscape orientation */

    tft.begin();          /* TFT init */
    tft.invertDisplay(1);
    tft.setRotation( 0 ); /* Landscape orientation */

    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_t * my_indev = lv_indev_drv_register( &indev_drv );
  
    /* Uncomment to create simple label */
    // lv_obj_t *label = lv_label_create( lv_scr_act() );
    // lv_label_set_text( label, "Hello Ardino and LVGL!");
    // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
 
    /* Try an example. See all the examples 
     * online: https://docs.lvgl.io/master/examples.html
     * source codes: https://github.com/lvgl/lvgl/tree/e7f88efa5853128bf871dde335c0ca8da9eb7731/examples
     */
     // lv_example_animimg_1();
     // lv_example_arc_1();
     // lv_example_arc_2();
     // lv_example_bar_1();
     // lv_example_bar_2();
     // lv_example_bar_3();
     // lv_example_bar_4();
     // lv_example_bar_5();
     // lv_example_bar_6();
     // lv_example_btn_1();
     // lv_example_btn_2();
     // lv_example_btn_3();
     // lv_example_btnmatrix_1();
     // lv_example_btnmatrix_2();
     // lv_example_btnmatrix_3();
     // lv_example_calendar_1();
     // lv_example_chart_1();
     // lv_example_chart_2();
     // lv_example_chart_3();
     // lv_example_chart_4();
     // lv_example_chart_5();
     // lv_example_chart_6();
     // lv_example_chart_7();
     // lv_example_chart_8();
     // lv_example_chart_9();
     // lv_example_checkbox_1();
     // lv_example_checkbox_2();
     // lv_example_dropdown_1();
     // lv_example_dropdown_2();
     // lv_example_dropdown_3();
     // lv_example_keyboard_1();
     // lv_example_label_1();
     // lv_example_label_2();
     // lv_example_label_3();
     // lv_example_label_4();
     // lv_example_label_5();
     // lv_example_line_1();
     // lv_example_list_1();
     // lv_example_list_2();
     // lv_example_msgbox_1();
     // lv_example_roller_1();
     // lv_example_roller_2();
     // lv_example_roller_3();
     // lv_example_slider_1();
     // lv_example_slider_2();
     // lv_example_slider_3();
     // lv_example_span_1();
     // lv_example_spinbox_1();
     // lv_example_spinner_1();
     // lv_example_switch_1();
     // lv_example_table_1();
     // lv_example_table_2();
     // lv_example_tabview_1();
     // lv_example_tabview_2();
     // lv_example_textarea_1();
     // lv_example_textarea_2();
     // lv_example_textarea_3();
     // lv_example_tileview_1();
     // lv_example_win_1();
   
     /*Or try out a demo. Don't forget to enable the demos in lv_conf.h. E.g. LV_USE_DEMO_WIDGETS*/
    ui_init();           
    // lv_demo_benchmark();          
    // lv_demo_keypad_encoder();     
    WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(OnDataRecv);
  xTaskCreate(StatusCheckTask, "StatusCheckTask", 2048, NULL, 1, &statusCheckTaskHandle);
  xTaskCreate(UpdateStatusesTask, "UpdateStatusesTask", 2048, NULL, 1, &updateStatusesTaskHandle);
}

void loop()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay( 5 );
}

void checkstatus() {
  long currentTime = millis();

  if (currentTime - lastStatusCheckTime >= STATUS_CHECK_INTERVAL) {
    lastStatusCheckTime = currentTime;  // Update the last check time

    // Check online status for each device and update variables
    updateDeviceStatus(DEVICE_1, device1Online, currentTime);
    updateDeviceStatus(DEVICE_2, device2Online, currentTime);
    updateDeviceStatus(DEVICE_3, device3Online, currentTime);
  }
}

void updateDeviceStatus(int deviceId, bool& isOnline, long currentTime) {
  if (lastHeartbeats.find(deviceId) == lastHeartbeats.end()) {
    // If the device is not in the map, consider it offline
    if (isOnline) {
      isOnline = false;
    }
  } else {
    long lastHeartbeat = lastHeartbeats[deviceId];
    bool shouldBeOnline = (currentTime - lastHeartbeat) <= TIMEOUT_DURATION;

    if (isOnline && !shouldBeOnline) {
      // Status changed from online to offline
      isOnline = false;
    } else if (!isOnline && shouldBeOnline) {
      // Status changed from offline to online
      isOnline = true;
    }
  }
}

void updateStatus(lv_obj_t* label, lv_obj_t* panel, bool currentStatus, bool& previousStatus) {
  if (currentStatus != previousStatus) {
    // Only update if there's a change in status
    if (currentStatus) {
      lv_obj_set_x(label, -10);
      lv_label_set_text(label, "online");
      lv_obj_set_style_bg_color(panel, lv_color_hex(0x26E868), LV_PART_MAIN | LV_STATE_DEFAULT);
    } else {
      lv_obj_set_x(label, -3);
      lv_label_set_text(label, "offline");
      lv_obj_set_style_bg_color(panel, lv_color_hex(0xEA2E50), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

    // Update the previous status to the current status
    previousStatus = currentStatus;
  }
}

// Update the statuses based on the current device states
void updateDeviceStatuses() {
  updateStatus(ui_status1, ui_Panel4, device1Online, previousDevice1Online);
  updateStatus(ui_status2, ui_Panel5, device2Online, previousDevice2Online);
  updateStatus(ui_status3, ui_Panel6, device3Online, previousDevice3Online);
}