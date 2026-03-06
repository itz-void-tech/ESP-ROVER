#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"

const char* ssid = "ESP32_CAM";
const char* password = "12345678";

/* CAMERA PINS */
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5

#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

static httpd_handle_t camera_httpd = NULL;

/* HTML PAGE */
const char INDEX_HTML[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>
<head>

<meta name="viewport" content="width=device-width, initial-scale=1">

<style>

body{
margin:0;
background:black;
display:flex;
justify-content:center;
align-items:center;
height:100vh;
}

img{
width:100vw;
height:auto;
}

</style>

</head>

<body>

<img src="/stream">

</body>

</html>

)rawliteral";


/* WEB PAGE HANDLER */
static esp_err_t index_handler(httpd_req_t *req){

httpd_resp_set_type(req,"text/html");
return httpd_resp_send(req,INDEX_HTML,strlen(INDEX_HTML));

}


/* STREAM HANDLER */
static esp_err_t stream_handler(httpd_req_t *req){

camera_fb_t * fb = NULL;

esp_err_t res = ESP_OK;

res = httpd_resp_set_type(req,"multipart/x-mixed-replace; boundary=frame");

while(true){

fb = esp_camera_fb_get();

if(!fb){
continue;
}

httpd_resp_send_chunk(req,"--frame\r\n",9);
httpd_resp_send_chunk(req,"Content-Type: image/jpeg\r\n\r\n",28);
httpd_resp_send_chunk(req,(const char*)fb->buf,fb->len);
httpd_resp_send_chunk(req,"\r\n",2);

esp_camera_fb_return(fb);

delay(35);   // reduce power spikes

}

return res;

}


void startCameraServer(){

httpd_config_t config = HTTPD_DEFAULT_CONFIG();

httpd_uri_t index_uri = {
.uri="/",
.method=HTTP_GET,
.handler=index_handler,
.user_ctx=NULL
};

httpd_uri_t stream_uri = {
.uri="/stream",
.method=HTTP_GET,
.handler=stream_handler,
.user_ctx=NULL
};

httpd_start(&camera_httpd,&config);

httpd_register_uri_handler(camera_httpd,&index_uri);

httpd_register_uri_handler(camera_httpd,&stream_uri);

}


void setup(){

Serial.begin(115200);

/* WiFi AP */

WiFi.mode(WIFI_AP);

WiFi.softAP(ssid,password);

/* Reduce power spikes */

WiFi.setTxPower(WIFI_POWER_8_5dBm);

/* CAMERA CONFIG */

camera_config_t config;

config.ledc_channel = LEDC_CHANNEL_0;
config.ledc_timer = LEDC_TIMER_0;

config.pin_d0 = Y2_GPIO_NUM;
config.pin_d1 = Y3_GPIO_NUM;
config.pin_d2 = Y4_GPIO_NUM;
config.pin_d3 = Y5_GPIO_NUM;
config.pin_d4 = Y6_GPIO_NUM;
config.pin_d5 = Y7_GPIO_NUM;
config.pin_d6 = Y8_GPIO_NUM;
config.pin_d7 = Y9_GPIO_NUM;

config.pin_xclk = XCLK_GPIO_NUM;
config.pin_pclk = PCLK_GPIO_NUM;
config.pin_vsync = VSYNC_GPIO_NUM;
config.pin_href = HREF_GPIO_NUM;

config.pin_sscb_sda = SIOD_GPIO_NUM;
config.pin_sscb_scl = SIOC_GPIO_NUM;

config.pin_pwdn = PWDN_GPIO_NUM;
config.pin_reset = RESET_GPIO_NUM;

config.xclk_freq_hz = 10000000;

config.pixel_format = PIXFORMAT_JPEG;

config.frame_size = FRAMESIZE_VGA;

config.jpeg_quality = 15;

config.fb_count = 1;

esp_camera_init(&config);

/* FIX ORIENTATION */

sensor_t * s = esp_camera_sensor_get();

s->set_vflip(s,1);

s->set_hmirror(s,1);

startCameraServer();

Serial.print("Camera Ready: ");
Serial.println(WiFi.softAPIP());

}


void loop(){

delay(1000);

}
