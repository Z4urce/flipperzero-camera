#include "esp_camera.h"

// Camera Config - change this if you have different camera model.
// Reference: https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h
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

// Target device config
#define TARGET_WIDTH     128
#define TARGET_HEIGHT     64

// First time setup
void setup() {
  Serial.begin(230400);

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
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_GRAYSCALE;

  // We need the smallest frame size
  config.frame_size = FRAMESIZE_QQVGA;
  config.fb_count = 1;

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Setting high contrast to make easier to dither
  sensor_t * s = esp_camera_sensor_get();
  s->set_contrast(s, 2);
}

// Global Variables
bool stop_stream = true;
bool disable_dithering = false;
bool invert = false;
uint8_t bit_depth = 1;

void loop() {
  execute_uart_command();  

  if (stop_stream){
    return;
  }

  send_frame_though_uart();
  delay(50);
}

void send_frame_though_uart() {
  camera_fb_t* fb = esp_camera_fb_get();

  if (!fb) {
    return;
  }
  
  //Length: 19200
  //Width: 160
  //Height: 120
  //Format: 2
  //Target: 128x64

  if (!disable_dithering) {
    dither_image(fb);
  }

  const uint16_t x_min = (fb->width - TARGET_WIDTH) / 2;
  const uint16_t x_max = fb->width - x_min;
  const uint16_t y_min = (fb->height - TARGET_HEIGHT) / 2;
  const uint16_t y_max = fb->height - y_min;

  uint8_t flipper_y = 0;
  for(uint16_t y = y_min; y < y_max; ++y) { // For each row
    Serial.print("Y");
    Serial.print((int)bit_depth);
    Serial.print((char)flipper_y);

    send_row_though_uart(fb, x_min, x_max, y * fb->width);

    ++flipper_y;
    Serial.flush();
  }

  esp_camera_fb_return(fb);
  fb = NULL;
}


void send_row_though_uart(camera_fb_t* fb, const uint16_t x_min, const uint16_t x_max, size_t y) {
  uint8_t increment = 8 / bit_depth;
  for (uint16_t x = x_min; x < x_max; x+=increment){  // For each column in given row (skipping 4 a time)
      char c = 0;
      for(uint8_t j = 0; j < increment; ++j){ // For 8 bits in 4 adjacent pixels // 0 = 00, 85 = 01, 170 = 10, 255 = 11
        uint8_t color = fb->buf[y + x + ((increment-1)-j)];
        for(uint8_t b = 0; b < bit_depth; ++b){
          if (is_bit_active(color, b)){
            c |= 1 << ((j*bit_depth)+b);
          }
        }
      }
      Serial.print(c);
      yield();
  }
}

bool is_bit_active(uint8_t color, uint8_t bit){
  return handle_invert(convert_to_binary(color, bit));
}

bool convert_to_binary(uint8_t color, uint8_t bit){
  if (bit_depth == 1) {     // Save performance if it's only 1 bit
      return color >= 128;
  }
  else if (color == 255){   // If it is pure white, all bites will be ones
    return 1;
  }
  else if (color == 0){     // If it is pure black, all bites will be zeros
    return 0;
  }
  
  return (find_closest_palette_color(color) & (1 << bit));
}

bool handle_invert(bool input){
  return invert ? !input : input;
}

void dither_image(camera_fb_t* fb) {
  for(uint8_t y = 0; y < fb->height; ++y){
      for (uint8_t x = 0; x < fb->width; ++x){
        size_t current = (y*fb->width) + x;
        uint8_t oldpixel = fb->buf[current];
        uint8_t newpixel = find_closest_palette_color(oldpixel);
        fb->buf[current] = newpixel;
        uint8_t quant_error = oldpixel - newpixel;
        fb->buf[(y*fb->width) + x + 1] = fb->buf[(y*fb->width) + x + 1] + quant_error * 7 / 16;
        fb->buf[(y+1*fb->width) + x-1] = fb->buf[(y+1*fb->width) + x-1] + quant_error * 3 / 16;
        fb->buf[(y + 1*fb->width) + x] = fb->buf[(y + 1*fb->width) + x] + quant_error * 5 / 16;
        fb->buf[(y+1*fb->width) + x+1] = fb->buf[(y+1*fb->width) + x+1] + quant_error * 1 / 16;
      }
    }  
}

uint8_t find_closest_palette_color(uint8_t color){
  if (bit_depth == 1){
    return color >= 128 ? 255 : 0;
  }
  if (color == 0 || color == 255){
    return color;
  }

  uint8_t shades = (1 << bit_depth) - 1;
  uint8_t shade = 256 / shades;
  for (uint8_t i = 0; i < shades; ++i){
      if (color <= (i+1)*shade){
        return round_to_closest(color, i*shade, (i+1)*shade);
      }
  }

  return 0;
}

uint8_t round_to_closest(uint8_t color, uint8_t a, uint8_t b){
  return (color <= ((a + b) / 2)) ? a : b;
}

void execute_uart_command(){
  if (Serial.available() > 0) {
    char r = Serial.read();
    sensor_t * s = esp_camera_sensor_get();
    
    switch(r) {
      case 'S':
        stop_stream = false;
        break;
      case 's':
        stop_stream = true;
        break;
      case 'D':
        disable_dithering = false;
        break;
      case 'd':
        disable_dithering = true;
        break;
      case 'C':
        s->set_contrast(s, s->status.contrast + 1);
        break;
      case 'c':
        s->set_contrast(s, s->status.contrast - 1);
        break;
      case 'B':
        s->set_contrast(s, s->status.brightness + 1);
        break;
      case 'b':
        s->set_contrast(s, s->status.brightness - 1);
        break;
      case 'Z':
        s->set_framesize(s, FRAMESIZE_HQVGA);
        break;
      case 'z':
        s->set_framesize(s, FRAMESIZE_QQVGA);

      // Toggle cases
      case 'M': // Toggle Mirror
        s->set_hmirror(s, !s->status.hmirror);
        break;
      case '>':
        disable_dithering = !disable_dithering;
        break;
      case '<':
        invert = !invert;
      case '+':
        s->set_framesize(s, (s->status.framesize == FRAMESIZE_QQVGA) ? FRAMESIZE_HQVGA : FRAMESIZE_QQVGA);
      case '*':
        bit_depth = bit_depth == 1 ? 2 : 1;
      default:
        break;
    }
  }
}