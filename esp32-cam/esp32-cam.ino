#include "esp_camera.h"
#include <esp_heap_caps.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include "model.h"
#include <TensorFlowLite_ESP32.h>
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"

#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#include "camera_pins.h"

camera_fb_t *frameBuffer; // 프레임 버퍼
float *inputImage;

const tflite::Model* model;
tflite::MicroInterpreter* interpreter;
tflite::ErrorReporter* error_reporter;
TfLiteTensor* input_tensor;
TfLiteTensor* output_tensor;

#define TENSOR_ARENA_SIZE (1024*1024) // 1MB Tensor Arena
uint8_t *tensorArena; // Tensor Arena

void setupLedFlash(int pin);

// UART 설정
HardwareSerial mySerial(1); // ESP32-CAM의 UART1 사용

void setup() {
  //Serial.begin(115200);
  mySerial.begin(115200, SERIAL_8N1, 3, 1); // ESP32-CAM의 TX=1, RX=3 핀 활성화, 즉 STM32 보드의 UART 핀에 연결되는 핀

  // 메모리 동적 할당
  inputImage = (float*)malloc(96 * 96 * 3 * sizeof(float));
  tensorArena = (uint8_t*)malloc(TENSOR_ARENA_SIZE);

  if (!inputImage || !tensorArena) {
    //Serial.println("Failed to allocate memory for PSRAM");
    while (1);
  }

  // 부팅 로그 비활성화
  mySerial.flush();
  esp_log_level_set("*", ESP_LOG_NONE);

  // UART 버퍼 초기화
  while (Serial.available() > 0) { Serial.read(); }

  model = tflite::GetModel(model_tflite);

// ErrorReporter 초기화
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  // AllOpsResolver 사용
  static tflite::AllOpsResolver resolver;

  // Interpreter 초기화
  static tflite::MicroInterpreter static_interpreter(
      model, resolver, tensorArena, TENSOR_ARENA_SIZE, error_reporter);
  interpreter = &static_interpreter;

  // Allocate tensor
  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
      //Serial.println("AllocateTensors() failed");
      return;
  }

  // 입력 및 출력 텐서 포인터 가져오기
  input_tensor = interpreter->input(0);
  output_tensor = interpreter->output(0);

  //Serial.println("Model initialized successfully!");

  // 카메라 설정
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA; // 해상도 설정 (QVGA = 320x240)
  config.pixel_format = PIXFORMAT_JPEG; // JPEG 포맷 설정
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 20; // JPEG 품질 설정 (0: 최고 품질, 63: 최저 품질)
  config.fb_count = 1; // 프레임 버퍼 개수 설정

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 15;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      //config.frame_size = FRAMESIZE_SVGA;
      config.frame_size = FRAMESIZE_240X240;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
    #if CONFIG_IDF_TARGET_ESP32S3
        config.fb_count = 2;
    #endif
  }

  #if defined(CAMERA_MODEL_ESP_EYE)
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif

    // 카메라 초기화
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      //Serial.printf("Camera init failed with error 0x%x", err);
      return;
    }

    sensor_t *s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1);        // flip it back
      s->set_brightness(s, 1);   // up the brightness just a bit
      s->set_saturation(s, -2);  // lower the saturation
    }
    // drop down frame size for higher initial frame rate
    if (config.pixel_format == PIXFORMAT_JPEG) {
      s->set_framesize(s, FRAMESIZE_QVGA);
    }

  #if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
  #endif

  #if defined(CAMERA_MODEL_ESP32S3_EYE)
    s->set_vflip(s, 1);
  #endif

  // Setup LED FLash if LED pin is defined in camera_pins.h
  #if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
  #endif
}

void preprocessImage(camera_fb_t *frame){
  uint8_t *imageData = frame->buf; // 이미지 데이터 버퍼
  int srcWidth = 320, srcHeight = 240;
  int destWidth = 96, destHeight = 96;

  for (int y = 0; y < destHeight; y++){
    for (int x = 0; x < destWidth; x++){
      int srcX = x * srcWidth / destWidth; // 원본 해상도
      int srcY = y * srcHeight / destHeight; // 모델 입력 해상도

      int srcIndex = (srcY * srcWidth + srcX) * 2; 
      uint16_t pixel = (imageData[srcIndex] << 8) | imageData[srcIndex + 1];
      uint8_t r = (pixel >> 11) & 0x1F; // 상위 5비트 (Red)
      uint8_t g = (pixel >> 5) & 0x3F;  // 중간 6비트 (Green)
      uint8_t b = pixel & 0x1F;         // 하위 5비트 (Blue)


      // 정규화 및 입력 배열 저장 (0-1 범위로 정규화)
      inputImage[(y * destWidth + x) * 3 + 0] = (float)r / 31.0; // R
      inputImage[(y * destWidth + x) * 3 + 1] = (float)g / 63.0; // G
      inputImage[(y * destWidth + x) * 3 + 2] = (float)b / 31.0; // B
    }
  }
}

void computeSoftmax(float* logits, int num_classes, float* probabilities) {
  float max_logit = logits[0];
  for (int i = 1; i < num_classes; i++) {
    if (logits[i] > max_logit) max_logit = logits[i];
  }

  float sum_exp = 0.0;
  for (int i = 0; i < num_classes; i++) {
    probabilities[i] = exp(logits[i] - max_logit);
    sum_exp += probabilities[i];
  }

  for (int i = 0; i < num_classes; i++) {
    probabilities[i] /= sum_exp;
  }
}

void runInference(){
  frameBuffer = esp_camera_fb_get();

  if(!frameBuffer){
    Serial.println("Failed to capture frame");
    return;
  }

  preprocessImage(frameBuffer);

  for(int i = 0; i < 96 * 96 * 3; i++){
    input_tensor->data.uint8[i] = (uint8_t)(inputImage[i] / 0.00390625); // Scale = 0.00390625
  }

  //모델 추론 실행
  if(interpreter->Invoke() != kTfLiteOk){
    Serial.println("Invoke failed");
    return;
  }

  // Softmax를 사용하여 확률 계산
  float logits[2];
  for (int i = 0; i < 2; i++) {
    logits[i] = (output_tensor->data.uint8[i] - output_tensor->params.zero_point) * output_tensor->params.scale;
  }

  float probabilities[2];
  computeSoftmax(logits, 2, probabilities);

  mySerial.printf("%.2f", probabilities[1]);

  esp_camera_fb_return(frameBuffer);
}
// ===========================
// Main Loop Function
// ===========================
void loop() {
  runInference();
  delay(10);
}
