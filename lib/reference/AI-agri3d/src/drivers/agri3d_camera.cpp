/**
 * @file agri3d_camera.cpp
 * @brief Camera init, FPM stream task, and captureFrameAtPosition()
 * implementation.
 */

#include "agri3d_camera.h"
#include "agri3d_config.h"
#include "agri3d_grbl.h"
#include "agri3d_network.h"
#include "agri3d_sd.h"
#include "agri3d_state.h"
#include "../core/agri3d_logger.h"
#include "../core/agri3d_ai.h"      // Edge Impulse inference API
#include <ArduinoJson.h>
#include <math.h>
#include <time.h>

// EI model input dimensions — resolved at compile time from model_variables.h
// (included transitively via agri3d_ai.h → ei_run_classifier.h)
#ifndef EI_CLASSIFIER_INPUT_WIDTH
  #define EI_CLASSIFIER_INPUT_WIDTH  96
  #define EI_CLASSIFIER_INPUT_HEIGHT 96
#endif

// ============================================================================
// CAMERA INITIALISATION
// ============================================================================

bool cameraInit() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_Y2;
  config.pin_d1 = CAM_Y3;
  config.pin_d2 = CAM_Y4;
  config.pin_d3 = CAM_Y5;
  config.pin_d4 = CAM_Y6;
  config.pin_d5 = CAM_Y7;
  config.pin_d6 = CAM_Y8;
  config.pin_d7 = CAM_Y9;
  config.pin_xclk = CAM_XCLK;
  config.pin_pclk = CAM_PCLK;
  config.pin_vsync = CAM_VSYNC;
  config.pin_href = CAM_HREF;
  config.pin_sccb_sda = CAM_SIOD;
  config.pin_sccb_scl = CAM_SIOC;
  config.pin_pwdn = CAM_PWDN;
  config.pin_reset = CAM_RESET;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // UXGA (1600×1200) — highest quality for plant-map stills
  // Requires PSRAM to be enabled in board settings
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10; // 0-63, lower = higher quality
    config.fb_count = 2;      // Double buffer for smoother stream
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    // Fall back to QVGA if no PSRAM to prevent DRAM exhaustion!
    config.frame_size = FRAMESIZE_QVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    AgriLog(TAG_CAM, LEVEL_WARN, "No PSRAM — falling back to QVGA to save memory");
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    AgriLog(TAG_CAM, LEVEL_ERR, "Init failed (0x%x)", err);
    return false;
  }

  // Fine-tune sensor settings for agricultural (outdoor) scenes
  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0); // -2 to 2
    s->set_contrast(s, 0);   // -2 to 2
    s->set_saturation(s, 1); // Boost green for vegetation
    s->set_sharpness(s, 1);
    s->set_whitebal(s, 1);      // Auto white balance on
    s->set_awb_gain(s, 1);      // AWB gain on
    s->set_exposure_ctrl(s, 1); // Auto exposure on
    s->set_aec2(s, 1);          // Better AEC algorithm
  }

  AgriLog(TAG_CAM, LEVEL_SUCCESS, "Camera OK (UXGA JPEG)");

  // Launch stream task on Core 0 to offload it from the main network/grbl loop on Core 1
  xTaskCreatePinnedToCore(streamTask, "streamTask", 8192, nullptr, 2, nullptr,
                          0);
  return true;
}

// ============================================================================
// FPM STREAM TASK
// ============================================================================

void streamTask(void * /*pvParameters*/) {
  while (true) {
    // Only stream if: flag set, camera not locked, and a client is connected
    if (sysState.isStreaming() && isCameraAvailable() &&
        sysState.getFlutter() == FLUTTER_CONNECTED) {

      // CONGESTION CONTROL: If the network task hasn't finished sending the previous
      // frame, don't even capture a new one. This keeps the heap clean and the CPU cool.
      if (sysState.pendingFrame != nullptr) {
        vTaskDelay(pdMS_TO_TICKS(10)); // Busy wait/throttle
        continue;
      }

      camera_fb_t *fb = esp_camera_fb_get();
      if (!fb) {
        vTaskDelay(pdMS_TO_TICKS(100));
        continue;
      }

      // ZERO-COPY HANDOFF: 
      // We pass the RAW pointer from the camera driver to the NetworkTask (Core 0).
      // The NetworkTask is responsible for calling esp_camera_fb_return(fb).
      sysState.pendingFrameFB = fb; 
      sysState.pendingFrame = fb->buf;
      sysState.pendingFrameLen = fb->len;
      sysState.pendingFrameClient = activeClientNum;

      // Convert FPM to delay: delay_ms = 60000 / fpm
      uint32_t delayMs = 60000UL / (uint32_t)sysState.getFpm();
      vTaskDelay(pdMS_TO_TICKS(delayMs));

    } else {
      vTaskDelay(pdMS_TO_TICKS(STREAM_IDLE_DELAY));
    }
  }
}

// ============================================================================
// CORE CAPTURE HELPER
// ============================================================================

bool captureFrameAtPosition(uint8_t clientNum, int idx, int total,
                            float targetX, float targetY) {
  // 1. Move to position
  char gcode[48];
  snprintf(gcode, sizeof(gcode), "G0 X%.2f Y%.2f F%d", targetX, targetY,
           GRBL_DEFAULT_FEEDRATE);
  enqueueGrblCommand(gcode);
  
  if (!waitForGrblIdle(SCAN_MOVE_TIMEOUT_MS)) return false;

  // 2. Request a high-quality frame
  // We tell Core 0 to stop streaming and give us a dedicated shot
  sysState.setStreaming(false);
  delay(200); // Wait for stream task to yield

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;

  // 3. Metadata & SD Save
  time_t now = time(nullptr);
  char dateStr[12], timeStr[10];
  struct tm *tm_ = localtime(&now);
  snprintf(dateStr, sizeof(dateStr), "%04d-%02d-%02d", tm_->tm_year + 1900, tm_->tm_mon + 1, tm_->tm_mday);
  snprintf(timeStr, sizeof(timeStr), "%02d:%02d:%02d", tm_->tm_hour, tm_->tm_min, tm_->tm_sec);

  char sdPath[96] = {0};
#if HW_SD_CONNECTED
  sdSaveImage(fb->buf, fb->len, SD_IMG_PLANTMAP, 'f', idx, targetX, targetY, now, sdPath);
#endif

  // 4. Handoff to Core 0 for transmission
  if (sysState.getFlutter() == FLUTTER_CONNECTED) {
    StaticJsonDocument<256> meta;
    meta["evt"] = "FRAME_META";
    meta["idx"] = idx;
    meta["total"] = total;
    meta["x"] = targetX;
    meta["y"] = targetY;
    meta["sdPath"] = sdPath;
    String metaStr; serializeJson(meta, metaStr);
    webSocket.sendTXT(clientNum, metaStr);

    // Use our new Zero-Copy bridge
    sysState.pendingFrameFB = fb;
    sysState.pendingFrame = fb->buf;
    sysState.pendingFrameLen = fb->len;
    sysState.pendingFrameClient = clientNum;
  } else {
    esp_camera_fb_return(fb);
  }

  AgriLog(TAG_CAM, LEVEL_INFO, "Frame %d: %s %s (%.1f,%.1f) %uB%s", idx, dateStr,
                timeStr, targetX, targetY, fb->len,
                sdPath[0] ? " [SD]" : "");

  // ── AI Weed Detection ────────────────────────────────────────────────────
  // Run Edge Impulse inference on the captured frame and broadcast the result.
  aiProcessFrame(fb->buf, fb->len, targetX, targetY);
  // ─────────────────────────────────────────────────────────────────────────

  return true;
}

// ============================================================================
// AI FRAME PROCESSING PIPELINE
// JPEG → RGB decode → nearest-neighbour resize → float normalise → EI infer
// ============================================================================

// ── JPEG decode callback state ───────────────────────────────────────────────
struct JpegDecodeCtx {
    uint8_t* rgbBuf;    // Output: raw RGB888 pixels (malloc'd by caller)
    uint32_t width;     // Decoded frame width
    uint32_t height;    // Decoded frame height
    bool     ok;
};

// Called by esp_jpg_decode for each decoded MCU block.
static bool jpegDecodeCallback(void* arg, uint16_t x, uint16_t y,
                               uint16_t w, uint16_t h, uint8_t* data) {
    JpegDecodeCtx* ctx = (JpegDecodeCtx*)arg;
    if (!ctx || !data) return false;

    for (uint16_t row = 0; row < h; row++) {
        for (uint16_t col = 0; col < w; col++) {
            uint32_t srcIdx  = (row * w + col) * 3;               // RGB888 from decoder
            uint32_t dstPx   = (y + row) * ctx->width + (x + col);
            uint32_t dstIdx  = dstPx * 3;
            ctx->rgbBuf[dstIdx + 0] = data[srcIdx + 0]; // R
            ctx->rgbBuf[dstIdx + 1] = data[srcIdx + 1]; // G
            ctx->rgbBuf[dstIdx + 2] = data[srcIdx + 2]; // B
        }
    }
    ctx->ok = true;
    return true;
}

void aiProcessFrame(const uint8_t* jpegBuf, size_t jpegLen,
                    float gantryX, float gantryY) {
    if (!jpegBuf || jpegLen == 0) return;

    // ── Step 1: Decode JPEG → raw RGB888 ────────────────────────────────────
    // esp_jpg_decode needs to know width/height before the pixel callback fires.
    // We use a small first-pass decode at JPEG_IMAGE_RGB888 to get dimensions.
    uint32_t srcW = 0, srcH = 0;
    esp_err_t dimErr = esp_jpeg_get_image_info(jpegBuf, jpegLen, &srcW, &srcH, nullptr);
    if (dimErr != ESP_OK || srcW == 0 || srcH == 0) {
        AgriLog(TAG_AI, LEVEL_WARN, "aiProcessFrame: JPEG info failed (0x%x)", dimErr);
        return;
    }

    // Allocate RGB buffer in PSRAM if available (srcW * srcH * 3 bytes)
    size_t rgbSize = srcW * srcH * 3;
    uint8_t* rgbBuf = (uint8_t*)(psramFound()
        ? heap_caps_malloc(rgbSize, MALLOC_CAP_SPIRAM)
        : malloc(rgbSize));
    if (!rgbBuf) {
        AgriLog(TAG_AI, LEVEL_ERROR, "aiProcessFrame: RGB malloc failed (%u bytes)", rgbSize);
        return;
    }

    JpegDecodeCtx ctx = { rgbBuf, srcW, srcH, false };
    esp_err_t decErr = esp_jpg_decode(jpegBuf, jpegLen, JPG_SCALE_NONE,
                                      jpegDecodeCallback, &ctx,
                                      JPEG_IMAGE_RGB888);
    if (decErr != ESP_OK || !ctx.ok) {
        AgriLog(TAG_AI, LEVEL_WARN, "aiProcessFrame: JPEG decode failed (0x%x)", decErr);
        free(rgbBuf);
        return;
    }

    // ── Step 2: Resize to EI model input (nearest-neighbour) ─────────────────
    const uint32_t dstW = EI_CLASSIFIER_INPUT_WIDTH;
    const uint32_t dstH = EI_CLASSIFIER_INPUT_HEIGHT;
    const size_t   floatCount = dstW * dstH * 3;

    float* floatBuf = (float*)(psramFound()
        ? heap_caps_malloc(floatCount * sizeof(float), MALLOC_CAP_SPIRAM)
        : malloc(floatCount * sizeof(float)));
    if (!floatBuf) {
        AgriLog(TAG_AI, LEVEL_ERROR, "aiProcessFrame: float malloc failed");
        free(rgbBuf);
        return;
    }

    for (uint32_t row = 0; row < dstH; row++) {
        uint32_t srcRow = (uint32_t)((float)row / dstH * srcH);
        for (uint32_t col = 0; col < dstW; col++) {
            uint32_t srcCol  = (uint32_t)((float)col / dstW * srcW);
            uint32_t srcIdx  = (srcRow * srcW + srcCol) * 3;
            uint32_t dstIdx  = (row * dstW + col) * 3;
            // Normalise to [0.0, 1.0]
            floatBuf[dstIdx + 0] = rgbBuf[srcIdx + 0] / 255.0f;
            floatBuf[dstIdx + 1] = rgbBuf[srcIdx + 1] / 255.0f;
            floatBuf[dstIdx + 2] = rgbBuf[srcIdx + 2] / 255.0f;
        }
    }
    free(rgbBuf); // No longer needed

    // ── Step 3: Run EI inference ──────────────────────────────────────────────
    AiResult result = aiRunInference(floatBuf, dstW * dstH);
    free(floatBuf);

    // ── Step 4: Broadcast result to Flutter via WebSocket ─────────────────────
    StaticJsonDocument<256> doc;
    doc["evt"]        = "AI_RESULT";
    doc["x"]          = gantryX;
    doc["y"]          = gantryY;
    doc["weed"]       = result.foundWeed;
    doc["plant"]      = result.foundPlant;
    doc["confidence"] = round(result.confidence * 100.0f) / 100.0f; // 2 d.p.
    if (result.foundWeed) {
        doc["offset_x"] = result.xOffset;
        doc["offset_y"] = result.yOffset;
    }
    String out;
    serializeJson(doc, out);
    webSocket.broadcastTXT(out);

    AgriLog(TAG_AI, LEVEL_INFO,
            "AI @ (%.1f,%.1f): weed=%d plant=%d conf=%.2f",
            gantryX, gantryY, result.foundWeed, result.foundPlant, result.confidence);
}

void cameraSanityCheck() {
    // If the system has been in an "invalid" state for too long
    static unsigned long lastSuccess = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb != NULL) {
        lastSuccess = millis();
        esp_camera_fb_return(fb);
    }
    
    if (millis() - lastSuccess > 10000) { 
        AgriLog(TAG_CAM, LEVEL_ERR, "[WATCHDOG] Camera unresponsive for 10s. Attempting reset...");
        esp_camera_deinit();
        delay(100);
        cameraInit(); // Re-init
        lastSuccess = millis();
    }
}
