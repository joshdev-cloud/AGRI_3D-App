/**
 * @file agri3d_ai.cpp
 * @brief Edge Impulse weed-detection inference implementation.
 *        Model: "Weed Detection" – EI project #923987, impulse v8.
 *
 * Inference is designed to run on Core 1 (the loop() / Brain task) so it
 * never blocks the Core 0 CommTask.  Raw camera frames are passed in as a
 * flat float array (RGB888 normalised 0-1) whose size must match
 * EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3.
 */

#include "agri3d_ai.h"
#include "agri3d_logger.h"
#include "agri3d_state.h"

// ── Edge Impulse SDK ────────────────────────────────────────────────────────
// Path is relative to the component root (AI-agri3d/).
// CMakeLists.txt and platformio.ini both add that root as an include dir.
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

// ── Label indices (must match model-parameters/model_variables.h) ──────────
// Labels are: 0 = "plant", 1 = "weed"  (order fixed by EI training)
#define EI_LABEL_PLANT 0
#define EI_LABEL_WEED  1

// ── Confidence threshold ────────────────────────────────────────────────────
static constexpr float WEED_CONFIDENCE_THRESHOLD = 0.65f;

// ── Static inference buffer (avoids heap fragmentation on ESP32) ────────────
static float ei_input_buf[EI_CLASSIFIER_INPUT_WIDTH *
                          EI_CLASSIFIER_INPUT_HEIGHT *
                          (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME / (EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT))];

// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief Callback used by ei_run_classifier to fill the feature buffer.
 *        We pre-fill ei_input_buf before calling run_classifier, so this
 *        just copies from that staging buffer.
 */
static int ei_get_data(size_t offset, size_t length, float *out_ptr) {
    memcpy(out_ptr, ei_input_buf + offset, length * sizeof(float));
    return 0;
}

// ────────────────────────────────────────────────────────────────────────────

void aiInit() {
    AgriLog(TAG_AI, LEVEL_INFO, "Edge Impulse Weed Detection initialised.");
    AgriLog(TAG_AI, LEVEL_INFO, "  Model input : %dx%d px",
            EI_CLASSIFIER_INPUT_WIDTH, EI_CLASSIFIER_INPUT_HEIGHT);
    AgriLog(TAG_AI, LEVEL_INFO, "  Label count : %d", EI_CLASSIFIER_LABEL_COUNT);
    AgriLog(TAG_AI, LEVEL_INFO, "  DSP blocks  : %d", EI_CLASSIFIER_DSP_BLOCKS_COUNT);
}

// ────────────────────────────────────────────────────────────────────────────

/**
 * @brief Run EI inference on a pre-scaled, normalised RGB float buffer.
 *
 * @param floatRGB  Pointer to float array of size
 *                  EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3.
 *                  Each channel value should be in the range [0.0, 1.0].
 * @param pixelCount  Number of pixels (width * height).
 */
AiResult aiRunInference(const float* floatRGB, size_t pixelCount) {
    AiResult res = { false, false, 0.0f, 0, 0 };

    if (floatRGB == nullptr || pixelCount == 0) {
        AgriLog(TAG_AI, LEVEL_WARN, "aiRunInference: null or empty buffer.");
        return res;
    }

    // ── 1. Copy into staging buffer ─────────────────────────────────────────
    const size_t totalFloats = pixelCount * 3;
    if (totalFloats > sizeof(ei_input_buf) / sizeof(float)) {
        AgriLog(TAG_AI, LEVEL_ERROR, "aiRunInference: buffer too large (%d floats, max %d).",
                totalFloats, sizeof(ei_input_buf) / sizeof(float));
        return res;
    }
    memcpy(ei_input_buf, floatRGB, totalFloats * sizeof(float));

    // ── 2. Build signal descriptor ──────────────────────────────────────────
    signal_t signal;
    signal.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    signal.get_data     = &ei_get_data;

    // ── 3. Run classifier ───────────────────────────────────────────────────
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, false);

    if (err != EI_IMPULSE_OK) {
        AgriLog(TAG_AI, LEVEL_ERROR, "run_classifier failed: error code %d", err);
        return res;
    }

    // ── 4. Parse results ────────────────────────────────────────────────────
#if EI_CLASSIFIER_OBJECT_DETECTION
    // Object-detection model: iterate bounding boxes
    for (size_t i = 0; i < result.bounding_boxes_count; i++) {
        auto& bb = result.bounding_boxes[i];
        if (bb.value < WEED_CONFIDENCE_THRESHOLD) continue;

        String label = String(bb.label);
        if (label == "weed") {
            res.foundWeed = true;
            res.confidence = bb.value;
            // Centre offset in pixels relative to image centre
            res.xOffset = (int)bb.x - (EI_CLASSIFIER_INPUT_WIDTH / 2);
            res.yOffset = (int)bb.y - (EI_CLASSIFIER_INPUT_HEIGHT / 2);
            AgriLog(TAG_AI, LEVEL_WARN,
                    "WEED DETECTED  conf=%.2f  offset=(%d,%d)",
                    res.confidence, res.xOffset, res.yOffset);
            break; // Act on highest-confidence detection only
        } else if (label == "plant") {
            res.foundPlant = true;
        }
    }
#else
    // Classification model: pick highest scoring label
    float maxVal = 0.0f;
    int   maxIdx = -1;
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        if (result.classification[i].value > maxVal) {
            maxVal = result.classification[i].value;
            maxIdx = (int)i;
        }
    }
    res.confidence = maxVal;
    if (maxIdx == EI_LABEL_WEED && maxVal >= WEED_CONFIDENCE_THRESHOLD) {
        res.foundWeed = true;
        AgriLog(TAG_AI, LEVEL_WARN, "WEED DETECTED  conf=%.2f", maxVal);
    } else if (maxIdx == EI_LABEL_PLANT) {
        res.foundPlant = true;
        AgriLog(TAG_AI, LEVEL_INFO, "Plant detected  conf=%.2f", maxVal);
    }
#endif

    return res;
}

// ── Legacy shim kept for backward compatibility with camera routine ──────────
AiResult aiAnalyzeFrame(uint8_t* buf, size_t len) {
    AiResult res = { false, false, 0.0f, 0, 0 };
    if (buf == nullptr || len == 0) return res;

    // This stub exists so existing callers compile without changes.
    // Full JPEG-decode + resize pipeline should be implemented in the
    // camera routine (agri3d_camera.cpp) before calling aiRunInference().
    AgriLog(TAG_AI, LEVEL_WARN,
            "aiAnalyzeFrame stub called – implement JPEG decode + aiRunInference().");
    return res;
}

