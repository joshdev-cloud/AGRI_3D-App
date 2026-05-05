/**
 * @file agri3d_ai.h
 * @brief Edge Impulse weed-detection inference API.
 *        Model: "Weed Detection" – EI project #923987, impulse v8.
 *
 * Two entry points are provided:
 *   1. aiRunInference()  – preferred; takes a pre-scaled float RGB buffer.
 *   2. aiAnalyzeFrame()  – legacy shim; kept for backward compatibility.
 */

#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

// ── Result struct ───────────────────────────────────────────────────────────

/** Result of an Edge Impulse inference pass. */
struct AiResult {
    bool  foundPlant;   ///< True if a plant label exceeded threshold
    bool  foundWeed;    ///< True if a weed label exceeded threshold
    float confidence;   ///< Score of the winning label [0.0 – 1.0]
    int   xOffset;      ///< Pixel offset from image centre-X (object detection only)
    int   yOffset;      ///< Pixel offset from image centre-Y (object detection only)
};

// ── Public API ──────────────────────────────────────────────────────────────

/**
 * @brief Initialise the Edge Impulse engine.
 *        Logs model dimensions and label count.  Must be called in setup().
 */
void aiInit();

/**
 * @brief Run weed-detection inference on a normalised float RGB buffer.
 *
 * The caller is responsible for:
 *   1. Decoding the JPEG frame from the camera.
 *   2. Resizing it to EI_CLASSIFIER_INPUT_WIDTH × EI_CLASSIFIER_INPUT_HEIGHT.
 *   3. Normalising each channel to [0.0, 1.0].
 *
 * @param floatRGB   Flat float array [R, G, B, R, G, B, …] of length pixelCount * 3.
 * @param pixelCount Number of pixels (width × height).
 * @return AiResult  Inference result.
 */
AiResult aiRunInference(const float* floatRGB, size_t pixelCount);

/**
 * @brief Legacy shim – kept for backward compatibility.
 *        Full JPEG decode + resize should be done in agri3d_camera.cpp
 *        before calling aiRunInference() directly.
 * @param buf Pointer to raw JPEG data.
 * @param len Length of JPEG data in bytes.
 */
AiResult aiAnalyzeFrame(uint8_t* buf, size_t len);
