cmake_minimum_required(VERSION 3.16)

# ── Edge Impulse SDK Sources ─────────────────────────────────────────────────
# Collect all EI SDK .cpp files recursively, then remove platform-specific
# sources that are not compatible with the ESP32 Xtensa toolchain.

file(GLOB_RECURSE EI_SDK_SOURCES
    "edge-impulse-sdk/dsp/*.cpp"
    "edge-impulse-sdk/classifier/*.cpp"
    "edge-impulse-sdk/tensorflow/lite/c/*.c"
    "edge-impulse-sdk/tensorflow/lite/core/*.cpp"
    "edge-impulse-sdk/tensorflow/lite/kernels/*.cpp"
    "edge-impulse-sdk/tensorflow/lite/micro/*.cpp"
    "edge-impulse-sdk/tensorflow/lite/micro/kernels/*.cpp"
    "edge-impulse-sdk/third_party/flatbuffers/src/*.cpp"
    "edge-impulse-sdk/third_party/gemmlowp/*.cpp"
    "edge-impulse-sdk/third_party/ruy/*.cpp"
)

# Espressif porting layer (esp32-specific timing / debug_log / memory)
file(GLOB EI_PORTING_SOURCES
    "edge-impulse-sdk/porting/espressif/*.cpp"
)

# tflite-model compiled inference graph
file(GLOB EI_MODEL_SOURCES
    "tflite-model/*.cpp"
)

# ── ESP-IDF / PlatformIO Component Registration ─────────────────────────────
idf_component_register(
    SRCS
        # ── AI-agri3d application sources ──────────────────────────────────
        "src/core/agri3d_ai.cpp"
        "src/core/agri3d_state.cpp"
        "src/core/agri3d_commands.cpp"
        "src/agri3d_network.cpp"
        "src/agri3d_grbl.cpp"
        "src/agri3d_camera.cpp"
        "src/agri3d_plant_map.cpp"
        "src/agri3d_npk.cpp"
        "src/agri3d_routine.cpp"
        "src/agri3d_environment.cpp"
        "src/agri3d_sd.cpp"
        # ── Edge Impulse SDK ───────────────────────────────────────────────
        ${EI_SDK_SOURCES}
        ${EI_PORTING_SOURCES}
        ${EI_MODEL_SOURCES}
    INCLUDE_DIRS
        # ── Project headers ────────────────────────────────────────────────
        "src"
        "src/core"
        # ── EI SDK root (enables: #include "edge-impulse-sdk/...") ─────────
        "."
        # ── Model-specific parameter headers ──────────────────────────────
        "model-parameters"
        # ── TFLite model compiled header ───────────────────────────────────
        "tflite-model"
    REQUIRES
        esp32-camera
        arduino-esp32
        ArduinoJson
        WebSockets
        SD_MMC
        mdns
        esp_wifi
        esp_http_client
)

# ── EI Build Defines ─────────────────────────────────────────────────────────
# EI_PORTING_ESPRESSIF  – selects the Espressif porting layer inside EI SDK
# TF_LITE_STATIC_MEMORY – disables dynamic allocation in TFLite Micro
# EIDSP_QUANTIZE_FILTERBANK=0 – keep float filterbank for better accuracy
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    EI_PORTING_ESPRESSIF=1
    TF_LITE_STATIC_MEMORY=1
    EIDSP_QUANTIZE_FILTERBANK=0
)

# ── Suppress known-harmless EI SDK warnings ──────────────────────────────────
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-unused-variable
    -Wno-unused-function
    -Wno-sign-compare
    -Wno-missing-field-initializers
)

