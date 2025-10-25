set(SOURCES "audio/audio_codec.cc"
            "audio/audio_service.cc"
            "audio/codecs/no_audio_codec.cc"
            "audio/codecs/box_audio_codec.cc"
            "audio/codecs/es8311_audio_codec.cc"
            "audio/codecs/es8374_audio_codec.cc"
            "audio/codecs/es8388_audio_codec.cc"
            "audio/codecs/es8389_audio_codec.cc"
            "audio/codecs/dummy_audio_codec.cc"
            "audio/processors/audio_debugger.cc"
            "audio/codecs/santa_audio_codec.cc"
            "led/single_led.cc"
            "led/circular_strip.cc"
            "led/gpio_led.cc"
            "display/display.cc"
            "display/lcd_display.cc"
            "display/oled_display.cc"
            "protocols/protocol.cc"
            "protocols/mqtt_protocol.cc"
            "protocols/websocket_protocol.cc"
            "protocols/ble_protocol.cc"
            "mcp_server.cc"
            "system_info.cc"
            "application.cc"
            "ota.cc"
            "settings.cc"
            "device_state_event.cc"
            "main.cc"
            )

set(INCLUDE_DIRS "." "display" "audio" "protocols")

# 添加板级公共文件
file(GLOB BOARD_COMMON_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/boards/common/*.cc)
list(APPEND SOURCES ${BOARD_COMMON_SOURCES})
list(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/boards/common)

# 根据 BOARD_TYPE 配置添加对应的板级文件
if(CONFIG_BOARD_TYPE_ESP_HI)
    set(BOARD_TYPE "Micropupper")
elseif(CONFIG_BOARD_TYPE_HEYSANTA)
    set(BOARD_TYPE "HeySanta")
endif()
file(GLOB BOARD_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_TYPE}/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_TYPE}/*.c
)
list(APPEND SOURCES ${BOARD_SOURCES})

if(CONFIG_USE_AUDIO_PROCESSOR)
    list(APPEND SOURCES "audio/processors/afe_audio_processor.cc")
else()
    list(APPEND SOURCES "audio/processors/no_audio_processor.cc")
endif()
if(CONFIG_USE_AFE_WAKE_WORD)
    list(APPEND SOURCES "audio/wake_words/afe_wake_word.cc")
elseif(CONFIG_USE_ESP_WAKE_WORD)
    list(APPEND SOURCES "audio/wake_words/esp_wake_word.cc")
elseif(CONFIG_USE_CUSTOM_WAKE_WORD)
    list(APPEND SOURCES "audio/wake_words/custom_wake_word.cc")
endif()

# 根据Kconfig选择语言目录
if(CONFIG_LANGUAGE_ZH_CN)
    set(LANG_DIR "zh-CN")
elseif(CONFIG_LANGUAGE_ZH_TW)
    set(LANG_DIR "zh-TW")
elseif(CONFIG_LANGUAGE_EN_US)
    set(LANG_DIR "en-US")
elseif(CONFIG_LANGUAGE_JA_JP)
    set(LANG_DIR "ja-JP")
endif()

# 定义生成路径
set(LANG_JSON "${CMAKE_CURRENT_SOURCE_DIR}/assets/${LANG_DIR}/language.json")
set(LANG_HEADER "${CMAKE_CURRENT_SOURCE_DIR}/assets/lang_config.h")
file(GLOB LANG_SOUNDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/${LANG_DIR}/*.p3)
file(GLOB COMMON_SOUNDS ${CMAKE_CURRENT_SOURCE_DIR}/assets/common/*.p3)

# 如果目标芯片是 ESP32，则排除特定文件
if(CONFIG_IDF_TARGET_ESP32)
    list(REMOVE_ITEM SOURCES "audio/codecs/box_audio_codec.cc"
                             "audio/codecs/es8388_audio_codec.cc"
                             "audio/codecs/es8389_audio_codec.cc"
                             "led/gpio_led.cc"
                             )
endif()

idf_component_register(SRCS ${SOURCES}
                    EMBED_FILES ${LANG_SOUNDS} ${COMMON_SOUNDS}
                    INCLUDE_DIRS ${INCLUDE_DIRS}
                    WHOLE_ARCHIVE
                    )

# 使用 target_compile_definitions 来定义 BOARD_TYPE, BOARD_NAME
# 如果 BOARD_NAME 为空，则使用 BOARD_TYPE
if(NOT BOARD_NAME)
    set(BOARD_NAME ${BOARD_TYPE})
endif()
target_compile_definitions(${COMPONENT_LIB}
                    PRIVATE BOARD_TYPE=\"${BOARD_TYPE}\" BOARD_NAME=\"${BOARD_NAME}\"
                    )

# 添加生成规则
add_custom_command(
    OUTPUT ${LANG_HEADER}
    COMMAND python ${PROJECT_DIR}/scripts/gen_lang.py
            --input "${LANG_JSON}"
            --output "${LANG_HEADER}"
    DEPENDS
        ${LANG_JSON}
        ${PROJECT_DIR}/scripts/gen_lang.py
    COMMENT "Generating ${LANG_DIR} language config"
)

# 强制建立生成依赖
add_custom_target(lang_header ALL
    DEPENDS ${LANG_HEADER}
)

if(CONFIG_BOARD_TYPE_ESP_HI)
set(URL "https://github.com/espressif2022/image_player/raw/main/test_apps/test_8bit")
set(SPIFFS_DIR "${CMAKE_BINARY_DIR}/emoji")
file(MAKE_DIRECTORY ${SPIFFS_DIR})

# List all files to download
set(FILES_TO_DOWNLOAD "")
list(APPEND FILES_TO_DOWNLOAD "Anger_enter.aaf" "Anger_loop.aaf" "Anger_return.aaf")
list(APPEND FILES_TO_DOWNLOAD "happy_enter.aaf" "happy_loop.aaf" "happ_return.aaf")
list(APPEND FILES_TO_DOWNLOAD "sad_enter.aaf" "sad_loop.aaf" "sad_return.aaf")
list(APPEND FILES_TO_DOWNLOAD "scorn_enter.aaf" "scorn_loop.aaf" "scorn_return.aaf")
list(APPEND FILES_TO_DOWNLOAD "left_enter.aaf" "left_loop.aaf" "left_return.aaf")
list(APPEND FILES_TO_DOWNLOAD "right_enter.aaf" "right_loop.aaf" "right_return.aaf")
list(APPEND FILES_TO_DOWNLOAD "asking.aaf" "blink_once.aaf" "blink_quick.aaf")
list(APPEND FILES_TO_DOWNLOAD "connecting.aaf" "panic_enter.aaf" "panic_loop.aaf")
list(APPEND FILES_TO_DOWNLOAD "panic_return.aaf" "wake.aaf")

foreach(FILENAME IN LISTS FILES_TO_DOWNLOAD)
    set(REMOTE_FILE "${URL}/${FILENAME}")
    set(LOCAL_FILE "${SPIFFS_DIR}/${FILENAME}")
    
    # 检查本地文件是否存在
    if(EXISTS ${LOCAL_FILE})
        message(STATUS "File ${FILENAME} already exists, skipping download")
    else()
        message(STATUS "Downloading ${FILENAME}")
        file(DOWNLOAD ${REMOTE_FILE} ${LOCAL_FILE}
             STATUS DOWNLOAD_STATUS)
        list(GET DOWNLOAD_STATUS 0 STATUS_CODE)
        if(NOT STATUS_CODE EQUAL 0)
            message(FATAL_ERROR "Failed to download ${FILENAME} from ${URL}")
        endif()
    endif()
endforeach()
endif()

if(CONFIG_BOARD_TYPE_HEYSANTA)
set(URL "https://github.com/espressif2022/image_player/raw/main/test_apps/test_8bit")
set(SPIFFS_DIR "${CMAKE_BINARY_DIR}/emoji")
file(MAKE_DIRECTORY ${SPIFFS_DIR})
set(FILES_TO_DOWNLOAD "")
list(APPEND FILES_TO_DOWNLOAD "heart.aaf" "bell.aaf" "star.aaf" "Blinking.aaf" "snowman.aaf"
    "cookie.aaf" "Whole_elf.aaf" "sleep.aaf" "cross.aaf" "happy.aaf" "cross2.aaf" "happy2.aaf")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/heart.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/bell.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/star.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/Blinking.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/snowman.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/cookie.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/Whole_elf.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/sleep.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/cross.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/happy.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/cross2.aaf" DESTINATION "${SPIFFS_DIR}")
file(COPY "${CMAKE_SOURCE_DIR}/emoji/happy2.aaf" DESTINATION "${SPIFFS_DIR}")

spiffs_create_partition_assets(
    assets_A
    ${SPIFFS_DIR}
    FLASH_IN_PROJECT
    MMAP_FILE_SUPPORT_FORMAT ".aaf"
)
endif()

if(CONFIG_BOARD_TYPE_ECHOEAR)

idf_build_get_property(build_components BUILD_COMPONENTS)
foreach(COMPONENT ${build_components})
    if(COMPONENT MATCHES "esp_emote_gfx" OR COMPONENT MATCHES "espressif2022__esp_emote_gfx")
        set(EMOTE_GFX_COMPONENT ${COMPONENT})
        idf_component_get_property(EMOTE_GFX_COMPONENT_PATH ${EMOTE_GFX_COMPONENT} COMPONENT_DIR)
        set(SPIFFS_DIR "${EMOTE_GFX_COMPONENT_PATH}/emoji_normal")
        break()
    endif()
endforeach()

spiffs_create_partition_assets(
    assets_A
    ${SPIFFS_DIR}
    FLASH_IN_PROJECT
    MMAP_FILE_SUPPORT_FORMAT ".aaf, ttf, bin"
    IMPORT_INC_PATH ${CMAKE_CURRENT_SOURCE_DIR}/boards/${BOARD_TYPE}
)
endif()