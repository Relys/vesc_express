cmake_minimum_required(VERSION 3.5)

# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
set(PROJECT_VER "6.0") 
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Default build target if not provided
if(NOT DEFINED HW_NAME)
    set(HW_NAME "VESC Express T")
endif()

file(GLOB_RECURSE HW_HEADER_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    main/hwconf/hw_*.h
)

foreach(HW_HEADER_PATH ${HW_HEADER_PATHS})
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${HW_HEADER_PATH}" FILE_CONTENTS)
    set(IS_HW_MATCH false)

    foreach(LINE IN LISTS FILE_CONTENTS)
        string(REGEX MATCH "^#define[ \t]+([A-Za-z_][A-Za-z0-9_]*)[ \t]+(.*)$" MATCHES "${LINE}")
        if(MATCHES)
            set(DEF_KEY ${CMAKE_MATCH_1})
            set(DEF_VAL ${CMAKE_MATCH_2})

            # Remove surrounding quotes
            if(DEF_VAL MATCHES "^\".*\"$")
                string(REGEX REPLACE "^\"(.*)\"$" "\\1" DEF_VAL ${DEF_VAL})
            endif()

            if (DEF_KEY STREQUAL "HW_NAME" AND DEF_VAL STREQUAL HW_NAME)
                set(IS_HW_MATCH true)
            endif()

            # Extract HW_TARGET once HW_NAME matches
            if (IS_HW_MATCH AND DEF_KEY STREQUAL "HW_TARGET")
                set(HW_TARGET ${DEF_VAL})
                message("Found HW_TARGET: ${HW_TARGET}")
                set(HW_FILE_NAME ${HW_HEADER_PATH})
            endif()

            # Extract HW_PSRAM if present
            if (IS_HW_MATCH AND DEF_KEY STREQUAL "HW_PSRAM")
                set(HW_PSRAM ${DEF_VAL})
                message("Found HW_PSRAM: ${HW_PSRAM}")
            endif()

            # Exit early if we found HW_NAME, HW_TARGET, and HW_PSRAM
            if (IS_HW_MATCH AND DEFINED HW_TARGET)
                break()
            endif()
        endif()
    endforeach()

    # Break the outer loop if we found a matching file
    if(IS_HW_MATCH)
        break()
    endif()
endforeach()


if(DEFINED HW_NAME AND DEFINED HW_TARGET AND DEFINED HW_FILE_NAME)
    # Get file name without extension
    string(REGEX MATCH "([^/]+)$" HW_FILE ${HW_FILE_NAME})
    string(REGEX REPLACE ".h" "" HW_FILE ${HW_FILE})

    # Define function to set the hardware target
    function(set_hw_target)
        file(REMOVE sdkconfig)
        configure_file(sdkconfig.${HW_TARGET} ${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig COPYONLY)
        execute_process(COMMAND "idf.py reconfigure" WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    endfunction()

    # Define HW_PSRAM if not already set (optional behavior)
    if(NOT DEFINED HW_PSRAM)
        set(HW_PSRAM "")  # Leave it empty if not provided
    endif()

    # Use HW_PSRAM in sdkconfig logic, but only if it's defined
    if(NOT DEFINED SDKCONFIG_NAME)
        if(HW_PSRAM)
            set(SDKCONFIG_FINAL "sdkconfig.${HW_TARGET}.psram.${HW_PSRAM}")
            message("Using custom sdkconfig: sdkconfig.${HW_TARGET}.psram.${HW_PSRAM}")
        else()
            set(SDKCONFIG_FINAL "sdkconfig.${HW_TARGET}")
            message("Using default sdkconfig: sdkconfig.${HW_TARGET}")
        endif()
    else()
        set(SDKCONFIG_FINAL ${SDKCONFIG_NAME})
        message("Using provided sdkconfig: ${SDKCONFIG_NAME}")
    endif()

    # Define the cache location
    set(TARGET_NAME_CACHE_LOCATION ${CMAKE_BINARY_DIR}/last_target)

    # Use a combined cache key that includes HW_PSRAM only if it's defined
    if(HW_PSRAM)
        set(CACHE_KEY "${HW_TARGET}_${HW_PSRAM}")
    else()
        set(CACHE_KEY "${HW_TARGET}")
    endif()

    # Set our sdkconfig based on hw target and psram settings
    if(EXISTS ${TARGET_NAME_CACHE_LOCATION})
        file(READ ${TARGET_NAME_CACHE_LOCATION} LAST_CACHE_KEY)
        if(NOT CACHE_KEY STREQUAL LAST_CACHE_KEY)
            message("Build target or PSRAM configuration has changed from ${LAST_CACHE_KEY} to ${CACHE_KEY}")
            file(REMOVE sdkconfig)
            set_hw_target()
        else()
            message("Build target and PSRAM configuration matches the last build: ${CACHE_KEY}")
        endif()
    else()
        message("Build target cache missing. Setting up new build with ${CACHE_KEY}")
        set_hw_target()
    endif()

    # Save the new cache key
    file(WRITE ${TARGET_NAME_CACHE_LOCATION} ${CACHE_KEY})
    SET(IDF_TARGET ${HW_TARGET})

    # Pass through hw file paths
    add_compile_definitions(HW_HEADER="${HW_FILE}.h" HW_SOURCE="${HW_FILE}.c")
else ()
    message(FATAL_ERROR "Build target info missing.\nHW_NAME: ${HW_NAME}\nHW_TARGET: ${HW_TARGET}\nHW_FILE: ${HW_FILE_NAME}") 
endif()

# Conditional message based on whether HW_PSRAM is set or not
if(HW_PSRAM)
    message("Processing target ${HW_NAME} on ${HW_TARGET} with PSRAM: ${HW_PSRAM} for file ${HW_FILE_NAME}")
else()
    message("Processing target ${HW_NAME} on ${HW_TARGET} for file ${HW_FILE_NAME}")
endif()


# ESP-IDF remainder boilerplate
get_filename_component(ProjectId ${CMAKE_CURRENT_LIST_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})
idf_build_set_property(COMPILE_OPTIONS "-fdiagnostics-color=always" APPEND)
