# CMake script to generate BHI385 firmware header from binary
# Called as: cmake -P gen_fw.cmake <input.fw> <output.h>

if(NOT DEFINED CMAKE_ARGV3 OR NOT DEFINED CMAKE_ARGV4)
    message(FATAL_ERROR "Usage: cmake -P gen_fw.cmake <input.fw> <output.h>")
endif()

set(INPUT_FILE "${CMAKE_ARGV3}")
set(OUTPUT_FILE "${CMAKE_ARGV4}")

# Read binary file
file(READ "${INPUT_FILE}" FIRMWARE_DATA HEX)

# Get file size
file(SIZE "${INPUT_FILE}" FILE_SIZE)

# Convert hex string to array format
string(LENGTH "${FIRMWARE_DATA}" HEX_LENGTH)
math(EXPR BYTE_COUNT "${HEX_LENGTH} / 2")

# Build the C array
set(C_ARRAY "")
set(BYTES_PER_LINE 16)
set(BYTE_INDEX 0)

while(BYTE_INDEX LESS BYTE_COUNT)
    set(LINE "    ")
    set(LINE_BYTES 0)

    while(LINE_BYTES LESS BYTES_PER_LINE AND BYTE_INDEX LESS BYTE_COUNT)
        math(EXPR HEX_INDEX "${BYTE_INDEX} * 2")
        string(SUBSTRING "${FIRMWARE_DATA}" "${HEX_INDEX}" 2 HEX_BYTE)

        if(LINE_BYTES GREATER 0)
            string(APPEND LINE ", ")
        endif()
        string(APPEND LINE "0x${HEX_BYTE}")

        math(INCR BYTE_INDEX)
        math(INCR LINE_BYTES)
    endwhile()

    string(APPEND LINE ",\n")
    string(APPEND C_ARRAY "${LINE}")
endwhile()

# Generate header file
file(WRITE "${OUTPUT_FILE}"
"/* Generated firmware array from ${INPUT_FILE} */
#include <stdint.h>

const uint8_t bhi385_firmware_image[] = {
${C_ARRAY}};

const uint32_t bhi385_firmware_image_size = ${FILE_SIZE};
")

message(STATUS "Generated ${OUTPUT_FILE} (${FILE_SIZE} bytes)")
