cmake_minimum_required(VERSION 3.22)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

set(CMAKE_PROJECT_NAME oscilloscope)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

enable_language(C ASM)

add_executable(${CMAKE_PROJECT_NAME})

add_subdirectory(cmake/stm32cubemx)

target_compile_options(oscilloscope PRIVATE
    -Os
    -flto
)

target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE)

# ── CMSIS-DSP ────────────────────────────────────────────────────────────────
# Define root FIRST, then use it
#──────────────────────────────────────────────────────────────────────────

set(CMSIS_DSP_ROOT ${CMAKE_SOURCE_DIR}/Middlewares/CMSIS-DSP)

set(CMSIS_DSP_SOURCES
    # RFFT (real FFT) - note: init is a SEPARATE file in your version
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_rfft_fast_f32.c
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_rfft_fast_init_f32.c

    # CFFT (complex FFT) - called internally by rfft
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_cfft_f32.c
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_cfft_init_f32.c
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_cfft_radix8_f32.c
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_cfft_radix2_f32.c
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_cfft_radix4_f32.c

    # Bit reversal tables
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_bitreversal2.c
    ${CMSIS_DSP_ROOT}/Source/TransformFunctions/arm_bitreversal.c

    # Common twiddle/constant tables
    ${CMSIS_DSP_ROOT}/Source/CommonTables/arm_const_structs.c
    ${CMSIS_DSP_ROOT}/Source/CommonTables/arm_common_tables.c

    # Complex magnitude - used by arm_cmplx_mag_f32
    ${CMSIS_DSP_ROOT}/Source/ComplexMathFunctions/arm_cmplx_mag_f32.c
)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Core/Src/ILI9341_GFX.c
    Core/Src/ILI9341_STM32_Driver.c
        Core/Src/time_domain.c
    Core/Src/fonts.c
       Core/Src/fft_display.c
    Core/Inc/fft_display.h
        Core/Inc/time_domain.h
    Core/Inc/ILI9341_GFX.h
    Core/Inc/ILI9341_STM32_Driver.h
    Core/Inc/fonts.h
    ${CMSIS_DSP_SOURCES}         # <-- actually add the sources
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMSIS_DSP_ROOT}/Include
        ${CMSIS_DSP_ROOT}/PrivateInclude   # <-- add this line

)

target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM4                 # required by arm_math.h
    __FPU_PRESENT=1              # enables FPU code paths
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    -lm                          # link libm for math builtins
)