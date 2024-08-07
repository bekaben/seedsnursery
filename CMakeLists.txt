cmake_minimum_required(VERSION 3.5)

set(PROJECT_NAME W7500-surf5)

include(${CMAKE_SOURCE_DIR}/tools/arm-none-eabi-gcc.cmake)
project(${PROJECT_NAME} ASM C CXX)

    # Compiler options
function(complie_link_option name)
    target_compile_options(${name} PRIVATE
    -mcpu=cortex-m0
    -mthumb
    -fdata-sections
    -ffunction-sections  
    -Wall
    -O2
    -g3
    )
    
    # Linker options
    target_link_options(${name} PRIVATE
    -T${CMAKE_SOURCE_DIR}/Libraries/CMSIS/Device/WIZnet/W7500/Source/GCC/gcc_w7500x.ld
    -mcpu=cortex-m0
    -mthumb
    -specs=nano.specs
    -lc
    -lm
    -Wl,-Map=${name}.map,--cref
    -Wl,--gc-sections
    -Xlinker -print-memory-usage -Xlinker
    )
endfunction()

function(create_binary_file PROJECT_NAME)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${PROJECT_NAME}> $<IF:$<BOOL:$<TARGET_PROPERTY:${PROJECT_NAME},OUTPUT_NAME>>,$<TARGET_PROPERTY:${PROJECT_NAME},OUTPUT_NAME>,$<TARGET_PROPERTY:${PROJECT_NAME},NAME>>.bin)
endfunction()

function(create_elf_file PROJECT_NAME)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${PROJECT_NAME}> $<TARGET_FILE_DIR:${PROJECT_NAME}>/${PROJECT_NAME}.elf
        COMMENT "Copying to ELF file: ${PROJECT_NAME}.elf"
    )
endfunction()

set(EXAMPLES_DIR ${CMAKE_SOURCE_DIR}/Projects/W7500x_StdPeriph_Examples)
add_subdirectory(${EXAMPLES_DIR})
add_subdirectory(Libraries)
add_subdirectory(src)
    

