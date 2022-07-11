
INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/src/common
    ${CMAKE_SOURCE_DIR}/src/kernel
    )

set(ACPI_INC "${CMAKE_CURRENT_SOURCE_DIR}/acpica/include")
set(KERNEL_LDS "${CMAKE_CURRENT_SOURCE_DIR}/kernel_link.lds")

SET(CMAKE_C_FLAGS         "${CMAKE_C_FLAGS} -fno-asynchronous-unwind-tables -mcmodel=large -fno-omit-frame-pointer -mno-mmx -mno-sse -mno-sse2 -isystem ${ACPI_INC}")

SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} \
  -nostdlib -static -z max-page-size=0x1000 -nodefaultlibs -T ${KERNEL_LDS} -v")

file(GLOB_RECURSE kernel_srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.s
    ${CMAKE_CURRENT_SOURCE_DIR}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/../common/*.c)

ADD_EXECUTABLE(kernel ${kernel_srcs})

set_target_properties(kernel PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/bin)
TARGET_LINK_LIBRARIES(kernel )

add_custom_command(
    TARGET kernel
    POST_BUILD
    COMMAND cp ARGS ${EXECUTABLE_OUTPUT_PATH}/kernel /run/media/simon/FAT/
)

