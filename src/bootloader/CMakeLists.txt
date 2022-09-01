
INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}/src/common
    )

set(EFI_LIB "${EFI_DIR}/x86_64/gnuefi")
set(EFI_CRT_OBJS "${EFI_LIB}/crt0-efi-${ARCH}.o")
set(EFI_LDS "${EFI_LIB}/elf_${ARCH}_efi.lds")

SET(CMAKE_C_FLAGS         "${CMAKE_C_FLAGS} -fno-stack-protector -fshort-wchar -fPIC")

SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} \
  -nostdlib -nodefaultlibs -znocombreloc -T ${EFI_LDS} -shared -Wl,-Bsymbolic -L ${EFI_LIB} ${EFI_CRT_OBJS} -v")

file(GLOB_RECURSE bootloader_srcs ${CMAKE_CURRENT_SOURCE_DIR}/*.c )

ADD_EXECUTABLE(bootloader ${bootloader_srcs})

set_target_properties(bootloader PROPERTIES ARCHIVE_OUTPUT_DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/bin)
TARGET_LINK_LIBRARIES(bootloader efi gnuefi)

add_custom_command(
    TARGET bootloader
    POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} ARGS -j.text -j.sdata -j.data -j.dynamic -j.dynsym -j.rel -j.rela -j.reloc --target=efi-app-${ARCH} ${EXECUTABLE_OUTPUT_PATH}/bootloader ${EXECUTABLE_OUTPUT_PATH}/BOOTx64.EFI
COMMAND cp ARGS ${EXECUTABLE_OUTPUT_PATH}/BOOTx64.EFI /run/media/simon/FAT/EFI/BOOT/
)

