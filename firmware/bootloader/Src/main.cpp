#include "main.h"

/*
Flash layout for STM32G473QE:

0x08000000 - 0x08007FFF  bank 1 pages 0-15   (32KB):  bootloader
0x08008000 - 0x0803FFFF  bank 1 pages 16-127 (224KB): active application
0x08040000 - 0x0807FFFF  bank 2              (256KB): OTA firmware update staging
*/

constexpr uint32_t application_addr = 0x08008000; // Start address of the application in flash memory
constexpr uint32_t new_image_addr   = 0x08040000; // Start address of the OTA staging area in flash memory

uint32_t* image_header = (uint32_t*)new_image_addr;

void unlock_flash() {
    // TODO: port to STM32G4 flash API (FLASH_KEYR / FLASH_KEYR2 for bank 2, dual-bank unlock)
}

void lock_flash() {
    // TODO: port to STM32G4 flash API
}

void erase_application_flash() {
    // TODO: port to STM32G4 page-based erase (FLASH_CR_PER + FLASH_CR_PNB)
    // Application occupies bank 1 pages 16-127 (0x08008000 - 0x0803FFFF)
}

void erase_staging_flash() {
    // TODO: port to STM32G4 page-based erase for bank 2
    // Staging area occupies all of bank 2 (0x08040000 - 0x0807FFFF)
}

void initialize_sram() {
    
}

void write_flash(uint32_t addr, void* src, uint32_t word_count) {
    // TODO: port to STM32G4 flash programming (double-word writes via FLASH_CR_PG)
    (void)addr; (void)src; (void)word_count;
}

void reset() {
    SCB->AIRCR = (1 << SCB_AIRCR_SYSRESETREQ_Pos) | (0x5FA << SCB_AIRCR_VECTKEY_Pos);
}

bool check_crc(void* src, uint32_t len) {
    // TODO: implement CRC validation
    (void)src; (void)len;
    return true;
}

void chainload(uint32_t offset) {
    SCB->VTOR = offset; // Set the vector table offset register to the new application address

    // Set the stack pointer and jump to the application reset handler
    asm volatile("ldr r1, [%0]; mov sp, r1; ldr %0, [%0, #4]; bx %0" :: "r" (offset) : "r1");

    while (true);   // should never reach here
}

int main(void) {
    // TODO: check OTA staging area for a valid new image, validate CRC,
    //       flash it into the application region, erase staging, then reset.
    //       For now, always chainload the application directly.
    chainload(application_addr);
}