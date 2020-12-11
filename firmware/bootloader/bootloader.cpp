#include "ch.h"
#include "hal.h"

#include "flash.h"

#include <cstring>

// These are defined in the linker script
extern uint32_t __appflash_start__;
extern uint32_t __appflash_size__;
extern uint32_t __ram_vectors_start__;
extern uint32_t __ram_vectors_size__;

__attribute__((noreturn))
void boot_app() {
    // Goodbye, ChibiOS
    chSysDisable();

    // Reset peripherals we might have used
    rccDisableCAN1();

    const uint32_t* appFlash = &__appflash_start__;

    // copy vector table to sram
    // TODO: use __ram_vectors_size__
    memcpy(reinterpret_cast<char*>(&__ram_vectors_start__), appFlash, 256);

    // The reset vector is at offset 4 (second uint32)
    uint32_t reset_vector = appFlash[1];

    // switch to use vectors in ram
    SYSCFG->CFGR1 |= 3;

    // TODO: is this necessary?
    //uint32_t app_msp = appLocation[0];
    //__set_MSP(app_msp);

    typedef void (*ResetVectorFunction)(void);
    ((ResetVectorFunction)reset_vector)();

    while(1);
}

uint32_t appFlashAddr = (uint32_t)&__appflash_start__;

void EraseAppPages()
{
    uintptr_t blSize = (uintptr_t)(appFlashAddr - 0x08000000);
    size_t pageIdx = blSize / 1024;

    size_t appSizeKb = __appflash_size__ / 1024;

    for (size_t i = 0; i <= appSizeKb; i++)
    {
        Flash::ErasePage(pageIdx);
        pageIdx++;
    }
}


void WaitForBootloaderCmd()
{
    while(true)
    {
        CANRxFrame frame;
        msg_t result = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);

        // Ignore non-ok results
        if (result != MSG_OK) 
        {
            continue;
        }

        // if we got a bootloader-init message, here we go!
        if (frame.EID == 0xEF0'0000)
        {
            return;
        }
    }
}

void sendAck()
{
    CANTxFrame frame;

    frame.IDE = CAN_IDE_EXT;
    frame.EID = 0x727573;   // ascii "rus"
    frame.RTR = CAN_RTR_DATA;
    frame.DLC = 0;

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);
}

void sendNak()
{
    // TODO: implement
}

bool bootloaderBusy = false;

void RunBootloaderLoop()
{
    // First ack that the bootloader is alive
    sendAck();

    while (true)
    {
        CANRxFrame frame;
        msg_t result = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);

        // Ignore non-ok results
        if (result != MSG_OK) 
        {
            continue;
        }

        // 29-bit extended ID:
        //  0 xxxy zzzz
        // xx = header, always equals 0xEF
        //  y = opcode
        // zzzz = extra 2 data bytes hidden in the address!

        uint16_t header = frame.EID >> 20;

        // All rusEfi bootloader packets start with 0x0EF, ignore other traffic on the bus
        if (header != 0x0EF)
        {
            continue;
        }

        uint8_t opcode = (frame.EID >> 16) & 0xFF;
        uint16_t embeddedData = frame.EID & 0xFFFF;

        switch (opcode) {
            case 0x00: // opcode 0 is simply the "enter BL" command, but we're already here.  Send an ack.
                sendAck();
                break;
            case 0x01: // opcode 1 is "erase app flash"
                // embedded data must be 0x5A5A
                if (embeddedData == 0x5A5A)
                {
                    EraseAppPages();
                    sendAck();
                }
                else
                {
                    sendNak();
                }

                break;
            case 0x02: // opcode 2 is "write flash data"
                // Embedded data is the flash address

                // Don't allow misaligned writes
                if (embeddedData % sizeof(flashdata_t) != 0 || frame.DLC % sizeof(flashdata_t) != 0)
                {
                    sendNak();
                }
                // Don't allow out of bounds writes
                else if (embeddedData < 0 || embeddedData > 26 * 1024)
                {
                    sendNak();
                }
                else
                {
                    Flash::Write(appFlashAddr + embeddedData, &frame.data8[0], frame.DLC);
                    sendAck();
                }

                break;
            case 0x03: // opcode 3 is "boot app"
                // Clear the flag
                bootloaderBusy = false;
                // Kill this thread
                return;
            default:
                sendNak();
                break;
        }
    }
}

THD_WORKING_AREA(waBootloaderThread, 512);
THD_FUNCTION(BootloaderThread, arg)
{
    (void)arg;

    WaitForBootloaderCmd();

    // We've rx'd a BL command, don't load the app!
    bootloaderBusy = true;

    RunBootloaderLoop();
}

/*
 * Application entry point.
 */
int main(void) {
    halInit();
    chSysInit();

    chThdCreateStatic(waBootloaderThread, sizeof(waBootloaderThread), NORMALPRIO + 1, BootloaderThread, nullptr);

    // PB5 is blue LED
    palSetPadMode(GPIOB, 5, PAL_MODE_OUTPUT_PUSHPULL);
    // PB6 is green LED
    palSetPadMode(GPIOB, 6, PAL_MODE_OUTPUT_PUSHPULL);
    palTogglePad(GPIOB, 6);

    for (size_t i = 0; i < 20; i++)
    {
        palTogglePad(GPIOB, 5);
        palTogglePad(GPIOB, 6);
        chThdSleepMilliseconds(40);
    }

    // Block until booting the app is allowed
    while (bootloaderBusy)
    {
        palTogglePad(GPIOB, 5);
        palTogglePad(GPIOB, 6);
        chThdSleepMilliseconds(200);
    }

    boot_app();
}
