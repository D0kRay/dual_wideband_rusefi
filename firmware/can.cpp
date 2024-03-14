#include "can.h"
#include "hal.h"

#include "can_helper.h"
#include "can_aemnet.h"

#include "port.h"
#include "fault.h"
#include "can_helper.h"
#include "heater_control.h"
#include "lambda_conversion.h"
#include "sampling.h"
#include "pump_dac.h"
#include "max3185x.h"

// this same header is imported by rusEFI to get struct layouts and firmware version
#include "../for_rusefi/wideband_can.h"

static Configuration* configuration;

static THD_WORKING_AREA(waCanTxThread, 256);
void CanTxThread(void*)
{
    int cycle;
    chRegSetThreadName("CAN Tx");

    systime_t prev = chVTGetSystemTime(); // Current system time.

    while(1)
    {
        // AFR - 100 Hz
        for (int ch = 0; ch < AFR_CHANNELS; ch++) {
            SendCanForChannel(ch);
        }

        // EGT - 20 Hz
        if ((cycle % 5) == 0) {
            for (int ch = 0; ch < EGT_CHANNELS; ch++) {
                SendCanEgtForChannel(ch);
            }
        }

        cycle++;
        prev = chThdSleepUntilWindowed(prev, chTimeAddX(prev, TIME_MS2I(WBO_TX_PERIOD_MS)));
    }
}

static void SendAck()
{
    CANTxFrame frame;

    frame.IDE = CAN_IDE_EXT;
    frame.EID = WB_ACK;
    frame.RTR = CAN_RTR_DATA;
    frame.DLC = 0;

    canTransmitTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);
}

// Start in Unknown state. If no CAN message is ever received, we operate
// on internal battery sense etc.
static HeaterAllow heaterAllow = HeaterAllow::Unknown;
static float remoteBatteryVoltage = 0;

static THD_WORKING_AREA(waCanRxThread, 512);
void CanRxThread(void*)
{
    chRegSetThreadName("CAN Rx");

    while(1)
    {
        CANRxFrame frame;
        msg_t msg = canReceiveTimeout(&CAND1, CAN_ANY_MAILBOX, &frame, TIME_INFINITE);

        // Ignore non-ok results...
        if (msg != MSG_OK)
        {
            continue;
        }

        // Ignore std frames, only listen to ext
        if (frame.IDE != CAN_IDE_EXT)
        {
            continue;
        }

        if (frame.DLC == 2 && frame.EID == WB_MGS_ECU_STATUS) {
            // This is status from ECU - battery voltage and heater enable signal

            // data1 contains heater enable bit
            if ((frame.data8[1] & 0x1) == 0x1)
            {
                heaterAllow = HeaterAllow::Allowed;
            }
            else
            {
                heaterAllow = HeaterAllow::NotAllowed;
            }

            // data0 contains battery voltage in tenths of a volt
            float vbatt = frame.data8[0] * 0.1f;
            if (vbatt < 5)
            {
                // provided vbatt is bogus, default to 14v nominal
                remoteBatteryVoltage = 14;
            }
            else
            {
                remoteBatteryVoltage = vbatt;
            }
        }
        // If it's a bootloader entry request, reboot to the bootloader!
        else if ((frame.DLC == 0 || frame.DLC == 1) && frame.EID == WB_BL_ENTER)
        {
            // If 0xFF (force update all) or our ID, reset to bootloader, otherwise ignore
            if (frame.DLC == 0 || frame.data8[0] == 0xFF || frame.data8[0] == GetConfiguration()->afr[0].RusEfiIdOffset)
            {
                SendAck();

                // Let the message get out before we reset the chip
                chThdSleep(50);

                NVIC_SystemReset();
            }
        }
        // Check if it's an "index set" message
        else if (frame.DLC == 1 && frame.EID == WB_MSG_SET_INDEX)
        {
            int offset = frame.data8[0];
            configuration = GetConfiguration();
            for (int i = 0; i < AFR_CHANNELS; i++) {
                configuration->afr[i].RusEfiIdOffset = offset + i * 2;
            }
            for (int i = 0; i < EGT_CHANNELS; i++) {
                configuration->egt[i].RusEfiIdOffset = offset + i;
            }
            SetConfiguration();
            SendAck();
        }
    }
}

HeaterAllow GetHeaterAllowed()
{
    return heaterAllow;
}

float GetRemoteBatteryVoltage()
{
    return remoteBatteryVoltage;
}

void InitCan()
{
    configuration = GetConfiguration();

    canStart(&CAND1, &GetCanConfig());
    chThdCreateStatic(waCanTxThread, sizeof(waCanTxThread), NORMALPRIO, CanTxThread, nullptr);
    chThdCreateStatic(waCanRxThread, sizeof(waCanRxThread), NORMALPRIO - 4, CanRxThread, nullptr);
}

static int LambdaIsValid(int ch)
{
    const auto& sampler = GetSampler(ch);
    const auto& heater = GetHeaterController(ch);

    float nernstDc = sampler.GetNernstDc();

    return ((heater.IsRunningClosedLoop()) &&
            (nernstDc > (NERNST_TARGET - 0.1f)) &&
            (nernstDc < (NERNST_TARGET + 0.1f)));
}

void SendRusefiFormat(uint8_t ch)
{
    auto baseAddress = WB_DATA_BASE_ADDR + configuration->afr[ch].RusEfiIdOffset;

    const auto& sampler = GetSampler(ch);

    float nernstDc = sampler.GetNernstDc();

    if (configuration->afr[ch].RusEfiTx) {
        CanTxTyped<wbo::StandardData> frame(baseAddress + 0);

        // The same header is imported by the ECU and checked against this data in the frame
        frame.get().Version = RUSEFI_WIDEBAND_VERSION;

        uint16_t lambda = GetLambda(ch) * 10000;
        frame.get().Lambda = lambda;
        frame.get().TemperatureC = sampler.GetSensorTemperature();
        frame.get().Valid = LambdaIsValid(ch) ? 0x01 : 0x00;
    }

    if (configuration->afr[ch].RusEfiTxDiag) {
        auto esr = sampler.GetSensorInternalResistance();

        CanTxTyped<wbo::DiagData> frame(baseAddress + 1);

        frame.get().Esr = esr;
        frame.get().NernstDc = nernstDc * 1000;
        frame.get().PumpDuty = GetPumpOutputDuty(ch) * 255;
        frame.get().Status = GetCurrentFault(ch);
        frame.get().HeaterDuty = GetHeaterDuty(ch) * 255;
    }
}

void SendRusefiADCINFormat(uint8_t ch)
{
    Configuration* configuration = GetConfiguration();
    auto id = WB_DATA_BASE_ADDR + configuration->adc[ch].RusEfiIdOffset;
    const auto& sampler = GetSampler(ch+AFR_CHANNELS);

    if (configuration->adc[ch].RusEfiTx) {
        CanTxTyped<wbo::AdcData> frame(id, true);

        frame.get().ADC_raw_Value = sampler.GetRawADCValue();
    }
}

// Weak link so boards can override it
__attribute__((weak)) void SendCanForChannel(uint8_t ch)
{
    SendRusefiFormat(ch);
    SendAemNetUEGOFormat(ch);
    SendCanADCINForChannel(ch);
}

__attribute__((weak)) void SendCanEgtForChannel(uint8_t ch)
{
#if (EGT_CHANNELS > 0)
    // TODO: implement RusEFI protocol?
    SendAemNetEGTFormat(ch);
#endif /* EGT_CHANNELS > 0 */
}

__attribute__((weak)) void SendCanADCINForChannel(uint8_t ch)
{
#if (ANALOG_IN_CHANNELS > 0)
    // TODO: TEST?
    SendRusefiADCINFormat(ch);
#endif /* ANALOG_IN_CHANNELS > 0 */
}