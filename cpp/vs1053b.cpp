#include "vs1053b.hpp"
// Standard libraries
#include <cstring>
#include <stdio.h>
// Framework libraries
#include "ssp0.h"
#include "utilities.h"


#define MAX_DREQ_TIMEOUT_US (50000)
#define MAX_DREQ_TIMEOUT_MS (TICK_MS(50))
#define MAX_CS_TIMEOUT      (TICK_MS(50))

enum
{
    OPCODE_READ  = 0x03,
    OPCODE_WRITE = 0x02
};

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         SYSTEM FUNCTIONS                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

VS1053b::VS1053b(vs1053b_gpio_init_S init) :    
                                            DREQ(init.port_dreq   , init.pin_dreq),
                                            RESET(init.port_reset , init.pin_reset, true),
                                            XCS(init.port_xcs     , init.pin_xcs  , true),
                                            XDCS(init.port_xdcs   , init.pin_xdcs , true)
{
    // Local register default values
    RegisterMap[MODE]        = { .reg_num=MODE,        .can_write=true,  .reset_value=0x4000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[STATUS]      = { .reg_num=STATUS,      .can_write=true,  .reset_value=0x000C, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[BASS]        = { .reg_num=BASS,        .can_write=true,  .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[CLOCKF]      = { .reg_num=CLOCKF,      .can_write=true,  .reset_value=0x0000, .clock_cycles=1200, .reg_value=0 };
    RegisterMap[DECODE_TIME] = { .reg_num=DECODE_TIME, .can_write=true,  .reset_value=0x0000, .clock_cycles=100,  .reg_value=0 };
    RegisterMap[AUDATA]      = { .reg_num=AUDATA,      .can_write=true,  .reset_value=0x0000, .clock_cycles=450,  .reg_value=0 };
    RegisterMap[WRAM]        = { .reg_num=WRAM,        .can_write=true,  .reset_value=0x0000, .clock_cycles=100,  .reg_value=0 };
    RegisterMap[WRAMADDR]    = { .reg_num=WRAMADDR,    .can_write=true,  .reset_value=0x0000, .clock_cycles=100,  .reg_value=0 };
    RegisterMap[HDAT0]       = { .reg_num=HDAT0,       .can_write=false, .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[HDAT1]       = { .reg_num=HDAT1,       .can_write=false, .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[AIADDR]      = { .reg_num=AIADDR,      .can_write=true,  .reset_value=0x0000, .clock_cycles=210,  .reg_value=0 };
    RegisterMap[VOL]         = { .reg_num=VOL,         .can_write=true,  .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[AICTRL0]     = { .reg_num=AICTRL0,     .can_write=true,  .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[AICTRL1]     = { .reg_num=AICTRL1,     .can_write=true,  .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[AICTRL2]     = { .reg_num=AICTRL2,     .can_write=true,  .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };
    RegisterMap[AICTRL3]     = { .reg_num=AICTRL3,     .can_write=true,  .reset_value=0x0000, .clock_cycles=80,   .reg_value=0 };

    Status.fast_forward_mode  = false;
    Status.rewind_mode        = false;
    Status.low_power_mode     = false;
    Status.playing            = false;
    Status.waiting_for_cancel = false;

    CSMutex = xSemaphoreCreateMutex();
}

void VS1053b::SystemInit(SemaphoreHandle_t dreq_sem)
{
    DREQSem = dreq_sem;

    // Hardware reset
    if (!HardwareReset())     printf("[VS1053b::SystemInit] Failed to hardware reset timeout of 100000 us.\n");

    // Software reset
    if (!SoftwareReset())     printf("[VS1053b::SystemInit] Software reset failed...\n");
    else                      printf("[VS1053b::SystemInit] Device reset.\n");

    // Chip selects should start high
    if (!SetXCS(true))        printf("[VS1053b::SystemInit] Failed to set XCS to HIGH at init.\n");
    if (!SetXDCS(true))       printf("[VS1053b::SystemInit] Failed to set XDCS to HIGH at init.\n");

    // Check if booted in RTMIDI mode which causes issues with MP3 not playing     
    // Fix : http://www.bajdi.com/lcsoft-vs1053-mp3-module/#comment-33773     
    UpdateLocalRegister(AUDATA);       
    if (44100 == RegisterMap[AUDATA].reg_value || 44101 == RegisterMap[AUDATA].reg_value)      
    {      
        printf("[VS1053b::SystemInit] Defaulted to MIDI mode. Switching to MP3 mode.\n");      
        // Switch to MP3 mode if in RTMIDI mode        
        RegisterMap[WRAMADDR].reg_value = 0xC017;      
        RegisterMap[WRAM].reg_value     = 3;       
        UpdateRemoteRegister(WRAMADDR);        
        UpdateRemoteRegister(WRAM);        
        RegisterMap[WRAMADDR].reg_value = 0xC019;      
        RegisterMap[WRAM].reg_value     = 0;       
        UpdateRemoteRegister(WRAMADDR);        
        UpdateRemoteRegister(WRAM);        
        // Wait a little to make sure it was written       
        TICK_MS(100);   
        // Software reset to boot into MP3 mode        
        SoftwareReset();       
    }      

    UpdateLocalRegister(STATUS);
    printf("[VS1053b::SystemInit] Initial status: %04X\n", RegisterMap[STATUS].reg_value);
    printf("[VS1053b::SystemInit] Initial DREQ: %d\n", DeviceReady());
    printf("[VS1053b::SystemInit] Updating device registers with default settings...\n");

    const uint16_t mode_default_state   = 0x4800;
    const uint16_t clock_default_state  = 0x6000;
    const uint16_t volume_default_state = 0x3333;

    RegisterMap[MODE].reg_value   = mode_default_state;
    RegisterMap[CLOCKF].reg_value = clock_default_state;
    RegisterMap[VOL].reg_value    = volume_default_state;

    UpdateRemoteRegister(MODE);
    UpdateRemoteRegister(CLOCKF);
    UpdateRemoteRegister(VOL);

    SetFastForwardMode(false);

    // Update local register values
    if (!UpdateRegisterMap())
    {
        printf("[VS1053b::SystemInit] Failed to update register map.\n");
    }

    printf("[VS1053b::SystemInit] System initialization complete.\n");
}

vs1053b_transfer_status_E VS1053b::TransferData(uint8_t *data, uint32_t size)
{
    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::TransferData] Failed to transfer data timeout of 100000us.\n");
        return TRANSFER_FAILED;
    }

    if (size < 1)
    {
        return TRANSFER_FAILED;
    }
    else
    {
        uint32_t cycles    = size / 32;
        uint16_t remainder = size % 32;

        if (!SetXDCS(false))
        {
            LOG_ERROR("[VS1053b::TransferData] Failed to set XDCS low!\n");
            return TRANSFER_FAILED;
        }

        // uint64_t start = sys_get_uptime_us();
        for (uint32_t i=0; i<cycles; i++)
        {
            for (int byte=0; byte<32; byte++)
            {
                ssp0_exchange_byte(data[i * 32 + byte]);
            }
            
            // Wait until DREQ goes high
            if (!WaitForDREQ())
            {
                LOG_ERROR("[VS1053b::TransferData] Failed to transfer data timeout of 100000.\n");
                return TRANSFER_FAILED;
            }
        }
        // uint64_t end   = sys_get_uptime_us() - start;
        // printf("Elapsed: %lu \n", (uint32_t)end);


        if (remainder > 0)
        {
            for (int byte=0; byte<remainder; byte++)
            {
                ssp0_exchange_byte(data[cycles * 32 + byte]);
            }
            
            // Wait until DREQ goes high
            if (!WaitForDREQ())
            {
                LOG_ERROR("[VS1053b::TransferData] Failed to transfer data timeout of 100000.\n");
                return TRANSFER_FAILED;
            }
        }

        if (!SetXDCS(true))
        {
            LOG_ERROR("[VS1053b::TransferData] Failed to set XDCS low!\n");
            return TRANSFER_FAILED;
        }

        // Check for pending cancellation request
        if (Status.waiting_for_cancel)
        {
            // Check cancel bit
            UpdateLocalRegister(MODE);
            // Cancel succeeded, exit, and return status to bubble up to parent function
            if (RegisterMap[MODE].reg_value & (1 << 3))
            {
                return TRANSFER_CANCELLED;
            }
        }

        return TRANSFER_SUCCESS;
    }
}

bool VS1053b::HardwareReset()
{
    // Pull reset line low
    SetReset(false);

    // Wait 1 ms
    TICK_MS(1);

    // Pull reset line back high
    SetReset(true);

    return WaitForDREQ();
}

bool VS1053b::SoftwareReset()
{
    const uint16_t RESET_BIT = (1 << 2);

    UpdateLocalRegister(MODE);

    // Set reset bit
    RegisterMap[MODE].reg_value |= RESET_BIT;
    UpdateRemoteRegister(MODE);

    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::SoftwareReset] Failed to software reset timeout of 100000us.\n");
        return false;
    }

    // Reset bit is cleared automatically
    return true;
}

void VS1053b::PrintDebugInformation()
{
    if (!UpdateRegisterMap())
    {
        printf("[VS1053b::PrintDebugInformation] Failed to update register map.\n");
    }
    UpdateHeaderInformation();

    printf("------------------------------------------------------\n");
    printf("Sample Rate     : %d\n", RegisterMap[AUDATA].reg_value);
    printf("Decoded Time    : %d\n", RegisterMap[DECODE_TIME].reg_value);
    printf("------------------------------------------------------\n");
    printf("Header Information\n");
    printf("------------------------------------------------------\n");
    printf("stream_valid    : %d\n",  Header.stream_valid);
    printf("id              : %d\n",  Header.id);
    printf("layer           : %d\n",  Header.layer);
    printf("protect_bit     : %d\n",  Header.protect_bit);
    printf("bit_rate        : %lu\n", Header.bit_rate);
    printf("sample_rate     : %d\n",  Header.sample_rate);
    printf("pad_bit         : %d\n",  Header.pad_bit);
    printf("mode            : %d\n",  Header.mode);
    printf("------------------------------------------------------\n");
    printf("MODE            : %04X\n", RegisterMap[MODE].reg_value);
    printf("STATUS          : %04X\n", RegisterMap[STATUS].reg_value);
    printf("BASS            : %04X\n", RegisterMap[BASS].reg_value);
    printf("CLOCKF          : %04X\n", RegisterMap[CLOCKF].reg_value);
    printf("DECODE_TIME     : %04X\n", RegisterMap[DECODE_TIME].reg_value);
    printf("AUDATA          : %04X\n", RegisterMap[AUDATA].reg_value);
    printf("WRAM            : %04X\n", RegisterMap[WRAM].reg_value);
    printf("WRAMADDR        : %04X\n", RegisterMap[WRAMADDR].reg_value);
    printf("HDAT0           : %04X\n", RegisterMap[HDAT0].reg_value);
    printf("HDAT1           : %04X\n", RegisterMap[HDAT1].reg_value);
    printf("AIADDR          : %04X\n", RegisterMap[AIADDR].reg_value);
    printf("VOL             : %04X\n", RegisterMap[VOL].reg_value);
    printf("AICTRL0         : %04X\n", RegisterMap[AICTRL0].reg_value);
    printf("AICTRL1         : %04X\n", RegisterMap[AICTRL1].reg_value);
    printf("AICTRL2         : %04X\n", RegisterMap[AICTRL2].reg_value);
    printf("AICTRL3         : %04X\n", RegisterMap[AICTRL3].reg_value);
    printf("------------------------------------------------------\n");
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                           API FUNCTIONS                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////

void VS1053b::CancelDecoding()
{
    const uint8_t CANCEL_BIT = (1 << 3);

    if (IsPlaying())
    {
        UpdateLocalRegister(MODE);
        RegisterMap[MODE].reg_value |= CANCEL_BIT;
        UpdateRemoteRegister(MODE);
        // Set flag to request cancellation
        Status.waiting_for_cancel = true;
    }

    Status.playing = false;
}

void VS1053b::SetEarSpeakerMode(ear_speaker_mode_E mode)
{
    UpdateLocalRegister(MODE);

    const uint16_t low_bit  = (1 << 4);
    const uint16_t high_bit = (1 << 7);

    switch (mode)
    {
        case EAR_SPEAKER_OFF:
            RegisterMap[MODE].reg_value &= ~low_bit;
            RegisterMap[MODE].reg_value &= ~high_bit;
            break;
        case EAR_SPEAKER_MINIMAL:
            RegisterMap[MODE].reg_value |=  low_bit;
            RegisterMap[MODE].reg_value &= ~high_bit;
            break;
        case EAR_SPEAKER_NORMAL:
            RegisterMap[MODE].reg_value &= ~low_bit;
            RegisterMap[MODE].reg_value |=  high_bit;
            break;
        case EAR_SPEAKER_EXTREME:
            RegisterMap[MODE].reg_value |=  low_bit;
            RegisterMap[MODE].reg_value |=  high_bit;
            break;
    }

    UpdateRemoteRegister(MODE);
}

void VS1053b::SetStreamMode(bool on)
{
    UpdateLocalRegister(MODE);

    const uint16_t stream_bit = (1 << 6);

    if (on) RegisterMap[MODE].reg_value |=  stream_bit;
    else    RegisterMap[MODE].reg_value &= ~stream_bit;

    UpdateRemoteRegister(MODE);
}

void VS1053b::SetClockDivider(bool on)
{
    UpdateLocalRegister(MODE);

    const uint16_t clock_range_bit = (1 << 15);

    if (on) RegisterMap[MODE].reg_value |=  clock_range_bit;
    else    RegisterMap[MODE].reg_value &= ~clock_range_bit;

    UpdateRemoteRegister(MODE);
}

void VS1053b::SetBaseEnhancement(uint8_t amplitude, uint8_t freq_limit)
{
    UpdateLocalRegister(STATUS);

    // Clamp to max
    if (amplitude  > 0xF) amplitude  = 0xF;
    if (freq_limit > 0xF) freq_limit = 0xF;
    
    const uint8_t bass_value = (amplitude << 4) | freq_limit;
    
    RegisterMap[STATUS].reg_value |= (bass_value << 0);

    UpdateRemoteRegister(STATUS);
}

void VS1053b::SetTrebleControl(uint8_t amplitude, uint8_t freq_limit)
{
    UpdateLocalRegister(STATUS);

    // Clamp to max
    if (amplitude  > 0xF) amplitude  = 0xF;
    if (freq_limit > 0xF) freq_limit = 0xF;
    
    const uint8_t treble_value = (amplitude << 4) | freq_limit;
    
    RegisterMap[STATUS].reg_value |= (treble_value << 8);

    UpdateRemoteRegister(STATUS);
}

void VS1053b::SetVolume(float percentage)
{
    // Limit to maximum 100%
    percentage = MIN(1, percentage);

    uint8_t volume = 0xFF * percentage;

    RegisterMap[VOL].reg_value = (volume << 8) | volume;

    UpdateRemoteRegister(VOL);
}

void VS1053b::IncrementVolume(float percentage)
{
    // Limit to maximum 100%
    percentage = MIN(1, percentage);

    uint8_t increment_step = 0xFF * percentage;

    uint8_t left_vol  = RegisterMap[VOL].reg_value >> 8;
    uint8_t right_vol = RegisterMap[VOL].reg_value & 0x00FF;
    left_vol  = (left_vol  + increment_step > 0xFF) ? (0xFF) : (left_vol  + increment_step);
    right_vol = (right_vol + increment_step > 0xFF) ? (0xFF) : (right_vol + increment_step);
    RegisterMap[VOL].reg_value = (left_vol << 8) | (right_vol & 0xFF);

    UpdateRemoteRegister(VOL);
}

void VS1053b::DecrementVolume(float percentage)
{
    // Limit to maximum 100%
    percentage = MIN(1, percentage);

    uint8_t decrement_step = 0xFF * percentage;

    uint8_t left_vol  = RegisterMap[VOL].reg_value >> 8;
    uint8_t right_vol = RegisterMap[VOL].reg_value & 0x00FF;
    left_vol  = (left_vol  - decrement_step < 0) ? (0) : (left_vol  - decrement_step);
    right_vol = (right_vol - decrement_step < 0) ? (0) : (right_vol - decrement_step);
    RegisterMap[VOL].reg_value = (left_vol << 8) | (right_vol & 0xFF);

    UpdateRemoteRegister(VOL);
}

void VS1053b::SetLowPowerMode(bool on)
{
    if (on)
    {
        // If not already in low power mode
        if (!Status.low_power_mode)
        {
            // Set clock speed to 1.0x, disabling PLL
            RegisterMap[CLOCKF].reg_value = 0x0000;
            UpdateRemoteRegister(CLOCKF);

            // Reduce sample rate
            RegisterMap[AUDATA].reg_value = 0x0010;
            UpdateRemoteRegister(AUDATA);

            // Turn off ear speaker mode
            SetEarSpeakerMode(EAR_SPEAKER_OFF);

            // Turn off analog drivers
            SetVolume(1.0f);
        }
    }
    else
    {
        // If in low power mode
        if (Status.low_power_mode)
        {
            // Turn off analog drivers
            SetVolume(1.0f);

            // Turn off ear speaker mode
            SetEarSpeakerMode(EAR_SPEAKER_OFF);

            // Reduce sample rate
            RegisterMap[AUDATA].reg_value = 0x0010;
            UpdateRemoteRegister(AUDATA);

            // Set clock speed to 1.0x, disabling PLL
            RegisterMap[CLOCKF].reg_value = 0x0000;
            UpdateRemoteRegister(CLOCKF);
        }
    }

    Status.low_power_mode = on;
}

vs1053b_transfer_status_E VS1053b::PlaySegment(uint8_t *mp3, uint32_t size, bool last_segment)
{
    static uint32_t segment_counter = 0;
    const  uint8_t dummy_short[] = { 0x00, 0x00 };

    // If first segment, set up for playback
    if (!Status.playing)
    {
        printf("[VS1053b::PlaySegment] Playback starting...\n");

        // Reset counter
        segment_counter = 0;

        // Clear decode time
        ClearDecodeTime();

        // Send 2 dummy bytes to SDI
        TransferData((uint8_t*)dummy_short, 2);

        Status.playing = true;
    }

    // Send mp3 file
    vs1053b_transfer_status_E status = TransferData(mp3, size);
    switch (status)
    {
        case TRANSFER_SUCCESS:
            break;
        case TRANSFER_FAILED:
            printf("[VS1053b::PlaySegment] Transfer failed on %lu segment!\n", segment_counter);
            break;
        case TRANSFER_CANCELLED:
            printf("[VS1053b::PlaySegment] Transfer cancelled on %lu segment!\n", segment_counter);
            break;
    }

    // Clean up if last segment
    if (last_segment || TRANSFER_CANCELLED == status)
    {
        // To signal the end of the mp3 file need to set 2052 bytes of EndFillByte
        SendEndFillByte(2052);

        // Wait 50 ms buffer time between playbacks
        TICK_MS(50);

        // Update status flags
        Status.playing = false;

        if (TRANSFER_CANCELLED == status)
        {
            Status.waiting_for_cancel = false;
        }

        segment_counter = 0;
    }
    else
    {
        ++segment_counter;
    }

    return status;
}

void VS1053b::SetFastForwardMode(bool on)
{
    const uint16_t play_speed_register_address = 0x1E04;

    if (on)
    {
        const uint16_t double_speed = 0x0002;
        WriteRam(play_speed_register_address, double_speed);
        Status.fast_forward_mode = true;
    }
    else
    {
        const uint16_t normal_speed = 0x0001;
        WriteRam(play_speed_register_address, normal_speed);
        Status.fast_forward_mode = false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         GETTER FUNCTIONS                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

bool VS1053b::GetFastForwardMode()
{
    return Status.fast_forward_mode;
}

uint16_t VS1053b::GetSampleRate()
{
    UpdateLocalRegister(AUDATA);

    // If bit 0 is a 1, then the sample rate is -1, else sample rate is the same
    return (RegisterMap[AUDATA].reg_value & 1) ? (RegisterMap[AUDATA].reg_value - 1) : (RegisterMap[AUDATA].reg_value);
}

vs1053b_status_S* VS1053b::GetStatus()
{
    return &Status;
}

uint16_t VS1053b::GetCurrentDecodedTime()
{
    UpdateLocalRegister(DECODE_TIME);

    return RegisterMap[DECODE_TIME].reg_value;
}

void VS1053b::UpdateHeaderInformation()
{
    UpdateLocalRegister(HDAT0);
    UpdateLocalRegister(HDAT1);

    // Clear header
    memset(&Header, 0, sizeof(Header));

    // Copy registers to bit fields
    Header.reg1.value = RegisterMap[HDAT1].reg_value;
    Header.reg0.value = RegisterMap[HDAT0].reg_value;

    // HDAT1
    Header.stream_valid = (Header.reg1.bits.sync_word) == 2047;
    Header.id           = Header.reg1.bits.id;
    Header.layer        = Header.reg1.bits.layer;
    Header.protect_bit  = Header.reg1.bits.protect_bit;

    // HDAT0
    Header.pad_bit      = Header.reg0.bits.pad_bit;
    Header.mode         = Header.reg0.bits.mode;

    // Lookup sample rate
    switch (Header.reg0.bits.sample_rate)
    {
        case 3:
            // No sample rate
            break;
        case 2:
            switch (Header.layer)
            {
                case 3:  Header.sample_rate = 32000; break;
                case 2:  Header.sample_rate = 16000; break;
                default: Header.sample_rate =  8000; break;
            }
        case 1:
            switch (Header.layer)
            {
                case 3:  Header.sample_rate = 48000; break;
                case 2:  Header.sample_rate = 24000; break;
                default: Header.sample_rate = 12000; break;
            }
        case 0:
            switch (Header.layer)
            {
                case 3:  Header.sample_rate = 44100; break;
                case 2:  Header.sample_rate = 22050; break;
                default: Header.sample_rate = 11025; break;
            }
    }

    // Calculate bit rate
    uint16_t increment_value = 0;
    uint16_t start_value     = 0;
    switch (Header.layer)
    {
        case 1: 
            switch (Header.id)
            {
                case 3:  increment_value = 32; start_value = 32; break;
                default: increment_value = 8;  start_value = 32; break;
            }
            break;
        case 2: 
            switch (Header.id)
            {
                case 3:  increment_value = 16; start_value = 32; break;
                default: increment_value = 8;  start_value = 8;  break;
            }
            break;
        case 3: 
            switch (Header.id)
            {
                case 3:  increment_value = 8; start_value = 32; break;
                default: increment_value = 8; start_value = 8;  break;
            }
            break;
    }

    const uint16_t bits_in_kilobit = (1 << 10);
    if (Header.reg0.bits.bit_rate != 0 && Header.reg0.bits.bit_rate != 0xF)
    {
        Header.bit_rate = (start_value + (increment_value * (Header.reg0.bits.bit_rate-1))) * bits_in_kilobit;
    }
}

vs1053b_mp3_header_S* VS1053b::GetHeaderInformation()
{
    UpdateHeaderInformation();

    return &Header;
}

uint32_t VS1053b::GetBitRate()
{
    UpdateHeaderInformation();

    return Header.bit_rate;
}

bool VS1053b::IsPlaying()
{
    return Status.playing;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                         INLINE FUNCTIONS                                       //
////////////////////////////////////////////////////////////////////////////////////////////////////

inline bool VS1053b::SetXDCS(bool value)
{
    if (!xSemaphoreTake(CSMutex, MAX_CS_TIMEOUT))
    {
        LOG_ERROR("[VS1053b::SetXDCS] Failed to take CSMutex.\n");
        return false;
    }
    else
    {
        XDCS.SetValue(value);
        xSemaphoreGive(CSMutex);
        return true;
    }
}

inline bool VS1053b::GetXDCS()
{
    return XDCS.GetValue();
}

inline bool VS1053b::SetXCS(bool value)
{
    if (!xSemaphoreTake(CSMutex, MAX_CS_TIMEOUT))
    {
        LOG_ERROR("[VS1053b::SetXCS] Failed to take CSMutex.\n");
        return false;
    }
    else
    {
        XCS.SetValue(value);
        xSemaphoreGive(CSMutex);
        return true;
    }
}

inline bool VS1053b::GetXCS()
{
    return XCS.GetValue();
}

inline bool VS1053b::DeviceReady()
{
    // Return true for ready when DREQ is HIGH
    return DREQ.IsHigh();
}

inline void VS1053b::SetReset(bool value)
{
    RESET.SetValue(value);
}

#if 0
inline bool VS1053b::IsValidAddress(uint16_t address)
{
    bool valid = true;

    if (address < 0x1800)                          valid = false;
    else if (address > 0x18FF && address < 0x5800) valid = false;
    else if (address > 0x58FF && address < 0x8040) valid = false;
    else if (address > 0x84FF && address < 0xC000) valid = false;

    if (!valid) 
    {
        printf("[VS1053b::IsValidAddress] Address is invalid!!\n");
    }

    return valid;
}
#endif

inline bool VS1053b::UpdateLocalRegister(SCI_reg reg)
{
    uint16_t data = 0;

    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::UpdateLocalRegister] Failed to update register: %d, DREQ timeout of 100000.\n", reg);
        return false;
    }

    // Select XCS
    if (!SetXCS(false))
    {
        printf("[VS1053b::UpdateLocalRegister] Failed to select XCS as XDCS is already active!\n");
        return false;
    }
    else
    {
        ssp0_exchange_byte(OPCODE_READ);
        ssp0_exchange_byte(reg);
        data |= ssp0_exchange_byte(0x00) << 8;
        data |= ssp0_exchange_byte(0x00);
        RegisterMap[reg].reg_value = data;
        SetXCS(true);
        return true;
    }

    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::UpdateLocalRegister] Failed to update register: %d, DREQ timeout of 100000.\n", reg);
        return false;
    }
}

inline bool VS1053b::UpdateRemoteRegister(SCI_reg reg)
{
    // Transfer local register value to remote
    return (RegisterMap[reg].can_write) ? (TransferSCICommand(reg)) : (false);
}

inline void VS1053b::ChangeSCIRegister(SCI_reg reg, uint8_t bit, bool bit_value)
{
    if (bit_value)
    {
        RegisterMap[reg].reg_value |= (1 << bit);
    }
    else
    {
        RegisterMap[reg].reg_value &= ~(1 << bit);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//                                       PRIVATE FUNCTIONS                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////

bool VS1053b::WaitForDREQ()
{
    bool success = true;
    if (!DeviceReady())
    {
        // If the sem is given, toss it away and try to take again
        xSemaphoreTake(DREQSem, NO_DELAY);            
        success = xSemaphoreTake(DREQSem, MAX_DREQ_TIMEOUT_MS);
    }
    return success;

#if 0
    // Blocking version
    int timeout_us = MAX_DREQ_TIMEOUT_US;

    // Wait until DREQ goes high
    while (!DeviceReady())
    {
        delay_us(100);
        timeout_us  -= 100;
        if (timeout_us <= 0)
        {
            success = false;
            break;
        }
    }
#endif
}

uint16_t VS1053b::ReadRam(uint16_t address)
{
    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::ReadRam] Failed to read RAM[%d], timeout of 100000.\n", address);
        return 0;
    }

    XCS.SetLow();

    // Write address into WRAMADDR
    RegisterMap[WRAMADDR].reg_value = address;
    UpdateRemoteRegister(WRAMADDR);

    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::ReadRam] Failed to read RAM[%d], timeout of 100000.\n", address);
        return 0;
    }

    XCS.SetHigh();
    XCS.SetLow();

    // Read WRAM
    UpdateLocalRegister(WRAM);

    XCS.SetHigh();

    return RegisterMap[WRAM].reg_value;
}

bool VS1053b::WriteRam(uint16_t address, uint16_t value)
{
    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::ReadRam] Failed to read RAM[%d], timeout of 100000.\n", address);
        return false;
    }

    XCS.SetLow();

    // Write address into WRAMADDR
    RegisterMap[WRAMADDR].reg_value = address;
    UpdateRemoteRegister(WRAMADDR);

    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::ReadRam] Failed to read RAM[%d], timeout of 100000.\n", address);
        return false;
    }

    XCS.SetHigh();
    XCS.SetLow();

    // Write data in WRAM
    RegisterMap[WRAM].reg_value = value;
    UpdateRemoteRegister(WRAM);

    XCS.SetHigh();

    return true;
}

uint8_t VS1053b::GetEndFillByte()
{
    const uint16_t end_fill_byte_address = 0x1E06;
    const uint16_t half_word = ReadRam(end_fill_byte_address);

    // Ignore upper byte
    return half_word & 0xFF;
}


#if 0
// Unecessary
float VS1053b::ClockCyclesToMicroSeconds(uint16_t clock_cycles, bool is_clockf)
{
    UpdateLocalRegister(CLOCKF);

    uint8_t multiplier  = RegisterMap[CLOCKF].reg_value >> 13;          // [15:13]
    uint8_t adder       = (RegisterMap[CLOCKF].reg_value >> 12) & 0x3;  // [12:11]
    uint16_t frequency  = RegisterMap[CLOCKF].reg_value & 0x07FF;       // [10:0]

    uint32_t XTALI = (frequency * 4000 + 8000000);
    uint32_t CLKI  = XTALI * (multiplier + adder);

    float microseconds_per_cycle = 1000.0f * 1000.0f;
    if (is_clockf)
    {
        microseconds_per_cycle /= XTALI;
    }
    else
    {
        microseconds_per_cycle /= CLKI;
    }

    return ceil(microseconds_per_cycle * clock_cycles) + 1;
}
#endif

bool VS1053b::TransferSCICommand(SCI_reg reg)
{
    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::TransferSCICommand] Failed to update register: %d, DREQ timeout of 100000us.\n", reg);
        return false;
    }

    // Select XCS
    if (!SetXCS(false))
    {
        printf("[VS1053b::TransferSCICommand] Failed to select XCS as XDCS is already active!\n");
        return false;
    }
    // High byte first
    ssp0_exchange_byte(OPCODE_WRITE);
    ssp0_exchange_byte(reg);
    ssp0_exchange_byte(RegisterMap[reg].reg_value >> 8);
    ssp0_exchange_byte(RegisterMap[reg].reg_value & 0xFF);
    // Deselect XCS
    SetXCS(true);

    // Wait until DREQ goes high
    if (!WaitForDREQ())
    {
        printf("[VS1053b::TransferSCICommand] Failed to update register: %d, DREQ timeout of 100000us.\n", reg);
        return false;
    }

    return true;
}

void VS1053b::SendEndFillByte(uint16_t size)
{
    const uint8_t end_fill_byte = GetEndFillByte();
    // Uses an array of 32 bytes instead of the 2048+ bytes to conserve stack space
    uint8_t efb_array[32] = { 0 };
    memset(efb_array, end_fill_byte, sizeof(uint8_t) * 32);

    for (int i=0; i<65; i++)
    {
        TransferData(efb_array, size);
    }
}

bool VS1053b::UpdateRegisterMap()
{
    for (int reg=MODE; reg<SCI_reg_last_invalid; reg++)
    {
        if (reg == (SCI_reg)AIADDR)
        {
            continue;
        }
        else if (!UpdateLocalRegister((SCI_reg)reg))
        {
            printf("[VS1053b::UpdateRegisterMap] Failed at reg %d\n", reg);
            return false;
        }
    }

    return true;
}

void VS1053b::ClearDecodeTime()
{
    RegisterMap[DECODE_TIME].reg_value = 0;
    // Must be cleared twice because once is not enough apparently...
    UpdateRemoteRegister(DECODE_TIME);
    UpdateRemoteRegister(DECODE_TIME);
}