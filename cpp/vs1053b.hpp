#pragma once
// Project libraries
#include "common.hpp"
#include "gpio_input.hpp"
#include "gpio_output.hpp"


typedef enum
{
    TRANSFER_SUCCESS,
    TRANSFER_FAILED,
    TRANSFER_CANCELLED
} vs1053b_transfer_status_E;

typedef enum 
{
    EAR_SPEAKER_OFF,
    EAR_SPEAKER_MINIMAL,
    EAR_SPEAKER_NORMAL,
    EAR_SPEAKER_EXTREME
} ear_speaker_mode_E;

typedef struct
{
    uint8_t  reg_num;
    bool     can_write;
    uint16_t reset_value;
    uint16_t clock_cycles;
    uint16_t reg_value;
} __attribute__((packed)) SCI_reg_t;

typedef enum 
{
    MODE        = 0x0,
    STATUS      = 0x1,
    BASS        = 0x2,
    CLOCKF      = 0x3,
    DECODE_TIME = 0x4,
    AUDATA      = 0x5,
    WRAM        = 0x6,
    WRAMADDR    = 0x7,
    HDAT0       = 0x8,
    HDAT1       = 0x9,
    AIADDR      = 0xA,
    VOL         = 0xB,
    AICTRL0     = 0xC,
    AICTRL1     = 0xD,
    AICTRL2     = 0xE,
    AICTRL3     = 0xF,
    SCI_reg_last_invalid
} SCI_reg;

typedef struct
{
    // HDAT1
    bool     stream_valid;
    uint8_t  id;
    uint8_t  layer;
    bool     protect_bit;
    // HDAT0
    uint32_t bit_rate;
    uint16_t sample_rate;
    bool     pad_bit;
    uint8_t  mode;

    union
    {
        struct
        {
            uint8_t protect_bit : 1;
            uint8_t layer       : 2;
            uint8_t id          : 2;
            uint16_t sync_word  : 11;
        } bits;
        uint16_t value;
    } reg1;

    union
    {
        struct
        {
            uint8_t emphasis    : 2;
            uint8_t original    : 1;
            uint8_t copyright   : 1;
            uint8_t extension   : 2;
            uint8_t mode        : 2;
            uint8_t private_bit : 1;
            uint8_t pad_bit     : 1;
            uint8_t sample_rate : 2;
            uint8_t bit_rate    : 4;
        } bits;
        uint16_t value;
    } reg0;

} __attribute__((packed)) vs1053b_mp3_header_S;

typedef struct
{
    gpio_port_t port_reset;
    gpio_port_t port_dreq;
    gpio_port_t port_xcs;
    gpio_port_t port_xdcs;
    uint8_t     pin_reset;
    uint8_t     pin_dreq;
    uint8_t     pin_xcs;
    uint8_t     pin_xdcs;
} __attribute__((packed)) vs1053b_gpio_init_S;

typedef struct 
{
    bool fast_forward_mode;
    bool rewind_mode;
    bool low_power_mode;
    bool playing;
    bool waiting_for_cancel;
} __attribute__((packed)) vs1053b_status_S;

class VS1053b
{
public:

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                         SYSTEM FUNCTIONS                                       //
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // @description     : Constructor, initializes the device
    // @param init      : Port and pin number for all GPIOS necessary for device to function
    VS1053b(vs1053b_gpio_init_S init);

    // @description     : Initializes the VS1053B system, all the pins, and the default states of the registers
    void SystemInit(SemaphoreHandle_t dreq_sem);

    // @description     : Perform a hardware reset
    // @returns         : True for success, false for unsuccessful (timeout)
    bool HardwareReset();

    // @description     : Perform a software reset
    // @returns         : True for success, false for unsuccessful (timeout)
    bool SoftwareReset();

    void PrintDebugInformation();

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                           API FUNCTIONS                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // @description     : Sets flag to request cancellation
    void CancelDecoding();

    // @description     : Sets the ear speaking procecssing mode
    // @param mode      : Either off, minimal, normal, or extreme
    void SetEarSpeakerMode(ear_speaker_mode_E mode);

    // @description     : Sets the mode of streaming
    // @param on        : True for on, false for off
    void SetStreamMode(bool on);

    // @description     : Set the clock divider register to divide by 2
    // @param on        : True for on, false for off
    void SetClockDivider(bool on);

    // @description     : Sets the base register value
    // @param amplitude : The value of the register, possible values from 0x0 to 0xF
    // @param freq_limit: The lower frequency
    void SetBaseEnhancement(uint8_t amplitude, uint8_t freq_limit);

    // @description     : Sets the treble register value
    // @param amplitude : The value of the register, possible values from 0x0 to 0xF
    // @param freq_limit: The lower frequency
    void SetTrebleControl(uint8_t amplitude, uint8_t freq_limit);

#if 0
    // @description       : Sets the sample rate register
    // @param sample_rate : The sample rate to set to, possible values range from 0 to 48k
    void SetSampleRate(uint16_t sample_rate);
#endif

    // @description      : Sets the volume register, left and right volumes can be different
    // @param percentage : Percentage to set the volume to
    void SetVolume(float percentage);
    void IncrementVolume(float percentage=0.05f);
    void DecrementVolume(float percentage=0.05f);

    // @description     : Turns on or off the lower power mode
    // @param on        : True for on, false for off
    void SetLowPowerMode(bool on);

    // @description        : Plays a segment of a song
    // @param mp3          : Array of mp3 file bytes
    // @param size         : Size of the arrray of file
    // @param last_segment : True for end of file, runs clean up routine, false for not end of file
    // @returns            : Status of transfer
    vs1053b_transfer_status_E PlaySegment(uint8_t *mp3, uint32_t size, bool last_segment);

    // @description     : Turns on or off the fast forward mode
    // @param on        : True for on, false for off
    void SetFastForwardMode(bool on);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                         GETTER FUNCTIONS                                       //
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // @description     : Status of fast forward mode
    // @returns         : True for fast forwarding, false for normal speed
    bool GetFastForwardMode();

    // @description     : Get the current sample rate
    // @returns         : The current sample rate
    uint16_t GetSampleRate();

    // @description     : Returns the current status struct
    // @returns         : Current status struct
    vs1053b_status_S* GetStatus();

    // @description     : Read the current decoding time register
    // @returns         : The current decoding time in seconds
    uint16_t GetCurrentDecodedTime();

#if 0
    // @description     : Reads the current playback position of the mp3 file
    // @returns         : Playback position in milliseconds
    uint32_t GetPlaybackPosition();
#endif

    // @description     : Parses the header information of the current mp3 file
    // @returns         : Struct of the header information
    vs1053b_mp3_header_S* GetHeaderInformation();

    // @description     : Reads the current bit rate setting
    // @returns         : The current bit rate
    uint32_t GetBitRate();

    // @description     : Status of decoder, playing means it is expecting data
    // @returns         : True for playing, false for not playing
    bool IsPlaying();

private:

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                         MEMBER VARIABLES                                       //
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // Pins for VS1053B interfacing
    // Must be initialized before using driver
    GpioInput  DREQ;
    GpioOutput RESET;
    GpioOutput XCS;
    GpioOutput XDCS;

    // Only one CS can be active at a time, mutex to ensure that
    SemaphoreHandle_t CSMutex;

    // Pointer to a semaphore that wait on DREQ to go HIGH
    SemaphoreHandle_t DREQSem;

    // Stores a struct of the current mp3's header information
    vs1053b_mp3_header_S Header;

    // Stores a struct of status information to be transmitted
    vs1053b_status_S Status;

    // Stores a map of structs of each register's values and information
    SCI_reg_t RegisterMap[SCI_reg_last_invalid];

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                         INLINE FUNCTIONS                                       //
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // @description     : Sets the XDCS pin
    // @param value     : Value to set the pin to
    // @returns         : True for set, false for did not set
    inline bool SetXDCS(bool value);

    // @description     : Reads the XDCS pin
    // @returns         : Value of pin
    inline bool GetXDCS();

    // @description     : Sets the XCS pin
    // @returns         : True for set, false for did not set
    inline bool SetXCS(bool value);

    // @description     : Reads the XCS pin
    // @param value     : Value to set the pin to
    // @returns         : Value of pin
    inline bool GetXCS();

    // @description     : Reads the DREQ pin
    // @returns         : Value of pin
    inline bool DeviceReady();

    // @description     : Sets the RESET pin
    // @param value     : Value to set the pin to
    inline void SetReset(bool value);

#if 0
    // @description     : Checks if the supplied address is a valid address to access
    // @param address   : The address to check
    // @returns         : True for valid, false for invalid
    inline bool IsValidAddress(uint16_t address);
#endif

    // @description     : Reads the register on the device and updates the RegisterMap
    // @param reg       : Enum of the register
    inline bool UpdateLocalRegister(SCI_reg reg);

    // @description     : Writes to the register from the value in the RegisterMap
    // @param reg       : Enum of the register
    // @returns         : True for successful, false for unsuccessful
    inline bool UpdateRemoteRegister(SCI_reg reg);

    // @description     : Change a single bit of one of the SCI registers
    // @param reg       : Specifies which SCI register
    // @param bit       : Specifies which bit of the SCI register
    // @param bit_value : Specifies value of bit to set, true for 1, false for 0
    inline void ChangeSCIRegister(SCI_reg reg, uint8_t bit, bool bit_value);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    //                                        PRIVATE FUNCTIONS                                       //
    ////////////////////////////////////////////////////////////////////////////////////////////////////

    // @description     : Sends data to the device
    // @param data      : The data byte to write
    // @param size      : Size of array to transfer
    // @returns         : Status after transfer
    vs1053b_transfer_status_E TransferData(uint8_t *data, uint32_t size);

    // @description     : Waits for DREQ semaphore to be given, times out based on MAX_DREQ_TIMEOUT_MS
    // @returns         : True for successful, false for unsuccessful    
    bool WaitForDREQ();

    // @description     : Read a register from RAM that is not a command register
    // @param address   : Address of register to read the data from
    // @returns         : Value of register
    uint16_t ReadRam(uint16_t address);

    // @description     : Writes a value to RAM that is not a command register
    // @param address   : Address of register to write the data to
    // @param value     : The value to write to the register
    // @returns         : True for successful, false for unsuccessful
    bool WriteRam(uint16_t address, uint16_t value);

    // @description     : Sends local SCI register value to remote
    // @param reg       : The specified register
    // @returns         : True for successful, false for unsuccessful
    bool TransferSCICommand(SCI_reg reg);

    // @description     : Reads the endFillByte parameter from the device
    // @returns         : The endFillByte
    uint8_t GetEndFillByte();

    // @description     : Sends the end fill byte for x amount of transfers
    // @param size      : The amount of end fill bytes to send
    // @returns         : True for successful, false for unsuccessful
    void SendEndFillByte(uint16_t size);

    // @description     : Updates the header struct with fresh information
    void UpdateHeaderInformation();

#if 0
    // @description        : Computes the microseconds needed to delay for a specified amount of clock cycles
    // @param clock_cycles : The number of cycles to apply to the calculation
    // @param is_clockf    : True to show the register it is calculating for is CLOCKF, which requires a different calculation
    //                       from the other registers since it is based off of XTALI instead of CLKI
    float ClockCyclesToMicroSeconds(uint16_t clock_cycles, bool is_clockf);

    // @description     : Reads the current status information from the device and updates the struct
    void UpdateStatusMap();
#endif

    // @description     : Reads the value of each register and updates the register map
    // @returns         : True for successful, false for unsuccessful
    bool UpdateRegisterMap();

    // @description     : Decode time register does not clear automatically so must be explicitly cleared
    void ClearDecodeTime();
};