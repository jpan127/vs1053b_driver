# vs1053b_driver
Driver for interfacing with a VS1053b decoder in C++ and C

## Dependencies

    This driver was made for an LPC17XX microcontroller using FreeRTOS.
    
    1. FreeRTOS
    2. cstring            : memset
    3. stdio (optional)   : printf
    4. SPI
    5. Non-FreeRTOS delay microsecond (optional)

#### API

    Most of the functions are documented in the header file.  
    Some functions are not implemented or were deemed unecessary so they are left commented out.
    Getter functions : Get information about the decoder state or the song details
    API functions    : Set values or settings, plays or stops a song
    System functions : Initialize device, reset device