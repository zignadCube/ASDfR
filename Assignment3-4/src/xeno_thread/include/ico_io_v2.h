#pragma once

#include <unistd.h>
#include <stdint.h>
#include <cstdio>

#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/sysinfo.h>
#include <sys/mman.h>
#include <error.h>
#include <errno.h>
#include <string.h>
#include <time.h>

#include <evl/evl.h>
#include <evl/timer.h>
#include <evl/clock.h>
#include <evl/proxy.h>

#include <fcntl.h>

#include <linux/spi/spidev.h>
#include <uapi/evl/devices/spidev.h>

#include <config.h>

#define FRAME_SIZE 12
#define MILLION 1000000L
#define BILLION 1000000000L

#if !ENABLE_OUTPUT_LIMIT
#define MAX_PWM_VALUE 2047
#endif

#if ENABLE_OUTPUT_LIMIT
#define MAX_PWM_VALUE (int)(2047 * MAX_PWM_OUTPUT)
#endif

class IcoIO
{
public:
    IcoIO();
    ~IcoIO();

    struct IcoWrite
    {
        int16_t pwm1;		// PWM value for channel 1
        bool val1;			// Digital output 1 of channel 1
        int16_t pwm2;		// PWM value for channel 2
        bool val2;			// Digital output 1 of channel 2
        int16_t pwm3;		// PWM value for channel 3
        bool val3;			// Digital output 1 of channel 3
        int16_t pwm4;		// PWM value for channel 4
        bool val4;			// Digital output 1 of channel 4
    };

   struct IcoRead
    {
        int channel1;		// Encoder value from channel 1
        int channel2;		// Encoder value from channel 1
        int channel3;		// Encoder value from channel 1
        int channel4;		// Encoder value from channel 1
        bool channel1_1;	// Digital input 1 from channel 1
        bool channel1_2;	// Digital input 2 from channel 1
        bool channel2_1;	// Digital input 1 from channel 1
        bool channel2_2;	// Digital input 2 from channel 1
        bool channel3_1;	// Digital input 1 from channel 1
        bool channel3_2;	// Digital input 2 from channel 1
        bool channel4_1;	// Digital input 1 from channel 1
        bool channel4_2;	// Digital input 2 from channel 1
    };

    struct ReadValue
    {
        unsigned int enc : 14;
        unsigned int pin1 : 1;
        unsigned int pin2 : 1;
    };

    // Functions
    void setOutput(uint8_t channel, int16_t value, bool pin_val);
    ReadValue getInput(uint8_t channel);
    size_t update_io(IcoWrite write_val, IcoRead *read_val);
    int spi_init();
    int init();
    int reset();
    int test();

private:
    // SPI related config
    struct spi_int
    {
        struct spi_ioc_oob_setup oob_setup;
        int fd;
        int len;
        char *tx;
        char *rx;
        void *iobuf;
    };
    // SPI commands codes
    enum OP_CODE
    {
        NOOP = 0,
        INIT = 1,
        RESET = 2,
        ALL = 3,
        PWM_1 = 4,
        PWM_2 = 5,
        PWM_3 = 6,
        PWM_4 = 7,
        ENC_1 = 8,
        ENC_2 = 9,
        ENC_3 = 10,
        ENC_4 = 11,
        TEST = 12

    };

    struct WriteValue
    {
        unsigned int pwm_val : 14;
        unsigned int pwm_dir : 1;
        unsigned int pin_val : 1;
    };

    struct WriteArray
    {
        unsigned int pwm_val1 : 14;
        unsigned int pwm_dir1 : 1;
        unsigned int pin_val1 : 1;
        unsigned int pwm_val2 : 14;
        unsigned int pwm_dir2 : 1;
        unsigned int pin_val2 : 1;
        unsigned int pwm_val3 : 14;
        unsigned int pwm_dir3 : 1;
        unsigned int pin_val3 : 1;
        unsigned int pwm_val4 : 14;
        unsigned int pwm_dir4 : 1;
        unsigned int pin_val4 : 1;
    };

    union WriteBytes
    {
        WriteValue n;
        uint8_t s[2];
    };

    union WriteArrayBytes
    {
        WriteArray in;
        uint8_t out[6];
    };

    union ReadBytes
    {
        ReadValue v;
        uint8_t s[2];
    };

    // Variables
    WriteValue write_value_;
    WriteBytes write_bytes_;
    ReadValue read_value_;
    ReadBytes read_bytes_;

    int max_pwm_value;
    // return value
    int ret = 0;

    const uint8_t rd_msg[4];
    int device_handle_;

    // functions
    int create_pwm_msg(uint8_t channel, int16_t value, bool pin_val);
    int create_all_msg(IcoWrite data);
    WriteValue calc_value(int pwm_val, bool pin_val);
    WriteBytes create_pwm_bytes(int16_t pwm_val, bool pin_val);

    //
    struct spi_int device;
    void spi_transfer();

    // To be removed, temp solution
    void reverseInputBytes();
    void reverseOutputBytes();
    const uint8_t reverseTable[256] = {0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0,
                                       0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8,
                                       0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4,
                                       0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC,
                                       0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2,
                                       0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
                                       0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6,
                                       0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
                                       0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
                                       0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9,
                                       0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
                                       0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
                                       0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3,
                                       0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
                                       0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7,
                                       0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF};
};
