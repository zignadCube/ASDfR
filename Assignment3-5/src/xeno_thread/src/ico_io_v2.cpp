#include "ico_io_v2.h"

IcoIO::IcoIO() : device_handle_{-1}, rd_msg{0, 0, 0, 0}
{
    printf("%s: Constructing rampio\n", __FUNCTION__);
}

IcoIO::~IcoIO()
{
    printf("%s: Destructing rampio\n", __FUNCTION__);
}

int IcoIO::spi_init()
{
    // SPI configuration
    uint8_t spi_count = 4;
    static uint32_t mode = SPI_MODE_0;
    static uint8_t bits = 8;
    static uint32_t speed = 8 * MILLION;
    static int len = 12;
    uint8_t lsb_first = 0;

    // Open the SPI device
    char device_name[20];

#if DEBUG_SPI
    evl_printf("SPI: Start with opening device");
#endif

    sprintf(device_name, "/dev/spidev%d.0", 0);
    device.fd = open(device_name, O_RDWR);
    if (device.fd < 0)
        error(1, errno, "can't open device %p", &device);

        /*
        ret = ioctl(device.fd, SPI_IOC_WR_LSB_FIRST, &lsb_first);
        if (ret)
            error(1, errno, "ioctl(SPI_IOC_WR_LSB_FIRST)");
        */

#if DEBUG_SPI
    evl_printf("SPI: setting up mode\n");
#endif

    // Setting SPI mode
    ret = ioctl(device.fd, SPI_IOC_WR_MODE, &mode);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_WR_MODE32)");

#if DEBUG_SPI
    evl_printf("SPI: Setting up bit per word: %d\n", bits);
#endif

    ret = ioctl(device.fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_WR_BITS_PER_WORD)");

#if DEBUG_SPI
    evl_printf("SPI: Setting up max speed: %d\n", speed);
#endif

    ret = ioctl(device.fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_WR_MAX_SPEED_HZ)");

#if DEBUG_SPI
    evl_printf("SPI: Switching to OOB operation\n");
#endif

    /* Switch to Out-of-Band operation */
    device.oob_setup.frame_len = len;
    device.oob_setup.speed_hz = speed;
    device.oob_setup.bits_per_word = bits;

    ret = ioctl(device.fd,
                SPI_IOC_ENABLE_OOB_MODE, &device.oob_setup);
    if (ret)
        error(1, errno, "ioctl(SPI_IOC_ENABLE_OOB_MODE)");

    /* Map the I/O area */
    device.iobuf = mmap(NULL,
                        device.oob_setup.iobuf_len,
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED, device.fd, 0);
    if (device.iobuf == MAP_FAILED)
        error(1, errno, "mmap()");

    /* Define TX/RX from I/O Buffer */
    // evl_printf("memset tx %d\n", index);
    device.tx = (char *)(device.iobuf + device.oob_setup.tx_offset);
    memset(device.tx, 0, len);

    // evl_printf("memset rx %d\n", index);
    device.rx = (char *)(device.iobuf + device.oob_setup.rx_offset);
    memset(device.rx, 0, len);

    /* Extra information */
    device.len = len;

    return 1;
}

// This function is a temperally solution to the problem of not beiing able to set thhe LSB register
void IcoIO::reverseInputBytes()
{
    for (int x = 0; x < FRAME_SIZE; x++)
    {
        device.tx[x] = reverseTable[device.tx[x]];
    }
    return;
}

void IcoIO::reverseOutputBytes()
{
    for (int x = 0; x < FRAME_SIZE; x++)
    {
        device.rx[x] = reverseTable[device.rx[x]];
    }
    return;
}

void IcoIO::spi_transfer()
{
    memset(device.rx, 0, FRAME_SIZE);

#if DEBUG_SPI
    evl_printf("write: ");
    for (int i = 0; i < spi_frame_size; i++)
        evl_printf("0x%02x ", device.tx[i]);
    evl_printf("\n");
#endif

    // to be removed, temp solution
    reverseInputBytes();

    int ret = oob_ioctl(device.fd, SPI_IOC_RUN_OOB_XFER);
    if (ret)
        error(1, errno, "oob_ioctl(SPI_IOC_RUN_OOB_XFER)");

    // to be removed, temp solution
    reverseOutputBytes();

#if DEBUG_SPI
    evl_printf("read: ");
    for (int i = 0; i < spi_frame_size; i++)
        evl_printf("0x%02x ", device.rx[i]);
    evl_printf("\n");
#endif
}

int IcoIO::init()
{
    useconds_t uSleep = 100;
    uint8_t msg[FRAME_SIZE] = {
        OP_CODE::INIT, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00};
    memcpy(device.tx, msg, FRAME_SIZE);

    spi_transfer();
    if (device.rx[4] == 11)
        return FRAME_SIZE;
    else
        error(1, 0, "Failed to init the SPI module on FPGA");
        
    return 1;
}

int IcoIO::test()
{
    uint8_t msg[FRAME_SIZE] = {
        OP_CODE::TEST, 0x00, 0x01, 0x02,
        0x03, 0x04, 0x05, 0x06,
        0x07, 0x08, 0x09, 0x10};

    memcpy(device.tx, msg, FRAME_SIZE);
    // transieve data
    spi_transfer();
    for (int i = 0; i < FRAME_SIZE; i++)
    {
        evl_printf("%d", device.rx[i]);
    }

    return 1;
}

int IcoIO::reset()
{
    uint8_t msg[FRAME_SIZE] = {
        OP_CODE::RESET, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00};

    memcpy(device.tx, msg, FRAME_SIZE);
    // transieve data
    spi_transfer();
    return FRAME_SIZE;
}

void IcoIO::setOutput(uint8_t channel, int16_t value, bool pin_val)
{
    create_pwm_msg(channel, value, pin_val);
    // transieve data
    spi_transfer();
}

// Gives you encoder and input values
IcoIO::ReadValue IcoIO::getInput(uint8_t channel)
{
    uint8_t opcode = OP_CODE::ENC_1 + channel - 1;

    uint8_t msg[FRAME_SIZE] = {
        opcode, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    memcpy(device.tx, msg, FRAME_SIZE);

    // transieve data
    spi_transfer();

    ReadBytes read_value;
    read_value.s[0] = static_cast<char>(device.rx[1]);
    read_value.s[1] = static_cast<char>(device.rx[2]);

    int16_t result = static_cast<int16_t>(read_value.v.enc);

    return read_value.v;
}

size_t IcoIO::update_io(IcoWrite write_val, IcoRead *read_val)
{
    int ret = create_all_msg(write_val);

    // transieve data
    spi_transfer();

    ReadBytes enc_bytes1;
    ReadBytes enc_bytes2;
    ReadBytes enc_bytes3;
    ReadBytes enc_bytes4;

    enc_bytes1.s[0] = static_cast<char>(device.rx[1]);
    enc_bytes1.s[1] = static_cast<char>(device.rx[2]);
    enc_bytes2.s[0] = static_cast<char>(device.rx[3]);
    enc_bytes2.s[1] = static_cast<char>(device.rx[4]);
    enc_bytes3.s[0] = static_cast<char>(device.rx[5]);
    enc_bytes3.s[1] = static_cast<char>(device.rx[6]);
    enc_bytes4.s[0] = static_cast<char>(device.rx[7]);
    enc_bytes4.s[1] = static_cast<char>(device.rx[8]);

    read_val->channel1 = enc_bytes1.v.enc;
    read_val->channel1_1 = enc_bytes1.v.pin1;
    read_val->channel1_2 = enc_bytes1.v.pin2;
    read_val->channel2 = enc_bytes2.v.enc;
    read_val->channel2_1 = enc_bytes2.v.pin1;
    read_val->channel2_2 = enc_bytes2.v.pin2;
    read_val->channel3 = enc_bytes3.v.enc;
    read_val->channel3_1 = enc_bytes3.v.pin1;
    read_val->channel3_2 = enc_bytes3.v.pin2;
    read_val->channel4 = enc_bytes4.v.enc;
    read_val->channel4_1 = enc_bytes4.v.pin1;
    read_val->channel4_2 = enc_bytes4.v.pin2;
    return FRAME_SIZE;
}

int IcoIO::create_pwm_msg(uint8_t channel, int16_t value, bool pin_val)
{
    WriteValue write_value;
    WriteBytes write_bytes;
    if (value < 0)
    {
        write_value.pwm_dir = 0;
        if (value < -MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = -value;
        }
    }
    else
    {
        write_value.pwm_dir = 1;
        if (value > MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = value;
        }
    }

    write_value.pin_val = static_cast<unsigned int>(pin_val);

    write_bytes.n = write_value;
    uint8_t opcode = OP_CODE::PWM_1 + channel - 1;
    uint8_t pwm_cmd[FRAME_SIZE] = {
        opcode, 0, write_bytes.s[0], write_bytes.s[1],
        0, 0, 0, 0,
        0, 0, 0, 0};
    memcpy(device.tx, pwm_cmd, FRAME_SIZE);
    return 1;
}

int IcoIO::create_all_msg(IcoWrite data)
{
    WriteArray write_array;

    WriteValue val1 = calc_value(data.pwm1, data.val1);
    WriteValue val2 = calc_value(data.pwm2, data.val2);
    WriteValue val3 = calc_value(data.pwm3, data.val3);
    WriteValue val4 = calc_value(data.pwm4, data.val4);

    write_array.pwm_val1 = val1.pwm_val;
    write_array.pwm_dir1 = val1.pwm_dir;
    write_array.pin_val1 = val1.pin_val;
    write_array.pwm_val2 = val2.pwm_val;
    write_array.pwm_dir2 = val2.pwm_dir;
    write_array.pin_val2 = val2.pin_val;
    write_array.pwm_val3 = val3.pwm_val;
    write_array.pwm_dir3 = val3.pwm_dir;
    write_array.pin_val3 = val3.pin_val;
    write_array.pwm_val4 = val4.pwm_val;
    write_array.pwm_dir4 = val4.pwm_dir;
    write_array.pin_val4 = val4.pin_val;

    WriteArrayBytes write_bytes;
    write_bytes.in = write_array;
    uint8_t msg[FRAME_SIZE] = {
        OP_CODE::ALL, 0,
        write_bytes.out[0], write_bytes.out[1],
        write_bytes.out[2], write_bytes.out[3],
        write_bytes.out[4], write_bytes.out[5],
        write_bytes.out[6], write_bytes.out[7],
        0, 0};

    memcpy(device.tx, msg, FRAME_SIZE);
    return 1;
}

IcoIO::WriteBytes IcoIO::create_pwm_bytes(int16_t pwm_val, bool pin_val)
{
    WriteValue write_value;
    WriteBytes write_bytes;
    if (pwm_val < 0)
    {
        write_value.pwm_dir = 0;
        if (pwm_val < -MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = -pwm_val;
        }
    }
    else
    {
        write_value.pwm_dir = 1;
        if (pwm_val > MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = pwm_val;
        }
    }

    write_value.pin_val = static_cast<unsigned int>(pin_val);
    write_bytes.n = write_value;
    return write_bytes;
}

IcoIO::WriteValue IcoIO::calc_value(int pwm_val, bool pin_val)
{
    WriteValue write_value;
    if (pwm_val < 0)
    {
        write_value.pwm_dir = 0;
        if (pwm_val < -MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = -pwm_val;
        }
    }
    else
    {
        write_value.pwm_dir = 1;
        if (pwm_val > MAX_PWM_VALUE)
        {
            write_value.pwm_val = MAX_PWM_VALUE;
        }
        else
        {
            write_value.pwm_val = pwm_val;
        }
    }
    write_value.pin_val = static_cast<unsigned int>(pin_val);
    return write_value;
}
