/** MBED headers */
#include "mbed.h"
#include "PinNames.h"
#include "rtos.h"
#include "EthernetInterface.h"
#include "TCPServer.h"
#include "TCPSocket.h"
#include "lpc_phy.h"
#include "CDCE906.h"

#define DP8_SPEED10MBPS (1 << 1)    /**< 1=10MBps speed */
#define DP8_VALID_LINK  (1 << 0)    /**< 1=Link active */

#define BUFSIZE         256
#define SERVER_PORT     6791

/* MBED Reset function */
extern "C" void mbed_reset();

// Hardware Initialization - MBED

// MBED pins
RawSerial pc(P0_2, P0_3); // Serial USB port. (NOTE: All printf() calls are redirected to this port)

DigitalOut led1(P1_18);
DigitalOut led2(P1_20);
DigitalOut led3(P1_21);
DigitalOut led4(P1_23);
DigitalIn sw1(P1_29);
DigitalIn sw2(P2_11);
DigitalIn sw3(P2_12);
DigitalIn sw4(P2_13);

bool get_eth_link_status(void)
{
    return (lpc_mii_read_data() & DP8_VALID_LINK) ? true : false;
}

int loopback_check( PinName n0, PinName n1 )
{
    DigitalIn *in;
    DigitalOut *out;

    in = new DigitalIn(n0, PullNone);
    if (n0 == P0_29) {
        DigitalIn usb_in(P0_30);
    }
    if (n0 == P0_30) {
        DigitalIn usb_in(P0_29);
    }
    out = new DigitalOut(n1);

    *out = 1;
    if (*in != 1) {
        return 1;
    }

    *out = 0;
    if (*in != 0) {
        return 1;
    }

    delete in;
    delete out;

    /* Reverse direction */
    in = new DigitalIn(n1, PullNone);
    out = new DigitalOut(n0);
    if (n0 == P0_29) {
        DigitalOut usb_out(P0_30);
    }
    if (n0 == P0_30) {
        DigitalOut usb_out(P0_29);
    }

    *out = 1;
    if (*in != 1) {
        return 1;
    }

    *out = 0;
    if (*in != 0) {
        return 1;
    }

    delete in;
    delete out;

    return 0;
}

int GPIO_loopback_test( void )
{
    uint8_t err = 0;
    uint8_t t;
    PinName loop_pair[9][2] = {
        {p8,p14},
        {p9,p10},
        {p17,p18},
        {p19,p20},
        {p21,p22},
        {p23,p24},
        {p25,p26},
        {P0_29,p27},
        {P0_30,p28}
    };

    printf("\nStarting GPIO Loopback test\n\r");

    for( t = 0; t < sizeof(loop_pair)/sizeof(loop_pair[0]); t++) {
        printf("Loopback pair %d: ", t);
        if (loopback_check(loop_pair[t][0], loop_pair[t][1]) == 0) {
            printf("\t-> Pass!\n\r");
        } else {
            err = 1;
            printf("\t-> Fail!\n\r");
        }
    }

    printf("\n\r");
    return err;
}

#define PS_TEST_3V3_NOMINAL   (3.3)
#define PS_TEST_3V3_TOLERANCE (PS_TEST_3V3_NOMINAL*0.1)
#define PS_TEST_3V3_LOW       (PS_TEST_3V3_NOMINAL-PS_TEST_3V3_TOLERANCE)
#define PS_TEST_3V3_HIGH      (PS_TEST_3V3_NOMINAL+PS_TEST_3V3_TOLERANCE)

#define PS_TEST_5V_NOMINAL    (5.0)
#define PS_TEST_5V_TOLERANCE  (PS_TEST_5V_NOMINAL*0.1)
#define PS_TEST_5V_LOW        (PS_TEST_5V_NOMINAL-PS_TEST_5V_TOLERANCE)
#define PS_TEST_5V_HIGH       (PS_TEST_5V_NOMINAL+PS_TEST_5V_TOLERANCE)

int power_supply_test( void )
{
    uint8_t err = 0;
    AnalogIn adc_3v3(p15);
    AnalogIn adc_5v(p16);
    float adc_read = 0;

    printf("\nStarting Power Supply level test\n\r");

    adc_read = adc_3v3.read()*3.3*2;
    printf("Power Supply 3.3v read: %f", adc_read);
    if ( (adc_read <= PS_TEST_3V3_HIGH) && (adc_read >= PS_TEST_3V3_LOW) ) {
        printf("\t-> Pass!\n\r");
    } else {
        printf("\t-> Fail!\n\r");
        err = 1;
    }

    adc_read = adc_5v.read()*3.3*2;
    printf("Power Supply 5.0v read: %f", adc_read);
    if ( (adc_read <= PS_TEST_5V_HIGH) && (adc_read >= PS_TEST_5V_LOW) ) {
        printf("\t-> Pass!\n\r");
    } else {
        printf("\t-> Fail!\n\r");
        err = 1;
    }

    return err;
}

unsigned int random( void )
{
    unsigned int x = 0;
    unsigned int iRandom = 0;

    AnalogIn analog(p15);

    for (x = 0; x <= 32; x += 2)
    {
        iRandom += ((analog.read_u16() % 3) << x);
        wait_us (10);
    }

    return iRandom;
}

int feram_test( void )
{
    I2C feram_i2c( P0_19, P0_20 );

    DigitalOut feram_wp( P0_21 );

    feram_wp = 0;

    const uint8_t slave_id = 0xA;
    uint8_t page, addr;
    uint16_t byte;
    uint8_t err = 1;
    uint16_t err_write = 0, err_read = 0;
    uint8_t data[2];
    uint8_t mac[6];
    uint8_t test_pattern[256];

    /* Fill test pattern array with random numbers */
    for (byte = 0; byte < sizeof(test_pattern); byte++) {
        test_pattern[byte] = random()%256;
    }

    printf("\n\rStarting FeRAM test\n\r");

#if 0
    if (sw2.read() == 0) {
        mac[0] = 0xD8;
        mac[1] = 0x80;
        mac[2] = 0x38;
        mac[3] = 0xA9;
        mac[4] = 0x88;
        mac[5] = 0x2E;

        for (byte = 0x0; byte < sizeof(mac); byte++) {
            addr = (slave_id << 4);
            data[0] = byte;
            data[1] = mac[byte];
            err_write += feram_i2c.write(addr, (char *)data, 2);
        }
    }
#endif

    /* Read stored MAC address and save it */
    addr = (slave_id << 4);
    data[0] = 0x0;
    feram_i2c.write(addr, (char *)data, 1);
    err_read += feram_i2c.read(addr, (char *)mac, 6);

    printf("Writing test pattern...");
    /* Write test pattern on the FeRAM */
    for (page = 0; page <= 7; page++) {
        for (byte = 0; byte <= 0xFF; byte++) {
            addr = (slave_id << 4) | (page << 1);
            data[0] = byte;
            data[1] = test_pattern[byte];
            err_write += feram_i2c.write(addr, (char *)data, 2);
        }
    }

    if (err_write == 0) {
        printf("\t-> Pass!\n\r");
        err = 0;
    } else {
        printf("\t-> Fail with %d errors on I2C write\n\r", err_write);
        return 1;
    }

    printf("Reading back test pattern...");
    /* Check test pattern */
    for (page = 0; page <= 7; page++) {
        for (byte = 0; byte <= 0xFF; byte++) {
            addr = (slave_id << 4) | (page << 1);
            data[0] = byte;
            feram_i2c.write(addr, (char *)data, 1);
            err_read += feram_i2c.read(addr, (char *)data, 1);

            if (data[0] != test_pattern[byte]) {
                err_read++;
            }
        }
    }

    if (err_read == 0) {
        printf("\t-> Pass!\n\r");
        err = 0;
    } else {
        printf("\t-> Fail with %d errors on I2C read!\n\r", err_read);
        return 1;
    }


    /* Zero-write the FeRAM */
    memset(test_pattern, 0, 256);
    for (page = 0; page <= 7; page++) {
        for (byte = 0; byte <= 0xFF; byte++) {
            addr = (slave_id << 4) | (page << 1);
            data[0] = byte;
            data[1] = test_pattern[byte];
            feram_i2c.write(addr, (char *)data, 2);
        }
    }

    /* Restore MAC address info to FeRAM */
    for (byte = 0x0; byte < sizeof(mac); byte++) {
        addr = (slave_id << 4);
        data[0] = byte;
        data[1] = mac[byte];
        err_write += feram_i2c.write(addr, (char *)data, 2);
    }

    return err;
}

uint8_t feram_store_mac( void )
{
    I2C feram_i2c( P0_19, P0_20 );

    DigitalOut feram_wp( P0_21 );

    feram_wp = 0;

    const uint8_t slave_id = 0xA;
    uint8_t addr;
    uint16_t byte;
    uint8_t err = 1;
    uint16_t err_write = 0, err_read = 0;
    uint8_t data[2];

    uint8_t mac[6];


    printf("\n\r Reading MAC Address...\n\r");

    addr = (slave_id << 4);
    data[0] = 0xFA;
    feram_i2c.write(addr, (char *)data, 1);

    err_read += feram_i2c.read(addr, (char *)mac, 6);

    if (err_read == 0) {
        printf("\t-> Pass!\n\r");
        printf("MAC:");
        for (byte = 0; byte < sizeof(mac); byte++) {
            printf(" 0x%X", mac[byte]);
        }
        printf("\n\r");
    } else {
        printf("\t-> Fail with %d errors on I2C read!\n\r", err_read);
        err = 0;
    }

    printf("\n\r Writing MAC to FeRAM at 0x0...\n\r");

    /* Write test pattern on the FeRAM */
    for (byte = 0x0; byte < sizeof(mac); byte++) {
        addr = (slave_id << 4);
        data[0] = byte;
        data[1] = mac[byte];
        err_write += feram_i2c.write(addr, (char *)data, 2);
    }

    if (err_write == 0) {
        printf("\t-> Pass!\n\r");
    } else {
        printf("\t-> Fail with %d errors on I2C write!\n\r", err_read);
        err = 0;
    }

    return err;
}

#define LDR_LIGHT_THRESHOLD 2.9

int leds_test( void )
{
    int err = 0;
    AnalogIn ldr(p18);
    DigitalOut ldr_loop(p17);
    ldr_loop = 1;

    PinName leds[4] = {
        P1_18,
        P1_20,
        P1_21,
        P1_23,
    };

    DigitalOut *led;

    /* Turn off all the LEDs */
    for( uint8_t t = 0; t < sizeof(leds)/sizeof(leds[0]); t++) {
        led = new DigitalOut(leds[t]);
        *led = 0;
        delete led;
    }


    for( uint8_t t = 0; t < sizeof(leds)/sizeof(leds[0]); t++) {
        printf("LED test: LED%d", t+1);
        led = new DigitalOut(leds[t]);

        *led = 1;
        Thread::wait(500);
        if ((ldr.read()*3.3) < LDR_LIGHT_THRESHOLD) {
            printf("\t-> Pass!\n\r");
        } else {
            printf("\t-> Fail!\n\r");
            err = 1;
        }

        *led = 0;
        delete led;
    }

    return err;
}

void pll_cfg( void )
{
    I2C pll_i2c(P0_27, P0_28);

    CDCE906 pll( pll_i2c, 0b11010010 );

    pll.cfg_eth();
}

int ethernet_test( void )
{
    int err=0;
    // Ethernet initialization
    EthernetInterface net;
    TCPSocket client;
    SocketAddress client_addr;
    TCPServer server;

    const char msg[] = "Test msg!";
    uint8_t buf[10];
    uint8_t t = 0;
    uint8_t recv_sz = 0;

    printf("\n\rStarting Ethernet test\n\r");

    printf("Configuring 50MHz PLL\n\r");
    pll_cfg();

#if defined(ETH_DHCP)
    err = net.set_dhcp(true);
#else
#if defined(ETH_FIXIP)
    err = net.set_network(ETH_IP,ETH_MASK,ETH_GATEWAY);
#else
#error "No Ethernet addressing mode selected! Please choose between DHCP or Fixed IP!"
#endif
#endif

    printf("Trying to initialize ETH interface...");

    do {
        err = net.connect();
        t++;
    } while ( (err != 0) && (t <= 5) );

    if ( err == 0 ) {
        printf("\t-> Pass!\n\r");
    } else {
        printf("\t-> Fail!\n\r");
        return 1;
    }

    printf("IP: %s\n\r", net.get_ip_address());
    printf("RFFE MAC Address: %s\n\r", net.get_mac_address());
    printf("Listening on port: %d\n\r", SERVER_PORT);

    /* Bind tcp server */
    server.open(&net);
    server.bind(net.get_ip_address(), SERVER_PORT);
    server.listen();

    client.set_blocking(true);
    server.accept(&client, &client_addr);

    recv_sz = client.recv(&buf[0], sizeof(msg));
    err = 1;
    if (recv_sz > 0 ) {
        if( strcmp((char *)msg, (char *)buf) == 0 ) {
            printf("Received correct message via ethernet -> Pass!\n\r");
            err = 0;
        } else {
            /* Add terminating char to prevent gargabe print */
            buf[9] = 0;
            printf("Received \"%s\" expected \"%s\" -> Fail!\n\r", buf, msg);
            /* clear buffer */
            memset(buf, 0, sizeof(buf));
        }
    } else {
        printf("Client Disconnected!\n");
    }

    client.close();

    return err;
}

Ticker blink;

static void blink_callback( void )
{
    led1 = !led1;
    led2 = !led2;
    led3 = !led3;
    led4 = !led4;
}

int main( void )
{
    //Init serial port for info printf
    pc.baud(115200);

    printf("Starting rffe-uc tests!\n\r");

    int err = 0;

    err += ((leds_test() & 1) << 0);
    err += ((GPIO_loopback_test() & 1) << 1);
    err += ((power_supply_test() & 1) << 2);
    err += ((feram_test() & 1) << 3);
    err += ((ethernet_test() & 1) << 4);
    //feram_store_mac();

    if (err == 0) {
        printf("All tests passed!\n\r");
        blink.attach(&blink_callback, 0.5);
    } else {
        printf("Board tests failed!\n\r");
    }


    while(1);
}
