#define __STDC_LIMIT_MACROS
#include <stdint.h>

// Fix for PROGMEM warning.
#include <avr/pgmspace.h>
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))
#undef PSTR
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];}))

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <EtherCard.h>

// 'off_t' is defined in stdio.h which we don't use on Arduino.
// We don't really need this but it's useful to document the purpose of
// a variable with a type.
typedef size_t off_t;

////////////////////////////////////////////////////////////////////////
// GPS

#define GPS_RATE_1HZ  F("$PMTK220,1000*1F")
#define GPS_RATE_5HZ  F("$PMTK220,200*2C")
#define GPS_RATE_10HZ F("$PMTK220,100*2F")

// NOTE: You must set SoftwareSerial.h _SS_MAX_RX_BUFF 64 to 128 in
// order to receive more than one NMEA string.
#define GPS_OUTPUT_GGA       F("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29")
#define GPS_OUTPUT_RMCGGA    F("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28")
#define GPS_OUTPUT_RMCGGAGSA F("$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0*29")
#define GPS_OUTPUT_RMCGSA    F("$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0*28")
#define GPS_OUTPUT_RMC       F("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29")
#define GPS_OUTPUT_ALLDATA   F("$PMTK314,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0*29")

#define GPS_BAUD_4800   F("$PMTK251,4800*14")
#define GPS_BAUD_9600   F("$PMTK251,9600*17")
#define GPS_BAUD_19200  F("$PMTK251,19200*22")
#define GPS_BAUD_38400  F("$PMTK251,38400*27")
#define GPS_BAUD_57600  F("$PMTK251,57600*2C")
#define GPS_BAUD_115200 F("$PMTK251,115200*1F")

#define GPS_RX 8
#define GPS_TX 9

static TinyGPS gps;
static SoftwareSerial gps_serial(GPS_RX, GPS_TX);

static void gps_setup()
{
    // Ensure GPS is using baud rate of 9600.
    gps_serial.begin(9600);
    gps_serial.println(GPS_BAUD_9600);
    gps_serial.begin(19200);
    gps_serial.println(GPS_BAUD_9600);
    gps_serial.begin(38400);
    gps_serial.println(GPS_BAUD_9600);
    gps_serial.begin(57600);
    gps_serial.println(GPS_BAUD_9600);
    gps_serial.begin(115200);
    gps_serial.println(GPS_BAUD_9600);

    // Set our baud rate to 9600.
    gps_serial.begin(9600);

    // Set 1Hz update rate.
    gps_serial.println(GPS_RATE_1HZ);

    // Set the output sentence to RMC. RMC is the only sentence
    // available on this GPS receiver which has the date as well
    // as the time in the message.
    gps_serial.println(GPS_OUTPUT_RMC);
}

////////////////////////////////////////////////////////////////////////
// PPS

// ATmega 168 and 328:
//  - Pin 2 = Interrupt 0
//  - Pin 3 = Interrupt 1

#define LED_PIN A4
#define PPS_PIN 3
#define PPS_INT 1

static volatile uint32_t g_pps_time;
static volatile bool g_waiting_for_fix;

static void pps_interrupt()
{
    g_pps_time = micros();
    g_waiting_for_fix = true;
}

static void pps_setup()
{
    g_pps_time = micros();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(PPS_PIN, INPUT);
    digitalWrite(PPS_PIN, HIGH);
    attachInterrupt(PPS_INT, pps_interrupt, RISING);
}

static void pps_loop()
{
    uint32_t pps_state = micros() - g_pps_time < 100000 ? HIGH : LOW;
    digitalWrite(LED_PIN, pps_state);
}

////////////////////////////////////////////////////////////////////////
// Web Server Utils

extern int __bss_end;
extern void *__brkval;

static int32_t get_free_memory()
{
    int32_t free_memory;

    if(__brkval)
        free_memory = ((int32_t)&free_memory) - ((int32_t)__brkval);
    else
        free_memory = ((int32_t)&free_memory) - ((int32_t)&__bss_end);

    return free_memory;
}

static char* print_dms(float degrees, char* buffer)
{
    float fValue = fabs(degrees);
    int deg = (int) fValue;
    float remainder = fValue-deg;
    fValue = remainder*60.0;
    int mins = (int) fValue;
    remainder = fValue - mins;
    fValue = remainder * 100.0;
    int secs = (int) fValue;
    remainder = fValue- secs;
    fValue = remainder *1000;
    int ptSec = (int) fValue;

    sprintf(buffer, "%d:%d:%d.%d", deg, mins, secs, ptSec);

    return buffer;
}

static char* print_ms_time(long ms, char* buffer)
{
    long t  = ms / 1000;
    word h  = t / 3600;
    byte m  = (t / 60) % 60;
    byte s  = t % 60;
    byte ss = (ms % 1000)/100;

    sprintf(buffer, "%d%d:%0d%d:%d%d.%d",
        h/10, h%10, m/10, m%10, s/10, s%10, ss);

    return buffer;
}

static char* print_gps_time(char *buffer)
{
    int year;
    byte month, day, hour, minute, second, hundredths;

    gps.crack_datetime(
        &year, &month, &day,
        &hour, &minute, &second, &hundredths);

    sprintf(buffer, "%d-%02d-%02d %02d:%02d:%02d.%02d",
        year, month, day,
        hour, minute, second, hundredths);
}

////////////////////////////////////////////////////////////////////////
// Web Server

static void write_status_page(BufferFiller &response)
{
    char buffer[32];

    response.emit_p(PSTR(
        "HTTP/1.0 200 OK\r\n"
        "Content-Type: text/html\r\n"
        "Pragma: no-cache\r\n"
        "\r\n"
        "<meta http-equiv='refresh' content='1'/>"
        "<title>Jystic NTP Server v0.1</title>"));

    print_gps_time(buffer);
    response.emit_p(PSTR("<h1>UTC $S<h1>"), buffer);

    response.emit_p(PSTR("Running $S\n"),
        print_ms_time(millis(), buffer));

    response.emit_p(PSTR("Free mem $D\n"),
        get_free_memory() );

    uint32_t time_since_pps = micros() - g_pps_time;
    response.emit_p(PSTR("Since PPS $S\n"),
        print_ms_time(time_since_pps/1000, buffer));
}

// NOTE: The request and response share the same underlying buffer, so
// NOTE: make sure you don't access the request after you start writing
// NOTE: the response.
static word process_http_request(const char *request, BufferFiller &response)
{
    //Serial.println(request);
    write_status_page(response);
}

static void http_loop(word pos)
{
    const char *request   = (const char *)ether.buffer + pos;
    BufferFiller response = ether.tcpOffset();

    // Process the request and build the response.
    process_http_request(request, response);

    // Send the response to the client.
    ether.httpServerReply(response.position());
}

////////////////////////////////////////////////////////////////////////
// Ethernet Utils

#define g_packet ether.buffer

inline static uint8_t read_uint8(off_t offset)
{
    return g_packet[offset];
}

static uint16_t read_uint16(off_t offset)
{
    return ((uint16_t)g_packet[offset + 0] << 8)
         | ((uint16_t)g_packet[offset + 1]);
}

static uint32_t read_uint32(off_t offset)
{
    return ((uint32_t)g_packet[offset + 0] << 24)
         | ((uint32_t)g_packet[offset + 1] << 16)
         | ((uint32_t)g_packet[offset + 2] <<  8)
         | ((uint32_t)g_packet[offset + 3]      );
}

static void write_uint32(off_t offset, uint32_t value)
{
    g_packet[offset + 0] = (value >> 24) & 0xFF;
    g_packet[offset + 1] = (value >> 16) & 0xFF;
    g_packet[offset + 2] = (value >>  8) & 0xFF;
    g_packet[offset + 3] = (value      ) & 0xFF;
}

static bool packet_is_udp(size_t len)
{
    return len >= 42
        && g_packet[ETH_TYPE_H_P] == ETHTYPE_IP_H_V
        && g_packet[ETH_TYPE_L_P] == ETHTYPE_IP_L_V
        && g_packet[IP_HEADER_LEN_VER_P] == 0x45
        && g_packet[IP_PROTO_P] == IP_PROTO_UDP_V;
}

static bool udp_dst_port_is(uint8_t port)
{
    return g_packet[UDP_DST_PORT_H_P] == 0
        && g_packet[UDP_DST_PORT_L_P] == port;
}

////////////////////////////////////////////////////////////////////////
// NTP Time Utils

#define NTP_EPOCH0 1900UL
#define NTP_FRAC_MAX UINT32_MAX

#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define SECS_PER_YEAR (SECS_PER_DAY * 365UL)

const uint16_t g_month_days[] = {0,31,59,90,120,151,181,212,243,273,304,334};

// The 64-bit NTP timestamp.
typedef struct {
    uint32_t secs; // Seconds since 1 Jan 1900 UTC
    uint32_t frac; // Fractions of a second, 1 unit == 1/UINT32_MAX secs
} timestamp_t;

inline static bool is_leap_year(uint16_t year)
{
    return (year % 4 == 0) && (year % 100 != 0 || year % 400 == 0);
}

static timestamp_t timestamp(
    uint16_t year,
    uint8_t month,
    uint8_t day,
    uint8_t hour,
    uint8_t minute,
    uint8_t second,
    uint8_t hundredths)
{
    timestamp_t t;

    t.secs = SECS_PER_YEAR * (year - NTP_EPOCH0)
           + SECS_PER_DAY  * g_month_days[month-1]
           + SECS_PER_DAY  * (day - 1)
           + SECS_PER_HOUR * hour
           + SECS_PER_MIN  * minute
           + second;

    // add extra day for each leap year since epoch 0
    for (uint16_t y = NTP_EPOCH0; y < year; y++)
    {
        if (is_leap_year(y))
        {
            t.secs += SECS_PER_DAY;
        }
    }

    // add extra day if this year is a leap year and we
    // have passed February 29
    if (is_leap_year(year) && month > 2)
    {
        t.secs += SECS_PER_DAY;
    }

    t.frac = hundredths * (NTP_FRAC_MAX/100);

    return t;
}

////////////////////////////////////////////////////////////////////////
// NTP Server

#define NTP_PORT 123
#define NTP_DATA_LEN 48

#define NTP_DATA_P            (UDP_DATA_P + 0)
#define NTP_FLAGS_P           (NTP_DATA_P + 0)
#define NTP_STRATUM_P         (NTP_DATA_P + 1)
#define NTP_POLL_P            (NTP_DATA_P + 2)
#define NTP_PRECISION_P       (NTP_DATA_P + 3)
#define NTP_ROOT_DELAY_P      (NTP_DATA_P + 4)
#define NTP_ROOT_DISPERSION_P (NTP_DATA_P + 8)
#define NTP_REFERENCE_ID_P    (NTP_DATA_P + 12)
#define NTP_REFERENCE_TIME_P  (NTP_DATA_P + 16)
#define NTP_TIME1_P           (NTP_DATA_P + 24)
#define NTP_TIME2_P           (NTP_DATA_P + 32)
#define NTP_TIME3_P           (NTP_DATA_P + 40)

static void read_flags(uint8_t *leap_indicator, uint8_t *version, uint8_t *mode)
{
    uint8_t flags = read_uint8(NTP_FLAGS_P);

    *leap_indicator = flags >> 6;
    *version        = (flags & 0x38) >> 3;
    *mode           = (flags & 0x7);
}

static timestamp_t last_fix_time()
{
    int year;
    byte month, day, hour, minute, second, hundredths;

    gps.crack_datetime(
        &year, &month, &day,
        &hour, &minute, &second, &hundredths);

    return timestamp(
        year, month, day,
        hour, minute, second, hundredths);
}

static timestamp_t micros_to_timestamp(uint32_t micros)
{
    uint64_t us_since_pps = micros - g_pps_time;
    uint64_t frac_since_pps = (NTP_FRAC_MAX * us_since_pps) / 1000000;

    timestamp_t time = last_fix_time();

    if (g_waiting_for_fix)
    {
        // pps is for the next fix
        time.secs++;
    }

    uint64_t frac = time.frac + frac_since_pps;

    while (frac > NTP_FRAC_MAX)
    {
        time.secs++;
        frac -= NTP_FRAC_MAX;
    }

    time.frac += frac;

    return time;
}

static volatile uint32_t g_receive_time;

static timestamp_t last_receive_time()
{
    return micros_to_timestamp(g_receive_time);
}

static timestamp_t current_time()
{
    return micros_to_timestamp(micros());
}

static timestamp_t read_timestamp(off_t offset)
{
    timestamp_t t;
    t.secs = read_uint32(offset+0);
    t.frac = read_uint32(offset+4);
    return t;
}

static void write_timestamp(off_t offset, timestamp_t time)
{
    write_uint32(offset+0, time.secs);
    write_uint32(offset+4, time.frac);
}

// Check RFC5905 for further details about the format.
const uint8_t ntp_header[] PROGMEM = {
    0x24,    // No warning / Version 3 / Server (packed bitfield)
    1,       // Stratum 1 server (connected to GPS)
    3,       // Polling interval
    -20,     // Precision in log2 seconds (2^-20 is about 1us)
    0,0,0,0, // Delay to reference clock (we have PPS, so effectively zero)
    0,0,0,1, // Jitter of reference clock (the PPS is rated to +/- 50ns)
    'G','P','S',0, // Reference ID - we are using a GPS receiver
};

static void ntp_loop()
{
    if (!udp_dst_port_is(NTP_PORT)) return;

    uint16_t len = read_uint16(UDP_LEN_H_P);

    if (len < UDP_HEADER_LEN + NTP_DATA_LEN) return;

    timestamp_t time1 = read_timestamp(NTP_TIME3_P);
    timestamp_t time2 = last_receive_time();

    memcpy_P(g_packet + NTP_DATA_P, ntp_header, sizeof ntp_header);

    timestamp_t ref_time = last_fix_time();

    write_timestamp(NTP_REFERENCE_TIME_P, ref_time);
    write_timestamp(NTP_TIME1_P, time1);
    write_timestamp(NTP_TIME2_P, time2);

    timestamp_t time3 = current_time();
    write_timestamp(NTP_TIME3_P, time3);

    ether.makeUdpReply(
        (char *)(g_packet + NTP_DATA_P), NTP_DATA_LEN, NTP_PORT);

    Serial.print(F("NTP request from "));
    uint8_t *dst_ip = &g_packet[IP_DST_P];
    ether.printIp("", dst_ip);
}

////////////////////////////////////////////////////////////////////////
// Ethernet

// MAC address must be unique on the LAN.
static uint8_t mac_address[] = { 0x74,0x69,0x69,0x2D,0x30,0x32 };
static uint8_t ip_address[] = { 1,1,1,100 };

#define ETHERNET_BUFFER_SIZE 550
uint8_t Ethernet::buffer[ETHERNET_BUFFER_SIZE];

#define ETHERNET_CS_PIN 10

#define ETHERNET_INT 0 // Pin 2 (on ATmega 168 and 328)

static void packet_received_interrupt()
{
    g_receive_time = micros();
}

static void ethernet_setup()
{
    if (!ether.begin(ETHERNET_BUFFER_SIZE, mac_address, ETHERNET_CS_PIN))
        Serial.println(F("Failed to access Ethernet controller"));

    ether.staticSetup(ip_address);

    attachInterrupt(ETHERNET_INT, packet_received_interrupt, FALLING);
}

static void ethernet_loop()
{
    word len = ether.packetReceive();

    if (packet_is_udp(len))
    {
        ntp_loop();
    }

    word pos = ether.packetLoop(len);

    // If we received a TCP packet then 'pos' is the index
    // where the packet's data is stored in 'ether.buffer'.
    if (pos != 0)
    {
        // Assume that it's an HTTP request.
        http_loop(pos);
    }
}

////////////////////////////////////////////////////////////////////////
// Main

// Baud rate for the UART serial port used for diagnostics.
#define UART_BAUD_RATE 57600

void setup()
{
    Serial.begin(UART_BAUD_RATE);
    Serial.println(F("Jystic NTP Server v0.1"));

    pps_setup();
    gps_setup();
    ethernet_setup();
}

static void gps_loop()
{
    while (gps_serial.available())
    {
        char c = gps_serial.read();
        //Serial.write(c);
        if (gps.encode(c))
        {
            g_waiting_for_fix = false;
        }
    }
}

void loop()
{
    pps_loop();
    gps_loop();
    ethernet_loop();
}

// vim: ft=arduino
