// Fix for PROGMEM warning.
#include <avr/pgmspace.h>
#undef PROGMEM
#define PROGMEM __attribute__((section(".progmem.data")))

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <EtherCard.h>

////////////////////////////////////////////////////////////////////////
// GPS

#define GPS_RATE_1HZ  "$PMTK220,1000*1F"
#define GPS_RATE_5HZ  "$PMTK220,200*2C"
#define GPS_RATE_10HZ "$PMTK220,100*2F"

// NOTE: You must set SoftwareSerial.h _SS_MAX_RX_BUFF 64 to 128 in
// order to receive more than one NMEA string.
#define GPS_OUTPUT_GGA       "$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define GPS_OUTPUT_RMCGGA    "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define GPS_OUTPUT_RMCGGAGSA "$PMTK314,0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define GPS_OUTPUT_RMCGSA    "$PMTK314,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0*28"
#define GPS_OUTPUT_RMC       "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29"
#define GPS_OUTPUT_ALLDATA   "$PMTK314,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0*29"

#define GPS_BAUD_4800   "$PMTK251,4800*14"
#define GPS_BAUD_9600   "$PMTK251,9600*17"
#define GPS_BAUD_19200  "$PMTK251,19200*22"
#define GPS_BAUD_38400  "$PMTK251,38400*27"
#define GPS_BAUD_57600  "$PMTK251,57600*2C"
#define GPS_BAUD_115200 "$PMTK251,115200*1F"

#define GPS_RX 8
#define GPS_TX 9

static TinyGPS gps;
static SoftwareSerial gps_serial(GPS_RX, GPS_TX);

static float g_latitude;
static float g_longitude;
static float g_speed;
static float g_course;

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

static void gps_loop()
{
    bool newData = false;
    unsigned long chars;

    while (gps_serial.available())
    {
        char c = gps_serial.read();
        //Serial.write(c);

        if (gps.encode(c)) // Did a new valid sentence come in?
        {
            newData = true;
        }
    }

    if (newData)
    {
        unsigned long age;

        gps.f_get_position(&g_latitude, &g_longitude, &age);
        g_speed  = gps.f_speed_kmph();
        g_course = gps.f_course();
    }
}

////////////////////////////////////////////////////////////////////////
// PPS

// ATmega 168 and 328:
//  - Pin 2 = Interrupt 0
//  - Pin 3 = Interrupt 1

#define LED_PIN A4
#define PPS_PIN 3
#define PPS_INT 1

static volatile long g_time_pps;

static void pps_interrupt()
{
    g_time_pps = millis();
}

static void pps_setup()
{
    g_time_pps = millis();

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    pinMode(PPS_PIN, INPUT);
    digitalWrite(PPS_PIN, HIGH);
    attachInterrupt(PPS_INT, pps_interrupt, RISING);
}

static void pps_loop()
{
    int pps_state = millis() - g_time_pps < 100 ? HIGH : LOW;
    digitalWrite(LED_PIN, pps_state);
}

////////////////////////////////////////////////////////////////////////
// Web Server Utils

extern int __bss_end;
extern void *__brkval;

static int get_free_memory()
{
    int free_memory;

    if((int)__brkval == 0)
        free_memory = ((int)&free_memory) - ((int)&__bss_end);
    else
        free_memory = ((int)&free_memory) - ((int)__brkval);

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
        "<title>Enchantee Log</title>"));

    print_gps_time(buffer);
    response.emit_p(PSTR("<h1>UTC $S<h1>"), buffer);

    print_dms(g_latitude, buffer);
    response.emit_p(PSTR("<h1>Lat $S<h1>"), buffer);

    print_dms(g_longitude, buffer);
    response.emit_p(PSTR("<h1>Lon $S<h1>"), buffer);

    unsigned short sats = gps.satellites();
    response.emit_p(PSTR("<h1>Satellites $D</h1>"),
        sats == TinyGPS::GPS_INVALID_SATELLITES ? 0 : sats);

    response.emit_p(PSTR("Running $S\n"),
        print_ms_time(millis(), buffer));

    response.emit_p(PSTR("Free mem $D\n"),
        get_free_memory() );

    long time_since_pps = millis() - g_time_pps;
    response.emit_p(PSTR("Since PPS $S\n"),
        print_ms_time(time_since_pps, buffer));
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

#define gPB ether.buffer

inline static uint8_t read_uint8(size_t offset)
{
    return gPB[offset];
}

static uint16_t read_uint16(size_t offset)
{
    return ((uint16_t)gPB[offset + 0] << 8) |
            (uint16_t)gPB[offset + 1];
}

static uint32_t read_uint32(size_t offset)
{
    return ((uint32_t)gPB[offset + 0] << 24) |
           ((uint32_t)gPB[offset + 1] << 16) |
           ((uint32_t)gPB[offset + 2] <<  8)  |
            (uint32_t)gPB[offset + 3];
}

static bool packet_is_udp(word len)
{
    return len >= 42
        && gPB[ETH_TYPE_H_P] == ETHTYPE_IP_H_V
        && gPB[ETH_TYPE_L_P] == ETHTYPE_IP_L_V
        && gPB[IP_HEADER_LEN_VER_P] == 0x45
        && gPB[IP_PROTO_P] == IP_PROTO_UDP_V;
}

static bool udp_dst_port_is(uint8_t port)
{
    return gPB[UDP_DST_PORT_H_P] == 0
        && gPB[UDP_DST_PORT_L_P] == port;
}

////////////////////////////////////////////////////////////////////////
// NTP Server

#define NTP_PORT 123
#define NTP_MIN_LEN 56

#define NTP_FLAGS_P           (UDP_DATA_P + 0)
#define NTP_STRATUM_P         (UDP_DATA_P + 1)
#define NTP_POLL_P            (UDP_DATA_P + 2)
#define NTP_PRECISION_P       (UDP_DATA_P + 3)
#define NTP_ROOT_DELAY_P      (UDP_DATA_P + 4)
#define NTP_ROOT_DISPERSION_P (UDP_DATA_P + 8)
#define NTP_REFERENCE_ID_P    (UDP_DATA_P + 12)
#define NTP_REFERENCE_TIME_P  (UDP_DATA_P + 16)
#define NTP_TIME1_P           (UDP_DATA_P + 24)
#define NTP_TIME2_P           (UDP_DATA_P + 32)
#define NTP_TIME3_P           (UDP_DATA_P + 48)

static void read_flags(uint8_t *leap_indicator, uint8_t *version, uint8_t *mode)
{
    uint8_t flags = read_uint8(NTP_FLAGS_P);

    *leap_indicator = flags >> 6;
    *version        = (flags & 0x38) >> 3;
    *mode           = (flags & 0x7);
}

static void ntp_loop()
{
    if (!udp_dst_port_is(NTP_PORT)) return;

    word len = read_uint16(UDP_LEN_H_P);

    if (len < NTP_MIN_LEN) return;

    Serial.println();
    Serial.println("Received NTP Packet");

    ether.printIp("src.ip = ", &gPB[IP_SRC_P]);
    Serial.print("src.port = ");
    Serial.println(read_uint16(UDP_SRC_PORT_H_P));

    ether.printIp("dst.ip = ", &gPB[IP_DST_P]);
    Serial.print("dst.port = ");
    Serial.println(read_uint16(UDP_DST_PORT_H_P));

    uint8_t leap, version, mode;
    read_flags(&leap, &version, &mode);
    Serial.print("ntp.leap_indicator = ");
    Serial.println(leap);
    Serial.print("ntp.version = ");
    Serial.println(version);
    Serial.print("ntp.mode = ");
    Serial.println(mode);

    Serial.print("ntp.stratum = ");
    Serial.println(read_uint8(NTP_STRATUM_P));
    Serial.print("ntp.poll = ");
    Serial.println(read_uint8(NTP_POLL_P));
    Serial.print("ntp.precision = ");
    Serial.println(read_uint8(NTP_PRECISION_P));
    Serial.print("ntp.root_delay = ");
    Serial.println(read_uint32(NTP_ROOT_DELAY_P));
    Serial.print("ntp.root_dispersion = ");
    Serial.println(read_uint32(NTP_ROOT_DISPERSION_P));
    ether.printIp("ntp.reference_id = ", &gPB[NTP_REFERENCE_ID_P]);
}

////////////////////////////////////////////////////////////////////////
// Ethernet

// MAC address must be unique on the LAN.
static uint8_t mac_address[] = { 0x74,0x69,0x69,0x2D,0x30,0x32 };
static uint8_t ip_address[] = { 1,1,1,100 };

#define ETHERNET_BUFFER_SIZE 550
uint8_t Ethernet::buffer[ETHERNET_BUFFER_SIZE];

#define ETHERNET_CS_PIN 10

static void ethernet_setup()
{
    if (!ether.begin(ETHERNET_BUFFER_SIZE, mac_address, ETHERNET_CS_PIN))
        Serial.println("Failed to access Ethernet controller");

    ether.staticSetup(ip_address);
}

static void ethernet_loop()
{
    word len = ether.packetReceive();

    if (packet_is_udp(len))
    {
        ntp_loop();
    }
    else
    {
        word pos = ether.packetLoop(len);

        // If we received a TCP packet then 'pos' is the index
        // where the packet's data is stored in 'ether.buffer'.
        if (pos != 0)
        {
            // Assume that it's an HTTP request.
            http_loop(pos);
        }
    }
}

////////////////////////////////////////////////////////////////////////
// Main

// Baud rate for the UART serial port used for diagnostics.
#define UART_BAUD_RATE 57600

void setup()
{
    Serial.begin(UART_BAUD_RATE);
    Serial.println("Jystic NTP Server v0.1");

    pps_setup();
    gps_setup();
    ethernet_setup();
}

void loop()
{
    pps_loop();
    gps_loop();
    ethernet_loop();
}

// vim: ft=arduino
