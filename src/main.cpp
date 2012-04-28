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
    unsigned short sentences, failed;

    while (gps_serial.available())
    {
        char c = gps_serial.read();
        Serial.write(c);

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
// Ethernet

// MAC address must be unique on the LAN.
static byte mac_address[] = { 0x74,0x69,0x69,0x2D,0x30,0x32 };
static byte ip_address[] = { 1,1,1,100 };

byte Ethernet::buffer[550];

static void ethernet_setup()
{
    if (ether.begin(sizeof Ethernet::buffer, mac_address, 10) == 0)
        Serial.println( "Failed to access Ethernet controller");

    ether.staticSetup(ip_address);
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

static char* print_dms(float degrees, char* buf)
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
    sprintf(buf, "%d:%d:%d.%d",deg,mins,secs, ptSec);

    return buf;
}

static char* print_ms_time(long ms, char* buf)
{
    long t = ms / 1000;
    word h = t / 3600;
    byte m = (t / 60) % 60;
    byte s = t % 60;
    byte ss= (ms % 1000)/100;
    sprintf(buf, "%d%d:%0d%d:%d%d.%d", h/10, h%10, m/10, m%10, s/10, s%10, ss );
    return buf;
}

////////////////////////////////////////////////////////////////////////
// Web Server

static char printBuf[16];

static word build_http_response(word request)
{
    BufferFiller bfill = ether.tcpOffset();

    char* data = (char *) Ethernet::buffer + request;
#if SERIAL
    Serial.println(data);
#endif
    // receive buf hasn't been clobbered by reply yet
    if (strncmp("GET /c", data, 6) == 0)      //calibration details
    {
        bfill = ether.tcpOffset();
        bfill.emit_p(PSTR(
                    "HTTP/1.0 200 OK\r\n"
                    "Content-Type: text/html\r\n"
                    "Pragma: no-cache\r\n"
                    "\r\n"
                    "<meta http-equiv='refresh' content='1'/>"
                    "<title>Enchantee Log</title>"));
    }
    else  //general home page
    {
        bfill.emit_p(PSTR(
                    "HTTP/1.0 200 OK\r\n"
                    "Content-Type: text/html\r\n"
                    "Pragma: no-cache\r\n"
                    "\r\n"
                    "<meta http-equiv='refresh' content='1'/>"
                    "<title>Enchantee Log</title>"));

        print_dms(g_latitude, printBuf);
        bfill.emit_p(PSTR("<h1>Lat $S<h1>"), printBuf );
        print_dms(g_longitude, printBuf);
        bfill.emit_p(PSTR("<h1>Lon $S<h1>"), printBuf );
        bfill.emit_p(PSTR("<h1>Speed $D.$D</h1>"), (int)g_speed, (int)((g_speed-(int)g_speed)*10) );
        bfill.emit_p(PSTR("<h1>Course $D.$D</h1>"), (int)g_course, (int)((g_course - (int)g_course)*10) );
        bfill.emit_p(PSTR("<h1>Satellites $D</h1>"), gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites() );
        int hDop = gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop();
        bfill.emit_p(PSTR("<h1>Horz DOP $D.$D</h1>"), (int)hDop/100, (int)((hDop/100.0 - (int)(hDop/100))*100));
    }
    bfill.emit_p(PSTR("Running $S\n"), print_ms_time(millis(), printBuf));
    bfill.emit_p(PSTR("Free mem $D\n"), get_free_memory() );
    bfill.emit_p(PSTR("Since PPS $S\n"), print_ms_time(millis()-g_time_pps, printBuf));

    return bfill.position();
}

static void http_loop()
{
    word plen    = ether.packetReceive();
    word request = ether.packetLoop(plen);

    if (request)
    {
        word response = build_http_response(request);
        ether.httpServerReply(response);
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
    http_loop();
}

// vim: ft=arduino
