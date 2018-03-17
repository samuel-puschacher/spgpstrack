#define TXdist 100
//#define DOUBLE_SEND

#define CFG_eu868
#define CONFIRMED 0

//#define SOFT_SERIAL
#ifndef SOFT_SERIAL
  //#define DEBUG
#endif
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <TinyGPS++.h>
#ifdef SOFT_SERIAL
#include <SoftwareSerial.h>
#else
#ifdef DEBUG
#include <SoftwareSerial.h>
#endif
#endif


//#include <CayenneLPP.h>

#define TXdist 100
#define DOUBLE_SEND

#define CFG_eu868
#define CONFIRMED 1
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
#ifdef SOFT_SERIAL
SoftwareSerial ss(RXPin, TXPin);
#else
  #ifdef DEBUG
    SoftwareSerial ds(A3, A2);
  #endif
#endif
uint8_t txBuffer[9];

//CayenneLPP lpp(20);

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the early prototype TTN
// network.
static const u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

// LoRaWAN end-device address (DevAddr)
static const u4_t DEVADDR = 0x26000000 ; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120;

// Pin mapping Dragino Shield
boolean lock = false;
const int buzzer = 5; //buzzer to arduino pin 5
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};


void onEvent (ev_t ev) {
    print(String(os_getTime()));
    print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            lock = false;
            if (LMIC.txrxFlags & TXRX_ACK)
              println(F("Received ack"));
              tone(buzzer, 3000); // Send 1KHz sound signal...
              delay(500);        // ...for 1 sec
              noTone(buzzer);
            if (LMIC.dataLen) {
              println(F("Received "));
              println(String(LMIC.dataLen));
              println(F(" bytes of payload"));
            }
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            println(F("EV_LINK_ALIVE"));
            break;
         default:
            println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        println(F("OP_TXRXPEND, not sending"));
    } else {
      lock = true;
        // Prepare upstream data transmission at the next possible time.
        //LMIC_setTxData2(3, lpp.getBuffer(), lpp.getSize(), confirmed);
        LMIC_setTxData2(3, txBuffer, sizeof(txBuffer), CONFIRMED);
        println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}
void println(String s)
{
  #ifdef SOFT_SERIAL
  Serial.println(s);
  #else
    #ifdef DEBUG
      ds.println(s);
    #endif
  #endif
}
void print(String s)
{
  #ifdef SOFT_SERIAL
  Serial.print(s);
  #else 
    #ifdef DEBUG
      ds.print(s);
    #endif
  #endif
}
void setup() {
    #ifdef SOFT_SERIAL
    Serial.begin(115200);
    ss.begin(GPSBaud);
    #else
    Serial.begin(GPSBaud);
      #ifdef DEBUG
        ds.begin(115200);
      #endif
    #endif
    println(F("Starting"));
    pinMode(buzzer, OUTPUT);
    
    print(F("TinyGPS++ library v. ")); println(TinyGPSPlus::libraryVersion());

    
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF8,14);

    // Start job
    //do_send(&sendjob);
}

void loop() {
    
    os_runloop_once();
    if(!lock)smartDelay(70);
    static double LAST_TX_LAT = 0, LAST_TX_LON = 0;
    static float LAST_COURSE = -1;
    static boolean doDouble = false;
    //printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
    //printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
    //println(String(gps.sentencesWithFix()));
    if(gps.location.isValid() && !lock){
      //Serial.println(String(gps.sentencesWithFix()));
       unsigned long lastTxDist =
          (unsigned long)TinyGPSPlus::distanceBetween(
            gps.location.lat(),
            gps.location.lng(),
            LAST_TX_LAT, 
            LAST_TX_LON);
          println("Last TX Distance " + String(lastTxDist) + "m");
      
        double lastTxCourse =
          TinyGPSPlus::courseTo(
            gps.location.lat(),
            gps.location.lng(),
            LAST_TX_LAT, 
            LAST_TX_LON);
      
        //println( "Course: " + String(lastTxCourse));
     
        const char *cardinalLastTx = TinyGPSPlus::cardinal(lastTxCourse);
      
        //println("Cardinal " + String(cardinalLastTx));
        if(lastTxDist > TXdist ){
         #ifdef DOUBLE_SEND
          doDouble!=doDouble;
         #endif
         /*lpp.reset();
         lpp.addGPS(0, float(gps.location.lat()), float(gps.location.lng()), gps.altitude.meters());
         lpp.addDigitalInput(1, uint8_t(lastTxDist));
         */
          
          // variables for GPS data we want to transmit
          uint8_t hdop;
          uint16_t alt;
          uint32_t flat, flon;
          flat = ((gps.location.lat() + 90) / 180) * 16777215;
          flon = ((gps.location.lng() + 180) / 360) * 16777215;
          txBuffer[0] = ( flat >> 16 ) & 0xFF;
          txBuffer[1] = ( flat >> 8 ) & 0xFF;
          txBuffer[2] = flat & 0xFF;
      
          txBuffer[3] = ( flon >> 16 ) & 0xFF;
          txBuffer[4] = ( flon >> 8 ) & 0xFF;
          txBuffer[5] = flon & 0xFF;
        
          alt = gps.altitude.meters();
          txBuffer[6] = ( alt >> 8 ) & 0xFF;
          txBuffer[7] = alt & 0xFF;
        
          hdop = gps.hdop.hdop()/10;
          txBuffer[8] = hdop & 0xFF;
         
         LAST_TX_LAT = gps.location.lat();
         LAST_TX_LON = gps.location.lng();
         do_send(&sendjob); 
        }
            
    }
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
     #ifdef SOFT_SERIAL
      while (ss.available())
     #else
      while (Serial.available())
     #endif
    {

    #ifdef SOFT_SERIAL
      gps.encode(ss.read());
     #else
      gps.encode(Serial.read());
      //ds.println("a " + Serial.read());
     #endif
    }
      //Serial.println(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      print("*");
      print(" ");
  }
  else
  {
    #ifdef SOFT_SERIAL
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
    #endif
  }
  smartDelay(0);
}