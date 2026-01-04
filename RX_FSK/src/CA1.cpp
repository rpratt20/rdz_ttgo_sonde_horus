/* CA1 decoder functions */

#include "CA1.h"
#include "SX1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include <SPIFFS.h>

#include "../libCATS/include/cats/interleaver.h"
#include "../libCATS/include/cats/ldpc.h"
#include "../libCATS/include/cats/ldpc_matrices.h"
#include "../libCATS/include/cats/whitener.h"
#include "../libCATS/include/cats/radio_iface.h"
#include "../libCATS/include/cats/whisker.h"
#include "../libCATS/include/cats/packet.h"
#include "../libCATS/include/cats/util.h"
#include "../libCATS/include/cats/error.h"

#define CA1_DEBUG 1

#if CA1_DEBUG
#define CA1_DBG(x) x
#else
#define CA1_DBG(x)
#endif

static struct st_CA1state {
	uint32_t id1, id2;
	uint8_t idok;
	uint32_t gpsdate;
	uint32_t gpsdatetime;
	bool dateok;
} ca1state;

static byte data[8192];
static byte *dataptr=data;

static uint8_t rxbitc;
static uint16_t rxbyte;
static int rxp=0;
static int haveNewFrame = 0;
//static int lastFrame = 0;
static int headerDetected = 0;

extern uint16_t MON[];

decoderSetupCfg CA1SetupCfg = {
	.bitrate = 9600,
	.rx_cfg = 0x00,
	.sync_cfg = 0x70,
	.sync_len = 4,
	.sync_data = (const uint8_t *)"\xAB\xCD\xEF\x12",
	.preamble_cfg = 0x55 | 0x55 | 0x55 | 0x55
};

int CA1::setup(float frequency, int /*type*/) 
{
	CA1_DBG(Serial.println("Setup sx1278 for CA1 sonde"));;
	if(sx1278.ON()!=0) {
		CA1_DBG(Serial.println("Setting SX1278 power on FAILED"));
		return 1;
	}
	// setFSK: switches to FSK standby mode
	if(sx1278.setFSK()!=0) {
		CA1_DBG(Serial.println("Setting FSK mode FAILED"));
		return 1;
	}
        Serial.print("CA1: setting RX frequency to ");
        Serial.println(frequency);
        int res = sx1278.setFrequency(frequency);
	// Test: maybe fix issue after spectrum display?
	sx1278.writeRegister(REG_PLL_HOP, 0);

        if(sx1278.setAFCBandwidth(sonde.config.ca1.agcbw)!=0) {
                CA1_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.ca1.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.ca1.rxbw)!=0) {
                CA1_DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.ca1.rxbw));
                return 1;
        }

//// Step 2: Real reception
	if(DecoderBase::setup(CA1SetupCfg, sonde.config.ca1.agcbw, sonde.config.ca1.rxbw)!=0) {
		return 1;
	}
# if 0
	// Now all done in Decoderbase
	// FSK standby mode, seems like otherweise baudrate cannot be changed?
	sx1278.setFSK();
	if(sx1278.setBitrate(9600)!=0) {
		CA1_DBG(Serial.println("Setting bitrate 9600bit/s FAILED"));
		return 1;
	}
	CA_DBG(Serial.printf("Exact bitrate is %f\n", sx1278.getBitrate()));
	// Probably not necessary, as this was set before
        if(sx1278.setAFCBandwidth(sonde.config.CA1.agcbw)!=0) {
               CA1_DBG(Serial.printf("Setting AFC bandwidth %d Hz FAILED", sonde.config.CA1.agcbw));
                return 1;
        }
        if(sx1278.setRxBandwidth(sonde.config.CA1.rxbw)!=0) {
                CA1
            _DBG(Serial.printf("Setting RX bandwidth to %d Hz FAILED", sonde.config.CA1
            .rxbw));
                return 1;
        }

	///// Enable auto-AFC, auto-AGC, RX Trigger by preamble
	//if(sx1278.setRxConf(0x1E)!=0) {
	// Disable auto-AFC, auto-AGC, RX Trigger by preamble
	if(sx1278.setRxConf(0x00)!=0) {
		CA1_DBG(Serial.println("Setting RX Config FAILED"));
		return 1;
	}
	// version 1, working with continuous RX
	const char *SYNC="\xAB\xCD\xEF\x12";
	if(sx1278.setSyncConf(0x70, 4, (const uint8_t *)SYNC)!=0) {
		CA1_DBG(Serial.println("Setting SYNC Config FAILED"));
		return 1;
	}
        // Preamble detection off (+ size 1 byte, maximum tolerance; should not matter for "off"?)
        if(sx1278.setPreambleDetect(0x55 | 0x55 | 0x55 | 0x55)!=0) {
		CA1_DBG(Serial.println("Setting PreambleDetect FAILED"));
		return 1;
	}
#endif

	// Packet config 1: fixed len, no mancecer, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)
	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		CA1_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

        // enable RX cats 8192 max packet length
        sx1278.setPayloadLength(8191);  // mp3h infinite for now used 0 for this
        //sx1278.setRxConf(0x20);
	uint16_t afc = sx1278.getRawAFC();
        sx1278.writeRegister(REG_OP_MODE, FSK_RX_MODE);
	delay(50);
	sx1278.setRawAFC(afc);
	delay(50);
        Serial.printf("after RX_MODE: AFC is %d\n", sx1278.getAFC());

	memset((void *)&ca1state, 0, sizeof(ca1state));
#if CA1_DEBUG
	CA1_DBG(Serial.println("Setting SX1278 config for CA1 finished\n"); Serial.println());
#endif
        return res;
}

/* above here should be set for CATS */
//__________________________________________________________________________________

CA1::CA1() {
}

// This needs change BECAUSE offsets need remap. I think max packet length should be 8191 bytes.
// But whiskers are 256 max so this could be 256 but 49 must be incorrect?
#define CA1_FRAMELEN 8192


/* This is partial remap to gps whisker

#define OFS -3                       // 3 bytes for 0x02 type and 14 length definition
#define pos_GPSecefX        (OFS+ 2)  //   4 byte latitude
#define pos_GPSecefY        (OFS+ 6)  //   4 byte longitude
#define pos_GPSecefZ        (OFS+10)  //   2 byte altitude
#define pos_        (OFS+12)  //   1 byte location error
#define pos_        (OFS+13)  //   1 byte heading
#define pos_        (OFS+14)  //   2 byte speed
*/

void CA1::printRaw(uint8_t *data, int len)
{
	char buf[3];
	int i;
	for(i=0; i<len; i++) {
		snprintf(buf, 3, "%02X ", data[i]);
		Serial.print(buf);
	}
	Serial.println();
}

#ifndef PI
#define  PI  (3.1415926535897932384626433832795)
#endif
#define RAD (PI/180)
#define DEG (180/PI)

static uint32_t u4(uint8_t *d)
{
	return d[0] | (d[1]<<8) | (d[2]<<16) | (d[3]<<24);
}
#define i4(d) ((int32_t)u4(d))

static uint16_t u2(uint8_t *d)
{
	return d[0] | (d[1]<<8);
}
#define i2(d) ((int16_t)u2(d))


// defined in RS41.cpp
extern void wgs84r(double x, double y, double z, double * lat, double * long0, double * heig);
extern double atang2(double x, double y);

// need gps whisker mapped to *buf

static void resetca1() {
	ca1state.id1 = ca1state.id2 = 0;
	ca1state.idok = 0;
	ca1state.gpsdate = 0;
	ca1state.dateok = 0;
}

// CATS lib should replace this next block with
int cats_packet_decode(cats_packet_t* pkt, uint8_t* buf, size_t buf_len);
   // Includes Deinterleave, LDPC decode, Dewhiten, Call to semi-decode

int cats_whisker_decode(const uint8_t* data, cats_whisker_t* out);
   // Includes CRC-Check, whisker-decode, packet_add_whisker
	
	// data is a frame with correct CRC
	// SondeInfo *si = sonde.si();
    // Can this work or do I need CATS CRC computation? I believe we need CATS CRC instead

	SondeData *si =&(sonde.si()->d);
	

/* ______________CATS processsing code ______________________ */ 
// CATS may need declarations, structures, types, etc
// need to match library data structure to rdz naming of data
// After this portion runs data should be mapped into the above CA1 code
// I don't understand this and it causes errors. I think data should be passed from receiveand not read.
// Trying to mirror CATS example.



// END OF ORIGINAL RECEIVE
/*
// When received proceed with these steps
int cats_packet_decode(cats_packet_t* pkt, uint8_t* buf, size_t buf_len);
    // includes these: interleave, ldpc, dewhiten

int cats_packet_semi_decode(cats_packet_t* pkt, uint8_t* buf, size_t buf_len);
    // includes these: CRC, whiskers
	// What is first decode or semi_decode?

int cats_whisker_decode(const uint8_t* data, cats_whisker_t* out);
    // From here transfer data into ttgo structures 
*/
int CA1::waitRXcomplete() 
    {

	return 0;
    }


CA1 ca1 = CA1();
	

// Match of si sonde_data structure to cats packets:
// struct st_sonde_data{
//    char id[10] = ID whisker byte 4 +
//    char ser[12] = 
//    Bool validID = 
//    char typestr[5] = 
//    int8_t typestr[5] = 
//    POSITION fields from GPS:
//    float lat = gps whisker byte 2-5
//    float long = gps whisker byte 6-9
//    float alt = node info whisker data index[9] bytes 18-21
//    float vs = can be included in comment 
//    float hs = gps whisker byte 14
//    float dir = gps whisker byte 13
//    uint8_t sats = 
//    uint8_t validPos = 
//    uint32_ t time = timrstamp whisker index[2] bytes 3-7
//    uint32_t frame = 
//    Bool validTime = 
//    float batteryVoltage = node info whisker data index[9] byte 16
//  
// }
