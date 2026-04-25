/*I will code using C or C++.

I want to modify https://github.com/dl9rdz/rdz_ttgo_sonde/blob/dev2/RX_FSK/src/MP3H.cpp to receive CATS radio format packets.
I have begun by using the MP3 section of code and converting to CA1.
Output of this code block should integrate to the original output formats that being the si structure.
Ask me for clarification as needed.
The include libcats can be found at https://github.com/CamK06/libCATS/tree/569dc26e5177e13aad808d60b7c5bef09eda550d
The converted code now consists of this:
*/
++
/* CA1 decoder functions */

#include <Arduino.h>
#include <vector>
#include <string>
#include <cstring>
#include <stdint.h>
#include <stdio.h>
#include "CA1.h"
#include "sx1278FSK.h"
#include "rsc.h"
#include "Sonde.h"
#include "decoder.h"
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
#define CA1_DBG(x)
#else
#define CA1_DBG(x)
#endif

extern "C" {
#include "libcats/libcats.h"
}

static struct st_CA1state {
	uint32_t id1, id2;
	uint8_t idok;
	uint32_t gpsdate;
	uint32_t gpsdatetime;
	bool dateok;
} ca1state;


static int haveNewFrame = 0;
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

#define CA1_MAX_FRAME 8191
#define CA1_MIN_FRAME 64
#define FRAME_TIMEOUT_MS 20

static uint8_t frameBuf[CA1_MAX_FRAME];
static uint16_t framePos = 0;

static bool inFrame = false;
static uint8_t syncIndex = 0;
static uint32_t lastByteTime = 0;

static const uint8_t SYNC_WORD[] = {0xAB, 0xCD, 0xEF, 0x12};


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

	// Packet config 1: fixed len, no manchester, no crc, no address filter
	// Packet config 2: packet mode, no home ctrl, no beackn, msb(packetlen)=0)

	if(sx1278.setPacketConfig(0x08, 0x40)!=0) {
		CA1_DBG(Serial.println("Setting Packet config FAILED"));
		return 1;
	}

        // enable RX
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
// But whiskers are 256 max so this could be 256. ??


// CATS has CRC included in library Don't need here.

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


static uint8_t hex(uint32_t n) {
	n = n % 16;
	return (n<10) ? (n+'0') : (n-10+'A');
}


// CATS lib should replace this next block with
int cats_packet_decode(cats_packet_t* pkt, uint8_t* buf, size_t buf_len)
   // Includes Deinterleave, LDPC decode, Dewhiten, Call to semi-decode

int cats_whisker_decode(const uint8_t* data, cats_whisker_t* out)
   // Includes CRC-Check, whisker-decode, packet_add_whisker



/* ______________CATS processsing code ______________________ */ 
// CATS may need declarations, structures, types, etc
// need to match library data structure to rdz naming of data
// After this portion runs data should be mapped into the above CA1 code
// I don't understand this and it causes errors. I think data should be passed from receive and not read.
// Trying to mirror CATS example.


static void parse_whiskers(cats_packet_t *pkt, SondeData *si)
{
    uint8_t *p = pkt->payload;
    size_t len = pkt->payload_len;

    while (len >= 2)
    {
        uint8_t type = p[0];
        uint8_t wlen = p[1];
        uint8_t *data = &p[2];

        if (len < (size_t)(2 + wlen))
            break;

        switch (type)
        {
            // -------------------------
            // TIMESTAMP (example type)
            // -------------------------
            case 0x01:
                if (wlen >= 4)
                {
                    si->timestamp =
                        (uint32_t)data[0] |
                        (uint32_t)data[1] << 8 |
                        (uint32_t)data[2] << 16 |
                        (uint32_t)data[3] << 24;
                }
                break;

            // -------------------------
            // LATITUDE / LONGITUDE
            // -------------------------
            case 0x02:
                if (wlen >= 8)
                {
                    int32_t lat = (int32_t)(
                        data[0] |
                        data[1] << 8 |
                        data[2] << 16 |
                        data[3] << 24);

                    int32_t lon = (int32_t)(
                        data[4] |
                        data[5] << 8 |
                        data[6] << 16 |
                        data[7] << 24);

                    si->latitude  = lat / 1e7;
                    si->longitude = lon / 1e7;
                }
                break;

            // -------------------------
            // ALTITUDE
            // -------------------------
            case 0x03:
                if (wlen >= 4)
                {
                    int32_t alt =
                        (int32_t)(
                            data[0] |
                            data[1] << 8 |
                            data[2] << 16 |
                            data[3] << 24);

                    si->altitude = alt;
                }
                break;

            default:
                break;
        }

        p += 2 + wlen;
        len -= 2 + wlen;
    }
}




int decode_cats_packet(uint8_t *buf, size_t len) {
    float rssi;

    // STEP 1: PHY decode
    if (cats_radio_iface_decode(buf, len, &rssi) != 0) {
        Serial.println("Radio decode failed");
        return -1;
    }

    // STEP 2: Packet parse
    cats_packet_t* pkt;
    cats_packet_prepare(&pkt);

    if (!cats_packet_from_buf(pkt, buf, len)) {
        Serial.println("Packet parse failed");
        cats_packet_free(pkt);
        return -1;
    }

    // STEP 3: Extract identification
    SondeData *si = &(sonde.si()->d);

    char callsign[64];
    uint8_t ssid;
    uint16_t icon;

    cats_packet_get_identification(pkt, callsign, &ssid, &icon);
    snprintf(si->id, 10, "%s-%d", callsign, ssid);
    si->validID = true;

    // STEP 4: TODO → whiskers (still missing!)

    cats_packet_free(pkt);
    return 0;
}
// block above is added 

int CA1::receive()
{
    static uint32_t lastFrame = 0;
    uint8_t retval = RX_TIMEOUT;

    unsigned long t0 = millis();
    Serial.printf("CA1::receive() start at %ld\n", t0);

    while (millis() - t0 < 2000)
    {
        uint8_t irq = sx1278.readRegister(REG_IRQ_FLAGS2);

        if (bitRead(irq, 7)) Serial.println("FIFO full");
        if (bitRead(irq, 4)) Serial.println("FIFO overflow");
        if (bitRead(irq, 2)) sx1278.clearIRQFlags();

        // -------------------------
        // READ BYTE FROM FIFO
        // -------------------------
        if (bitRead(irq, 6) == 0)
        {
            uint8_t data = sx1278.readRegister(REG_FIFO);
            lastByteTime = millis();

            // DEBUG raw stream
            Serial.printf("%02X ", data);

            // -------------------------
            // SYNC STATE MACHINE
            // -------------------------
            if (!inFrame)
            {
                if (data == SYNC_WORD[syncIndex])
                {
                    syncIndex++;
                    if (syncIndex == sizeof(SYNC_WORD))
                    {
                        inFrame = true;
                        framePos = 0;

                        memcpy(frameBuf, SYNC_WORD, sizeof(SYNC_WORD));
                        framePos = sizeof(SYNC_WORD);

                        syncIndex = 0;
                        Serial.println("\nSYNC detected");
                    }
                }
                else
                {
                    syncIndex = 0;
                }
            }
            else
            {
                // -------------------------
                // BUFFER FRAME
                // -------------------------
                if (framePos < CA1_MAX_FRAME)
                {
                    frameBuf[framePos++] = data;
                }
                else
                {
                    Serial.println("Frame overflow");
                    inFrame = false;
                    framePos = 0;
                    retval = RX_ERROR;
                }

                // -------------------------
                // TIMEOUT END OF FRAME
                // -------------------------
                if (millis() - lastByteTime > FRAME_TIMEOUT_MS)
                {
                    Serial.println("\nFrame timeout → processing");
                    inFrame = false;
                    haveNewFrame = 1;
                }
            }
        }
        else
        {
            delay(2);
        }

        // -------------------------
        // PROCESS COMPLETE FRAME
        // -------------------------
        if (haveNewFrame)
        {
            haveNewFrame = 0;

            Serial.printf("Full frame received (%d bytes)\n", framePos);

            // -------------------------
            // STEP 1: PHY DECODE
            // -------------------------
            float rssi = 0;
            if (cats_radio_iface_decode(frameBuf, framePos, &rssi) != 0)
            {
                Serial.println("Radio decode failed");
                framePos = 0;
                continue;
            }

            Serial.printf("CATS decode OK (len=%d RSSI=%.1f)\n", framePos, rssi);

            // -------------------------
            // STEP 2: PACKET PARSE
            // -------------------------
            cats_packet_t pkt;
            cats_packet_prepare(&pkt);

            if (!cats_packet_from_buf(&pkt, frameBuf, framePos))
            {
                Serial.println("Packet parse failed");
                framePos = 0;
                continue;
            }

            // -------------------------
            // STEP 3: IDENTIFICATION
            // -------------------------
            SondeData *si = &(sonde.si()->d);

            char callsign[64] = {0};
            uint8_t ssid = 0;
            uint16_t icon = 0;

            cats_packet_get_identification(&pkt, callsign, &ssid, &icon);

            if (callsign[0] != '\0')
            {
                snprintf(si->id, sizeof(si->id), "%s-%d", callsign, ssid);
                si->validID = true;
                Serial.printf("ID: %s\n", si->id);
            }
            else
            {
                Serial.println("No ID whisker present");
            }

            // -------------------------
            // TODO: WHISKERS (GPS ETC)
            // -------------------------

            // -------------------------
            // RESET FRAME STATE
            // -------------------------
            framePos = 0;
            inFrame = false;

            return RX_OK;
        }
    }

    int32_t afc = sx1278.getAFC();
    int16_t rssi = sx1278.getRSSI();

    Serial.printf("AFC=%d RSSI=%.1f\n", afc, rssi / 2.0);
    Serial.println("CA1::receive() timed out");

    return retval;
}

if(haveNewFrame) {


      			
    #if 0Serial.printf("Full frame received (%d bytes)\n", framePos);

    // DEBUG dump (optional)
    for (int i = 0; i < framePos; i++) {
        Serial.printf("%02X ", frameBuf[i]);
    }
    Serial.println();

    // --- PARSE PACKET ---
    cats_packet_t pkt;
    cats_packet_prepare(&pkt);

    if (!cats_packet_from_buf(&pkt, frameBuf, framePos)) {
        Serial.println("Packet parse failed");
        haveNewFrame = 0;
        framePos = 0;
        return RX_ERROR;
    }

// --- EXTRACT IDENTIFICATION ---
SondeData *si = &(sonde.si()->d);

char callsign[64] = {0};
uint8_t ssid = 0;
uint16_t icon = 0;

cats_packet_get_identification(&pkt, callsign, &ssid, &icon);

if (callsign[0] != '\0') {
    snprintf(si->id, sizeof(si->id), "%s-%d", callsign, ssid);
    si->validID = true;
}
Serial.printf("ID: %s\n", si->id);

Serial.printf("Full frame received (%d bytes)\n", framePos);

// DEBUG dump
for (int i = 0; i < framePos; i++) {
    Serial.printf("%02X ", frameBuf[i]);
}
Serial.println();

// --- PARSE PACKET ---
cats_packet_t pkt;
cats_packet_prepare(&pkt);

if (!cats_packet_from_buf(&pkt, frameBuf, framePos)) {
    Serial.println("Packet parse failed");
    haveNewFrame = 0;
    framePos = 0;
    return RX_ERROR;
}

// --- EXTRACT IDENTIFICATION ---
SondeData *si = &(sonde.si()->d);

char callsign[64] = {0};
uint8_t ssid = 0;
uint16_t icon = 0;

cats_packet_get_identification(&pkt, callsign, &ssid, &icon);

// SAFE CHECK
if (callsign[0] != '\0') {
    snprintf(si->id, sizeof(si->id), "%s-%d", callsign, ssid);
    si->validID = true;
    Serial.printf("ID: %s\n", si->id);
} else {
    Serial.println("No ID whisker present");
}


haveNewFrame = 0;
framePos = 0;

return RX_OK;
haveNewFrame = 0;
framePos = 0;

return RX_OK;
				if(nFrames <= 0) {
					// up to 6 old or erronous frames received => break out
					Serial.printf("nFrames is %di, giving up\n", nFrames);
					break;
				}
#endif
			}
			delay(2);
    		}
    	}
        int32_t afc = sx1278.getAFC();
        int16_t rssi = sx1278.getRSSI();
        Serial.printf("receive: AFC is %d, RSSI is %.1f\n", afc, rssi/2.0);
	Serial.printf("CA1::receive() timed out\n");
    	return retval;
}

int CA1::waitRXcomplete() 
{

	return 0;
}


CA1 ca1 = CA1();
	




