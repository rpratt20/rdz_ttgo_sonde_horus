/*
 * CA1.h
 * Functions for decoding CA1 radiosonde
 * Copyright (C) 2021 Hansi Reiser, dl9rdz
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef CA1_h
#define CA1_h

#include <stdlib.h>
#include <stdint.h>
#include <Arduino.h>
#ifndef inttypes_h
        #include <inttypes.h>
#endif
#include "DecoderBase.h"

/* Main class */
class CA1 : public DecoderBase
{
private:
	void printRaw(uint8_t *data, int len);
	void processHV2data(uint8_t data);
        int decodeframeCA1(uint8_t *data);
public:
	HV2();
	int setup(float frequency, int type = 0);
	int receive();
	int waitRXcomplete();
};

extern CA1 CA1;

#endif

