/**
 * @file net.h
 * @brief Network / UDP communication with AgOpenGPS / AgIO.
 *
 * Handles sending GPS and Steer status frames, and receiving
 * AgIO commands (hello, scan, subnet change, steer data).
 */

#pragma once

#include "aog_udp_protocol.h"

/// Initialise network (W5500 Ethernet via HAL).
void netInit(void);

/// Poll for received UDP frames and process them.
/// Should be called frequently from commTask.
void netPollReceive(void);

/// Send periodic AOG frames (GPS main + steer status).
/// Should be called at ~10 Hz from commTask.
void netSendAogFrames(void);

/// Internal: process a single decoded frame.
void netProcessFrame(uint8_t src, uint8_t pgn,
                     const uint8_t* payload, size_t payload_len);
