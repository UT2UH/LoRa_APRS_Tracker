/* Copyright (C) 2025 Ricardo Guzman - CA2RXU
 * 
 * This file is part of LoRa APRS Tracker.
 * 
 * LoRa APRS Tracker is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or 
 * (at your option) any later version.
 * 
 * LoRa APRS Tracker is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with LoRa APRS Tracker. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LORA_UTILS_H_
#define LORA_UTILS_H_

#include <Arduino.h>

#define MAX_BLOCKSIZE 256

struct CryptoKey {
    uint8_t bytes[32];
    /// # of bytes, or -1 to mean "invalid key - do not use"
    int8_t length;
};

struct ReceivedLoRaPacket {
    String  text;
    int     rssi;
    float   snr;
    int     freqError;
};

namespace LoRa_Utils {

    void setFlag();
    void changeFreq();
    void setup();
    void sendNewPacket(const String& newPacket);
    void wakeRadio();
    ReceivedLoRaPacket receiveFromSleep();
    ReceivedLoRaPacket receivePacket();
    void sleepRadio();
    void    setKey                      (const CryptoKey &k);
    void    getBeaconKey                ();
    void    _encrypt                    (uint32_t timeNonce, uint8_t *bytes, size_t numBytes);
    void    _decrypt                    (uint32_t timeNonce, uint8_t *bytes, size_t numBytes);

}

#endif