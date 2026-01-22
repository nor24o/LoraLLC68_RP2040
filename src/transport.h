// transport.h
#include <Arduino.h>

// Your Network Password
const uint8_t MY_NET_ID = 0xAA; 

// The Routing Header
struct __attribute__((packed)) TransportHeader {
  uint8_t netId;      // Network ID (Security)
  uint8_t senderId;   // Who sent this
  uint8_t targetId;   // Who needs to read this
  uint8_t packetId;   // Sequence counter
};