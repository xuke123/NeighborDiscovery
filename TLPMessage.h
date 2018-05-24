#ifndef TLP_MESSAGE_H
#define TLP_MESSAGE_H

#define TLP_MAX_LENGTH 1000

//实际上是TLP
typedef nx_struct tlp_message_t {
  nx_uint8_t type;
  nx_uint8_t length;
  nx_uint8_t payload[TLP_MAX_LENGTH];
} tlp_message_t;

#endif