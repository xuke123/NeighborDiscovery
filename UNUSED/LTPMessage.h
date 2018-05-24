#ifndef LTP_MESSAGE_H
#define LTP_MESSAGE_H

#define LTP_MAX_LENGTH  50

//实际上是TLP
typedef nx_struct ltp_message_t {
  nx_uint8_t type;
  nx_uint8_t length;
  nx_uint8_t pload[LTP_MAX_LENGTH];
} ltp_message_t;

#endif
