#include "LTPMessage.h"
interface LTPPacket{
      command void* getPayload(ltp_message_t*payload,uint8_t len); 
      command void* detachPayload(void * payload, uint8_t* len,uint8_t type);
      command void  clear(ltp_message_t*payload,uint8_t len);
}