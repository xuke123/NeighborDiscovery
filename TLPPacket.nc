#include "TLPMessage.h"
interface TLPPacket {
      command void* getPayload(tlp_message_t* tlp, uint8_t len); 
      command void* detachPayload(void* tlp, uint8_t* len, uint8_t type);
      command void clear(tlp_message_t* tlp, uint8_t len);
}