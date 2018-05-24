#include "AM.h"
#include "TLPMessage.h"

module NetBeaconPiggybackP {
    provides interface NetBeaconPiggyback;
    provides interface TLPPacket;
    uses{
        interface  AsyncReceive;
        interface BeaconPiggyBack;
    }
}

implementation {
      // event message_t* Receive.receive(message_t* msg, void* payload, uint8_t length) {

       async event void  AsyncReceive.receive(message_t * msg, void * payload, uint8_t len) {
           signal NetBeaconPiggyback.receive(msg, payload, len);
       }

       command uint8_t NetBeaconPiggyback.set(tlp_message_t* tlp, uint16_t len, uint8_t type) {
           tlp->length = len+2;
           tlp->type = type;
           return call BeaconPiggyBack.piggyBack((uint8_t*)tlp, len+2);
       }

       command void* TLPPacket.getPayload(tlp_message_t* tlp, uint8_t len) {
           if (len <= TLP_MAX_LENGTH)
              return tlp->payload;
           else
             return NULL;
       }

       command void TLPPacket.clear(tlp_message_t* tlp, uint8_t len) {
           memset(tlp, 0x0, len);
       }

       command void* TLPPacket.detachPayload(void* tlp, uint8_t* len, uint8_t type) {
            uint8_t* tlp_ = (uint8_t *)tlp;
            uint8_t slen = 0;
            while (*tlp_ != 0) {
	        //*tlp是type,*(tlp+1)是length,*(tlp+2)是payload
                if (*tlp_ == type) {
                    *len = *(tlp_+1)-2;
                    return tlp_ + 2;
                }
                //当前的tlp不是要查找的话,换到下一个tlp,指针往前移动len个字节
                slen = *(tlp_ + 1);
                tlp_ += slen;  
            }

            return NULL;
       }

}
