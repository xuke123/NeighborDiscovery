#include "AM.h"

generic configuration NetworkhubC(am_id_t AMId) {
    
}

implementation {
    components NetworkDispatherC;

    components new AMReceiverC(AMId) as AMReceiver;
    components new AMReceiverC(AMId) as AMSnoop;
    AMReceiver.Receive->NetworkDispatherC;
    
}