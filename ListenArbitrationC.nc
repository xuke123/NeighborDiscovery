#include "AM.h"
configuration ListenArbitrationC{
	provides interface ListenRemote;
}
implementation{
    //components ListenArbitrationP,BeaconListenerC;
    components RadioArbiterC;
    ListenRemote = RadioArbiterC;
    //ListenArbitrationP.subListernRemote->BeaconListenerC;
}