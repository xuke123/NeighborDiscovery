#include "AM.h"
configuration NetBeaconPiggybackC{
   provides interface NetBeaconPiggyback;
   provides interface TLPPacket;
}
implementation{
   components NetBeaconPiggybackP;

   NetBeaconPiggyback=NetBeaconPiggybackP;
   TLPPacket=NetBeaconPiggybackP;
  // components new AMReceiverC(2) as Receiver;
   
 //  NetBeaconPiggybackP.Receive->Receiver;

   components MacC;
   NetBeaconPiggybackP.BeaconPiggyBack->MacC;
   NetBeaconPiggybackP.AsyncReceive->MacC.BeaconSnoop; 
}