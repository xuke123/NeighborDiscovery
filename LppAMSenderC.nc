#include "AM.h"

generic configuration LppAMSenderC(am_id_t AMId)
{
  provides {
    interface AMSend;
    interface Packet;
    interface AMPacket;
    interface PacketAcknowledgements as Acks;
  }
}

implementation
{
  components new DirectAMSenderC(AMId);
  components new LppAMSenderP();
  components ActiveMessageC;
 // components SystemLowPowerListeningC;
  components WakeupTimeC;
  components PacketSupC;

  AMSend = LppAMSenderP;
  Packet = DirectAMSenderC;
  AMPacket = DirectAMSenderC;
  Acks = DirectAMSenderC;

  LppAMSenderP.SubAMSend -> DirectAMSenderC;
  LppAMSenderP.WakeupTime-> WakeupTimeC;
  LppAMSenderP.PacketSup-> PacketSupC;
}