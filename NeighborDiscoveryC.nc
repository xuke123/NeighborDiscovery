#include "NeighborDiscovery.h"

configuration NeighborDiscoveryC {
   provides {
       interface LinkEstimate;
   }
   
}
implementation {
    components MainC;
    NeighborDiscoveryP.Boot -> MainC;

    components NeighborDiscoveryP;
    LinkEstimate = NeighborDiscoveryP;

    components WakeupTimeC;
    NeighborDiscoveryP.WakeupTime->WakeupTimeC;
    NeighborDiscoveryP.NeighborParameterManage->WakeupTimeC;

    components ListenArbitrationC;
    NeighborDiscoveryP.ListenRemote->ListenArbitrationC;

    components new AMSenderC(NDP_ID) as Sender;
    NeighborDiscoveryP.Send->Sender;
    
    components new AMReceiverC(NDP_ID) as Receiver;
    NeighborDiscoveryP.Receive->Receiver; 

    components NetBeaconPiggybackC;
    NeighborDiscoveryP.NetBeaconPiggyback->NetBeaconPiggybackC;
    NeighborDiscoveryP.TLPPacket->NetBeaconPiggybackC;
    
    components new TimerMilliC() as CheckActTimer,
               new TimerMilliC() as SampleTimer,
               new TimerMilliC() as ListenCtlTimer;
    NeighborDiscoveryP.CheckTimer->CheckActTimer;
    NeighborDiscoveryP.ListenTimer->ListenCtlTimer;
    NeighborDiscoveryP.SampleTimer->SampleTimer;

    components LocalTimeMilliC as LocalTimeC;
    NeighborDiscoveryP.LocalTime->LocalTimeC;

    components ActiveMessageC;
    NeighborDiscoveryP.Packet->ActiveMessageC;
    NeighborDiscoveryP.AMPacket->ActiveMessageC;

    components CC2420PacketC;
    NeighborDiscoveryP.PacketTimeStampMilli->CC2420PacketC;
    NeighborDiscoveryP.PacketTimeSyncOffset->CC2420PacketC;
    NeighborDiscoveryP.CC2420Packet->CC2420PacketC;
    NeighborDiscoveryP.CC2420PacketBody->CC2420PacketC;

    components PacketSupC;
    NeighborDiscoveryP.PacketSup->PacketSupC;

    components MacC;
    NeighborDiscoveryP.SplitControl->MacC;

    components MacControlC;
    NeighborDiscoveryP.LocalWakeup->MacControlC;
    NeighborDiscoveryP.ReceiveMode->MacControlC;

    components RandomC; 
    NeighborDiscoveryP.Random -> RandomC;

    //components BeaconListenerC;
    //NeighborDiscoveryP.BeaconSnoop->BeaconListenerC;

    components SerialPrintfC;

    //components RadioArbiterC;
    //NeighborDiscoveryP.RadioState->RadioArbiterC;
    //components AdapterC;

    //NeighborDiscoveryP.PacketTimeStampMilli->ActiveMessageC;
    //NeighborDiscoveryP.PacketTimeSyncOffset->AdapterC;

    
   // NeighborDiscoveryP.TossimPacketBody -> AdapterC;

    //components MacControlC;
    //NeighborDiscoveryP.SplitControl->MacControlC;
    //NeighborDiscoveryP.SplitControl->ActiveMessageC;
        

    // components TossimPacketModelC;
    // NeighborDiscoveryP.SetPower->TossimPacketModelC;
    //NeighborDiscoveryP.SetPower->TossimPacketModelC;
    

}