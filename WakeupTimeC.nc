configuration WakeupTimeC {
    provides interface WakeupTime;
    provides interface NeighborParameterManage;
}

implementation {
    components MacControlC;
    components WakeupTimeP;
    components RandomC;
    
    WakeupTime = WakeupTimeP;
    NeighborParameterManage = WakeupTimeP;
     
    WakeupTimeP.LocalWakeup->MacControlC;

    components  LocalTimeMilliC as timeC;
    WakeupTimeP.LocalTime->timeC; 
    components new TimerMilliC() as NWTimeC;
    WakeupTimeP.NWTimer->NWTimeC;
    WakeupTimeP.Random -> RandomC;
}
