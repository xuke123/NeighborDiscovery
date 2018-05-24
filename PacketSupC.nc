configuration PacketSupC {
     provides interface PacketSup;
}

implementation {
    components PacketSupP;
    components CC2420PacketC;
    PacketSup = PacketSupP;
    PacketSupP.CC2420PacketBody -> CC2420PacketC;
}