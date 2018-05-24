interface NetBeaconPiggyback {
    //上层保留缓冲好了，下层MAC直接复制得了
    command uint8_t set(tlp_message_t* payload, uint16_t len, uint8_t type);
    event message_t* receive(message_t* msg, void* payload, uint16_t len);
}