interface PacketSup{
    command uint32_t getTxTime(message_t *msg);
    command void setTxTime(message_t *msg, uint32_t txInterval);
}