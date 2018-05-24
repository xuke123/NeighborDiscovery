module PacketSupP {
   provides interface PacketSup;
   uses interface CC2420PacketBody;
}
implementation {
   
   //获取唤醒时间
   command uint32_t PacketSup.getTxTime(message_t* msg) {
         return  ((cc2420_metadata_t *)call CC2420PacketBody.getMetadata(msg))->wakeupTime;
   }

   //设置唤醒时间   
   command void PacketSup.setTxTime(message_t* msg, uint32_t txInterval){
         ((cc2420_metadata_t *)call CC2420PacketBody.getMetadata(msg))->wakeupTime = txInterval;
   }
}
