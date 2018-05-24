generic module LppAMSenderP() {
  provides interface AMSend;
  uses {
    interface AMSend as SubAMSend;
    interface WakeupTime;
    interface PacketSup; 
  }
}

implementation {
  command error_t AMSend.send(am_addr_t addr, message_t* msg, uint8_t len) {
    // uint32_t TxTime = call PacketSup.getTxTime(msg);
    // if (TxTime != 0) {
	  // //如果是广播，则获取最小（最近）的唤醒时间
    //     if (addr == AM_BROADCAST_ADDR) {
    //         TxTime = call WakeupTime.remoteNearestWakeupTime(); 
    //     } else {
	  // //否则，获取指定节点的下一次唤醒时间
    //         TxTime = call WakeupTime.getRemoteNextWakeupTime(addr);
    //     }
	  // //得到了下次唤醒时间之后，去设置。
    //      call PacketSup.setTxTime(msg, TxTime);
    // }    
    // //调用AMSend接口中的send命令发送数据包，
    return call SubAMSend.send(addr, msg, len);
  }

  event void SubAMSend.sendDone(message_t* msg, error_t error) { signal AMSend.sendDone(msg, error); }
  command error_t AMSend.cancel(message_t* msg) { return call SubAMSend.cancel(msg); }
  command uint8_t AMSend.maxPayloadLength() { return call SubAMSend.maxPayloadLength(); }
  command void* AMSend.getPayload(message_t* msg, uint8_t len) { return call SubAMSend.getPayload(msg, len); }
  event void WakeupTime.localWakeup() {};

}
