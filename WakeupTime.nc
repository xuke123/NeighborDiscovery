//在无效的时候返回-1，比如很多时候，wakeupTime不见的有效，也有可能节点不存在什么的
interface WakeupTime { 
    command myPseudoPara_t* getPseudoPara();
    command error_t setNotifyAdvanceTime(uint8_t tms); //设置本节点唤醒通知的提前时间
    event void localWakeup(); //每次本地唤醒的事件（每次唤醒通知使用该接口的组件）
    command uint32_t getRemoteNextWakeupTime(uint8_t nodeId); //基于当前时间，得到邻节点下一次唤醒时刻
    command uint32_t getRemoteNthWakeupTime(uint8_t nodeId, uint8_t n);//基于当前时间，得到邻节点第N个唤醒时刻
    command uint32_t remoteNearestWakeupTime();//邻节点最近唤醒的时间
    command uint8_t  remoteNearestWakeupNode();//最近唤醒的节点编号
   // command void updateInitListenBeaconCnt(uint8_t nodeId, uint8_t* listenBeaconCnt);
    //command void getListenCntInInitiation(uint8_t nodeId, uint8_t* listenBeaconCnt, uint32_t remainTime);
}