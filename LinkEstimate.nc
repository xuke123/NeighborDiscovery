interface LinkEstimate {
  command int8_t getRssi(uint8_t nodeId); //获得节点的RSSI值
  command int8_t getPathloss(uint8_t nodeId); //获得路径损耗 (节点发送功率 减去节点的接收功率)
  command uint16_t getOutQuality(uint8_t nodeId);//inquality
  command uint16_t getInQuality(uint8_t nodeId);//outquality
}
