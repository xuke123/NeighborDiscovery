//这个表必须和网络层匹配
//插入新节点或更新(节点号,参数,b,checktime),删除旧节点（节点号）,更新b,更新b和checktime
interface NeighborParameterManage {
    command void init();//初始化整个表
    command error_t insert(uint8_t nodeId, myPseudoPara_t *pInfo, int32_t offsetTime);// 插入一个表项
    command error_t update(uint8_t nodeId, myPseudoPara_t *pInfo, int32_t offsetTime);
    command void delete(uint8_t nodeId);//删除节点
    command void setClockDifference(uint8_t nodeId , int32_t _offsetTime);//更新时钟偏差
    command int32_t getClockDifference(uint8_t nodeId);//读取时钟偏差
    command void printNeighborTable();
}

