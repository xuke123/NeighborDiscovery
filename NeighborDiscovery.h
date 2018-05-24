#ifndef NEIGHBOR_DISCOVRY_H
#define NEIGHBOR_DISCOVRY_H
#include "WakeupTime.h"
#include "/opt/tinyos-2.1.2/wustl/upma/lib/macs/RbMac/RbMac.h"
//flag的几种类型
enum {
    LOST,
    ONEWAY,
    TWOWAY,
    CHECK,
};

//Beacon的几种类型
enum {
    INITBEACON,
    BASEBEACON,
    RENDEZBEACON,
};

enum {
    ONEHOP,
    TWOHOP,
};

enum {
    LONG,
    SHORT,
};

enum {
    MAX_TIME = 0xFFFFFFFF,
    MAX_BEACON_CNT = 4,               //达到这个值发预约点Beacon,小于这个值发普通Beancon或初始Beacon
    WINDOW_LENGTH = 3,
    MAX_RECOVER_CNT = 3,              //最大尝试恢复次数
    MAX_LISTEN_CNT = 8,               //帧听次数大于这个值，listenBeaconCnt，rcvBeaconCnt就重新计数
    //RCV_BEACON_CNT_THR = 1,           //二跳邻接点在MAX_LISTEN_CNT次帧听中收到的次数大于RCV_BEACON_CNT_THR才会加入到一跳邻节点表
    MAX_POT_CHECKCNT = 2,            //二跳表里的如果listenBeaconCnt小于这个值可以去帧听，如果大于还要进行其他判断． 
    NDP_ID = 22,
    INITSIZE = 100,                    //协同机制记录信息里邻节点set的长度
    LISTEN_CHECK_DURATION = 40,      //一次帧听时长
    MIN_POT_CHECK_INTERVAL = 1000,   //在二跳表中某个节点两次检查间隔大于这个值时才会去帧听
    NEIGHBOR_TABLE_SIZE = 8,
    POTNEIGHBOR_TABLE_SIZE = 8,  
    MAX_INITBEACON_TABLE_SIZE = 8,   //初始Beacon里neighTable里元素的最大个数
    MAX_RENDEZBEACON_TABLE_SIZE = 8, //预约点Beacon里neighTable里元素的最大个数
    //SEND_POWER = 3,                   //发射功率
    RSSI_THR = -85,                    //rssi的阈值,大于它才去发inform_t
    CO_GAMMA = 5,                   //计算lqi的系数 目前没有LQI
    CO_ALPHA = 3,                       //计算inQuality的系数,
    //INQUALITY_THR = 50,                 //链路质量的阈值
    SAFE_DOMAIN_LEN = 10,               //安全检查区域的长度
    LONG_CYCLE_VALUE = 4,               //长周期的检查间隔
    SHORT_CYCLE_VALUE = 1,              //短周期的检查间隔
    LQ_THR_H = 90,
    LQ_THR_L = 30,
    RSSI_THR_H = -85,
    RSSI_THR_L = -95,
    INIT_DURATION = 4000,
};

//一跳邻节点表项
typedef struct {      
    uint8_t nodeId;
    uint32_t expiredTime;              //到期时间,可以是下次预约点时间也可能不是．不是下次预约点时间的情况：１．收到ｂｅａｃｏｎ进入短周期２．没收到ｂｅａｃｏｎ
    uint8_t recoverCnt;                //尝试恢复次数，被帧听节点如果此次没有被听到．该值＋１．该值大于阈值，表明链路质量较差，进入长周期．一旦被帧听节点被听到了，该值清０
    uint8_t flag;                      //ONEWAY，TWOWAY ,LOST
    uint8_t listenBeaconCnt;           //侦听Beacon的次数(每侦听一次加一) 达到最大值将listenBeaconCnt和rcvBeaconCnt 重新计数
    uint8_t rcvBeaconCnt;              //收到Beacon的次数，这里的两个cnt是用来算链路质量的
    //uint8_t latestBeaconRcvRatio;
    //链路信息
    int8_t rssi;                       //接收信号强度单位dbm 
    int8_t pathLoss;                   //衰落(单位：dBm)，底层获得发送功率-rssi
    uint8_t lqi;                       //使用Lqi估计部分
    uint8_t inQuality;                 //从邻节点到自己的链路质量，根据Lqi和rssi算的
    uint8_t outQuality;                //从自己到邻节点的链路质量，传过来的
    bool sendDataLastBeacon;
    bool inQualityEstimated;
} neighbor_tab_t;

//两跳邻节点表项
typedef struct {
    uint8_t  nodeId;
    uint32_t lastListenTime;           //上一次侦听的时间,暂时没用
    uint32_t nextWakeupTime;           //下次唤醒时间
    uint8_t listenBeaconCnt;           //侦听Beacon的次数,与1跳表里的意义不一样
    uint8_t rcvBeaconCnt;              //收到Beacon的次数，与阈值比较，大于才加到邻节点表，暂时就取１，只要听到就加到表里
} two_hop_neighbor_tab_t;

//预约点Beacon里所携带的邻节点信息表项
struct neighbor_info {
    uint8_t nodeId;
    uint8_t rssi;
    uint8_t inQuality;
    uint32_t nextWakeupTime;           //下次唤醒时间
    int32_t offsetTime;
};

//基本beacon类型，只包含自己的信息
typedef struct {
    uint8_t type;
    uint8_t count;
    //节点工作参数
    myPseudoPara_t para;
} base_beacon;

//预约点beacon类型，除自己信息以外还有其邻节点的信息
typedef struct {
	uint8_t type;  
	uint8_t count;
    int8_t txPower;         //节点发送功率(单位：dbm)
    myPseudoPara_t para;
    uint8_t neighborCnt;    //邻节点数量
	struct neighbor_info neighborTable[];
} rendezvous_beacon;

//初始化Beacon，只有邻节点号，但是没有邻节点的具体信息
typedef struct {	
	uint8_t type; 
    uint8_t count;
    myPseudoPara_t para;
    uint8_t neighborCnt;
	uint8_t neighborTable[];
} init_beacon;


//邻节点通知包
typedef struct {
    myPseudoPara_t para;
    int32_t offsetTime;
} inform_t;

//协同机制记录信息
typedef struct {
    uint8_t locked; 
    uint8_t cnt;   
    uint8_t neighborCnt; 
    int32_t offsetTime;  
    struct neighbor_info coSet[INITSIZE];	
} CoMechanismRecord;

#endif
