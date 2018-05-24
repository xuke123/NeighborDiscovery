#include <string.h>
#include <stdlib.h>
#include "WakeupTime.h"
#include "TLPMessage.h"
#include "NeighborDiscovery.h"
#include "printf.h"

#define DEBUG printf("%u\r\n", __LINE__);
module NeighborDiscoveryP {
	uses {
		interface WakeupTime;
		interface NeighborParameterManage;
		interface ListenRemote;
		interface AMSend as Send;
		interface Receive as Receive;
		interface NetBeaconPiggyback;
		interface TLPPacket;
		interface Boot;
		interface Timer<TMilli> as CheckTimer;   
		interface Timer<TMilli> as ListenTimer;  
		//interface Timer<TMilli> as StopRadioTimer;
		interface Timer<TMilli> as SampleTimer; //用来算radio打开的时间
		interface LocalTime<TMilli> as LocalTime;
		interface AMPacket;
		interface Packet;
        interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
		interface PacketTimeSyncOffset as PacketTimeSyncOffset;
		interface CC2420Packet;
		interface CC2420PacketBody;
		interface PacketSup;
		interface SplitControl;
		interface ReceiveMode;
		interface LocalWakeup;
		interface Random;
	}
	provides {
		interface LinkEstimate;
	}
}

implementation {
//-----------------------------------Global Variables----------------------------------------
	neighbor_tab_t neighborTable[NEIGHBOR_TABLE_SIZE];      
	two_hop_neighbor_tab_t potNeighborTable[POTNEIGHBOR_TABLE_SIZE];

	uint32_t checkTime = MAX_TIME;

	uint8_t initFlag = FALSE;
	
	message_t informMsg;		  

	tlp_message_t tlp;		  

	CoMechanismRecord record; 

	//这个变量表示，开启ｒａｄｉｏ是否成功，在初始化阶段未ｔｒｕｅ，在占空比阶段，若开ｒａｄｉｏ成功了为ｔｒｕｅ，否则为ｆａｌｓｅ
	//在超时任务处理中，如果为ｔｒｕｅ，更新参数，如果为ｆａｌｓｅ，只更新下次唤醒时间，更新定时器．
	bool startRadio = TRUE;
	

//-------------------------------------------------------------------------------------------

//-----------------------------------Function Declarations-----------------------------------
	void initNeighborTable();
	void initPotNeighborTable();
	void delNode(uint8_t nodeId);
	void delPotNode(uint8_t nodeId);
	uint8_t getNodeIdx(uint8_t nodeId);
	uint8_t getPotNodeIdx(uint8_t nodeId);
	neighbor_tab_t* insertNode(uint8_t nodeId, myPseudoPara_t* para, int32_t offsetTime);
	two_hop_neighbor_tab_t* insertPotNode(uint8_t nodeId, uint32_t rendezvousTime);
	uint8_t getEmptyIdx(uint8_t nodeId);
	uint8_t getPotEmptyIdx(uint8_t nodeId);
	uint8_t hash(uint8_t nodeId, uint8_t size);
	uint8_t fillInitBeaconTable(void* neighborSet, uint8_t maxSize);
	uint8_t fillRendezvousBeaconTable(void* neighborSet, uint8_t maxSize);
	void generateInformMsg(int32_t offsetTime);
	void freshCheckTimer(uint32_t freshTime);
	void receiveInitBeacon(message_t * msg, void * payload, uint8_t len);
	void receiveBeacon(message_t* msg, void* payload, uint8_t len);
	void updateLinkQuality(neighbor_tab_t* neig);
	neighbor_tab_t* getNeig(uint8_t nodeId);
	two_hop_neighbor_tab_t* getPotNeig(uint8_t nodeId);
	neighbor_tab_t* isRestarted(neighbor_tab_t* neig, int32_t offsetTime);
	void printNeighborTable();
	task void checkPotTable();

//--------------------------------------------------------------------------------------------

//-------------------------------------Events & Commands---------------------------------------
    event void Boot.booted() {
		myPseudoPara_t myInfo;
        uint32_t ltime = call LocalTime.get();
        myInfo.a = 20 * TOS_NODE_ID + 1;
		myInfo.c = 7;
		myInfo.m = 1000;
        myInfo.randomState = call Random.rand32() % 1000;
		myInfo.randomState += MIN_INTERVAL;
		myInfo.nextWakeupTime = ltime + myInfo.randomState;	
		call ReceiveMode.setInitDuration(INIT_DURATION);		
		call LocalWakeup.setWakeupMode(PSEUDO_WAKEUP);
		call LocalWakeup.setPseudoWakeupPara(myInfo);
		call CC2420Packet.setPower(&informMsg, SEND_POWER);
		call SplitControl.start();
		initNeighborTable();
		initPotNeighborTable();
        call NeighborParameterManage.init();
    }

	//初始化结束
    event void SplitControl.startDone(error_t err) { 
		uint8_t i;
		initFlag = TRUE;

		printNeighborTable();
		call NeighborParameterManage.printNeighborTable();
		call CheckTimer.stop();
		call ListenTimer.stop();
		atomic startRadio = FALSE;
		checkTime = MAX_TIME;
		for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
			if (neighborTable[i].nodeId != INVALID_NODE) {
				freshCheckTimer(neighborTable[i].expiredTime);
			}
		}

		//如果表里的时间都是过期的，这是有可能的
		if (checkTime == MAX_TIME) {
			freshCheckTimer(call WakeupTime.remoteNearestWakeupTime());
		}
	}

	event void SampleTimer.fired() {

	}

 	event void SplitControl.stopDone(error_t error) {

	}

	event void Send.sendDone(message_t* msg, error_t error) {
		//printf("SendDone error %u\r\n", error);
	}

	//收初始化ｂｅａｃｏｎ的处理流程
	void receiveInitBeacon(message_t * msg, void * payload, uint8_t len) {
		uint8_t nodeId, isInSet = 0;
		void* beaconPayload;					
		uint8_t payloadLen;      
		neighbor_tab_t* neig;
		cc2420_header_t *header = (cc2420_header_t *)call CC2420PacketBody.getHeader(msg);	
		uint32_t receiveTime = call PacketTimeStampMilli.timestamp(msg);
		uint32_t sendTime = ((init_beacon_t*)payload)->currentTime;
		int32_t offsetTime = sendTime - receiveTime;
		uint8_t neighborCnt;
		uint8_t _idx = 0;
		uint8_t* pset;

		beaconPayload = call TLPPacket.detachPayload(payload + ((init_beacon_t*)payload)->length, &payloadLen, NDP_ID);
		if (beaconPayload == NULL) 
			return;

		neighborCnt = ((init_beacon*)beaconPayload)->neighborCnt;
		pset = ((init_beacon*)beaconPayload)->neighborTable;
		nodeId = header->src;
		neig = getNeig(nodeId);
		neig = isRestarted(neig, offsetTime);

		while (_idx < neighborCnt && pset[_idx] != INVALID_NODE) {
			if (pset[_idx] == TOS_NODE_ID) {
				isInSet = 1;
				break;
			}
			_idx++;
		}

		if (neig == NULL) {
			delPotNode(nodeId);
			neig = insertNode(nodeId, &(((init_beacon*)beaconPayload)->para), offsetTime);
			if (!neig) {
				printf("one hop table is full!\r\n");
				return;
			}
			neig->listenBeaconCnt++;
			neig->flag = isInSet?TWOWAY:ONEWAY;
			neig->recoverCnt = 0;											        //recoverCnt
			neig->rcvBeaconCnt = 1;													//rcvBeaconCnt
			neig->rssi = call CC2420Packet.getRssi(msg) - 45;						//rssi
			neig->pathLoss = SEND_POWER - neig->rssi;								//pathLoss
			updateLinkQuality(neig);
		} else {
			int8_t nowRssi = call CC2420Packet.getRssi(msg) - 45;
			neig->listenBeaconCnt++;
			neig->flag = isInSet?TWOWAY:ONEWAY;									//flag
			neig->recoverCnt = 0;													//recoverCnt
			neig->rcvBeaconCnt++;													//rcvBeaconCnt							
			neig->rssi = 0.5 * nowRssi + 0.5 * neig->rssi;						    //rssi
			neig->pathLoss = SEND_POWER - neig->rssi;								//pathLoss
			updateLinkQuality(neig);
		}	
		//printf("rssi %d\r\n", neig->rssi);
		neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, MAX_BEACON_CNT - (((init_beacon*)beaconPayload)->count) % MAX_BEACON_CNT);
		freshCheckTimer(neig->expiredTime);
	}

	void receiveBeacon(message_t* msg, void* payload, uint8_t len) {
		uint8_t nodeId, isInSet = 0, beaconType;
		void* beaconPayload;					
        uint8_t payloadLen;      
		neighbor_tab_t* neig;
		uint8_t outQuality = 0;
		uint32_t receiveTime = call PacketTimeStampMilli.timestamp(msg);
		uint32_t sendTime = ((beacon_t*)payload)->currentTime;
		int32_t offsetTime = sendTime - receiveTime;	
		beaconPayload = call TLPPacket.detachPayload(payload + ((beacon_t*)payload)->length, &payloadLen, NDP_ID);
		if (beaconPayload == NULL)
			return;
		nodeId = call AMPacket.source(msg);	
		neig = getNeig(nodeId);
		neig = isRestarted(neig, offsetTime);

		beaconType = *(uint8_t*)beaconPayload;
		if (beaconType == RENDEZBEACON) {	
			uint8_t neighborCnt;
			uint8_t _idx = 0;
			struct neighbor_info* pset;
		//	printf("Rcv RENDEZ Beacon id %u\r\n", nodeId);
            printf("RENDEZ Beacon id %u at %lu ,rcv %lu, send %lu\r\n", nodeId, call LocalTime.get(), receiveTime, sendTime);
			pset = ((rendezvous_beacon*)beaconPayload)->neighborTable;
			neighborCnt = ((rendezvous_beacon*)beaconPayload)->neighborCnt;
			while (_idx < neighborCnt && pset[_idx].nodeId != INVALID_NODE) {
				int32_t deltaOffsetTime = -pset[_idx].offsetTime - offsetTime;
				if (pset[_idx].nodeId == TOS_NODE_ID && abs(deltaOffsetTime) < 30) {
					outQuality = pset[_idx].inQuality;
					isInSet = 1;
					break;
				}
				_idx++;
			}
		} else if (beaconType == BASEBEACON){
			printf("Rcv BASE Beacon id %u\r\n", nodeId);
			
			//printf("BASE Beacon id %u at %lu, rcv %lu, send %lu\r\n", nodeId, call LocalTime.get(), receiveTime, sendTime);
		} 

		if (neig == NULL) {
			delPotNode(nodeId);
			if (beaconType == RENDEZBEACON) 
				neig = insertNode(nodeId, &(((rendezvous_beacon*)beaconPayload)->para), offsetTime); 
		    else 
				neig = insertNode(nodeId, &(((base_beacon*)beaconPayload)->para), offsetTime); 
			if (!neig) {
				printf("one hop table is full!\r\n");
				return;
			}
			if (beaconType == RENDEZBEACON) 											//flag
				neig->flag = isInSet?TWOWAY:ONEWAY;
			else 
				neig->flag = ONEWAY;

			neig->listenBeaconCnt++;
			neig->recoverCnt = 0;											        //recoverCnt
			neig->rcvBeaconCnt = 1;													//rcvBeaconCnt
			neig->rssi = call CC2420Packet.getRssi(msg) - 45;					//rssi
			neig->pathLoss = SEND_POWER - neig->rssi;								//pathLoss
			if (neig->flag == TWOWAY && beaconType == RENDEZBEACON) 				//outQuality
				neig->outQuality = outQuality;
			updateLinkQuality(neig);
			neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, SHORT_CYCLE_VALUE);
		} else {
			int8_t nowRssi = call CC2420Packet.getRssi(msg) - 45;
			if (beaconType == RENDEZBEACON) {
				neig->flag = isInSet?TWOWAY:ONEWAY;
			}
			neig->recoverCnt = 0;	
			neig->listenBeaconCnt++;												//recoverCnt
			neig->rcvBeaconCnt++;													//rcvBeaconCnt							
			neig->rssi = 0.5 * nowRssi + 0.5 * neig->rssi;						    //rssi
			neig->pathLoss = SEND_POWER - neig->rssi;								//pathLoss
			updateLinkQuality(neig);

			if (beaconType == RENDEZBEACON) { 
				if (!neig->inQualityEstimated || (neig->rssi < RSSI_THR_H && neig->rssi > RSSI_THR_L)) {
					neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, SHORT_CYCLE_VALUE);
				} else {
					neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, MAX_BEACON_CNT - ((((rendezvous_beacon*)beaconPayload)->count) % MAX_BEACON_CNT)); 
				}
			} else {
				if (!neig->inQualityEstimated || (neig->rssi < RSSI_THR_H && neig->rssi > RSSI_THR_L)) {
					neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, SHORT_CYCLE_VALUE);
				} else {
					neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, MAX_BEACON_CNT - ((((base_beacon*)beaconPayload)->count) % MAX_BEACON_CNT)); 
				}
			} 
		}
		//printf("rssi %d\r\n", neig->rssi);
	    if (neig->flag == ONEWAY && initFlag && !neig->sendDataLastBeacon) {
			neig->sendDataLastBeacon = TRUE;
			generateInformMsg(offsetTime);
			call CC2420Packet.setPower(&informMsg, SEND_POWER);
			call PacketSup.setTxTime(&informMsg, call WakeupTime.getRemoteNextWakeupTime(nodeId));
			if (call Send.send(nodeId, &informMsg, sizeof(inform_t)) != SUCCESS) {
				printf("Call send Data FAIL at %lu\r\n", call LocalTime.get());
			} else {
				printf("Call send Data SUCCESS at %lu\r\n", call LocalTime.get());
			}	
		} else {
			neig->sendDataLastBeacon = FALSE;
		}

		if (beaconType == RENDEZBEACON && record.locked == FALSE) {
			atomic record.locked = TRUE;
			record.cnt = 0; 
			record.neighborCnt = ((rendezvous_beacon*)beaconPayload)-> neighborCnt;     //use it in looping
			record.offsetTime = call NeighborParameterManage.getClockDifference(nodeId);			
			memcpy(record.coSet, ((rendezvous_beacon*)beaconPayload)->neighborTable, sizeof(struct neighbor_info) * INITSIZE);
			post checkPotTable(); 
		}
	}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		uint8_t nodeId;
		inform_t* informPayload;
		neighbor_tab_t* neig;
		informPayload = (inform_t*)payload;
		nodeId = call AMPacket.source(msg);
		printf("Receive informMsg from %u\r\n", nodeId);
		neig = getNeig(nodeId);
		neig = isRestarted(neig, informPayload->offsetTime);
		if (neig == NULL) {
			delPotNode(nodeId);
			neig = insertNode(nodeId, &(informPayload->para), informPayload->offsetTime);
			if (!neig) {
				printf("one hop table is full!\r\n");
				return msg;
			}
		}
		//这里收到的话，应该链路置为双向
		neig->flag = TWOWAY;
		neig->expiredTime = call WakeupTime.getRemoteNextWakeupTime(nodeId);
		freshCheckTimer(neig->expiredTime);
		return msg;
	} 

    event message_t* NetBeaconPiggyback.receive(message_t* msg, void* payload, uint16_t len) {
		cc2420_header_t *header = (cc2420_header_t *)call CC2420PacketBody.getHeader(msg);
		beacon_t *pload = (beacon_t *)call CC2420PacketBody.getPayload(msg);
		//ack
		if (pload->dest == TOS_NODE_ID) {
			uint8_t nodeId = header->src;
			neighbor_tab_t* neig = getNeig(nodeId);
			neig->flag = TWOWAY;
			printf("rcv ack\r\n");
			return msg;	
		}

		if (call PacketTimeStampMilli.isValid(msg)) {
			//init beacon			
			if(header->type == INIT_BEACON) {
				receiveInitBeacon(msg, payload, len);
				return msg;
			} 
			//beacon
			receiveBeacon(msg, payload, len);
		}
		return msg;
    }

	task void checkPotTable() {
		struct neighbor_info* pset = record.coSet;	
		if (record.cnt < record.neighborCnt && pset[record.cnt].nodeId != INVALID_NODE && pset[record.cnt].nodeId != 0) {
			uint8_t nodeId = pset[record.cnt].nodeId;
			if (nodeId != TOS_NODE_ID) {
				if (!getNeig(nodeId)) {
					two_hop_neighbor_tab_t* potNeig = getPotNeig(nodeId);
					if (!potNeig) {
						uint32_t nextWakeupTime = pset[record.cnt].nextWakeupTime - record.offsetTime;
						potNeig = insertPotNode(nodeId, nextWakeupTime);
						if (!potNeig) {
							printf("two hop table is full!\r\n");
							return;
						}
					} else {
						potNeig->nextWakeupTime = pset[record.cnt].nextWakeupTime - record.offsetTime;
					}
					if (/*potNeig->listenBeaconCnt < MAX_POT_CHECKCNT || */((potNeig->nextWakeupTime - potNeig->lastListenTime) > MIN_POT_CHECK_INTERVAL))
		 				freshCheckTimer(potNeig->nextWakeupTime);
				}
			}
			record.cnt++;
			post checkPotTable();				//loop
		} else {	
			atomic	record.locked = FALSE;
		}
	}

	void freshCheckTimer(uint32_t freshTime) {
		if (freshTime < checkTime && (freshTime > call LocalTime.get())) {
//printf("CheckTimer %lu neig %lu now %lu\r\n", freshTime, freshTime + 15 + offsetTimeTest, call LocalTime.get());			
			checkTime = freshTime;
			call CheckTimer.stop();
			call CheckTimer.startOneShot(checkTime - call LocalTime.get());
			return;
		} 
	}

	event void CheckTimer.fired(){
		uint8_t i;
		uint32_t firedTime = call LocalTime.get();
		checkTime = MAX_TIME;

		if (initFlag) {
			uint32_t wakeupTime, wakeupTime2;
			wakeupTime = (call WakeupTime.getPseudoPara())->nextWakeupTime;
			wakeupTime2 = wakeupTime - (call WakeupTime.getPseudoPara())->randomState;
			//在发ｂｅａｃｏｎ时刻前后一小段时间去帧听的话会出问题，简单地避免这种情况
			if ((wakeupTime > firedTime-10 && wakeupTime < (firedTime + 10)) || 
							(wakeupTime2 > firedTime-10 && wakeupTime2 < (firedTime + 10))) {
				atomic startRadio = FALSE;
				call ListenTimer.startOneShot(1);
				return;
			}
			if (call ListenRemote.startListen(LISTEN_CHECK_DURATION) != SUCCESS) {
				atomic startRadio = FALSE;
				call ListenTimer.startOneShot(1);
				return;
			}
			call ListenTimer.startOneShot(LISTEN_CHECK_DURATION);
			atomic startRadio = TRUE;
		}

		for (i=0; i<POTNEIGHBOR_TABLE_SIZE; i++) {
			if (potNeighborTable[i].nodeId != INVALID_NODE) {
				if (potNeighborTable[i].nextWakeupTime >= firedTime && (potNeighborTable[i].nextWakeupTime < (firedTime + LISTEN_CHECK_DURATION))) {
					potNeighborTable[i].lastListenTime = firedTime;
				}
			}
		}
	}

	//关ｒａｄｉｏ失败了再去关
	task void stopListenAgain(){
		uint8_t error;
		error = call ListenRemote.stopListen();
		if(error==FAIL){
			printf("stopRadio again fail\r\n");
			post stopListenAgain();
		}
	}

	event void ListenTimer.fired() {
		uint8_t i;
		uint32_t firedTime = call LocalTime.get();
		uint8_t error;
		if (initFlag) {
			error = call ListenRemote.stopListen();
			if (error != SUCCESS) {
				if (error == FAIL) post stopListenAgain();	
				printf("stopListen fail, error %u\r\n", error);
			}
		}

		for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
			if (neighborTable[i].nodeId != INVALID_NODE) {
				if (neighborTable[i].expiredTime <= firedTime) {
					if (startRadio) {
						printf("Miss %u, %lu\r\n", neighborTable[i].nodeId, call LocalTime.get());
						neighborTable[i].recoverCnt++;
						neighborTable[i].listenBeaconCnt++;
						updateLinkQuality(&neighborTable[i]);
					}
					if (neighborTable[i].recoverCnt > MAX_RECOVER_CNT) 
						neighborTable[i].expiredTime = call WakeupTime.getRemoteNthWakeupTime(neighborTable[i].nodeId, LONG_CYCLE_VALUE);
				    else 
						neighborTable[i].expiredTime = call WakeupTime.getRemoteNthWakeupTime(neighborTable[i].nodeId, SHORT_CYCLE_VALUE);
				} else if (neighborTable[i].expiredTime < firedTime + SAFE_DOMAIN_LEN) {
				 	//if safe domain, then check next beacon
				 	neighborTable[i].expiredTime = call WakeupTime.getRemoteNextWakeupTime(neighborTable[i].nodeId);
				} 
				freshCheckTimer(neighborTable[i].expiredTime);
			}
		}

		for (i=0; i<POTNEIGHBOR_TABLE_SIZE; i++) {
			if (potNeighborTable[i].nodeId != INVALID_NODE) {
				if (potNeighborTable[i].nextWakeupTime <= firedTime) {
					//二跳表里的下次唤醒时间过期的情况
				} else if (potNeighborTable[i].nextWakeupTime > firedTime + SAFE_DOMAIN_LEN) {
					if ((potNeighborTable[i].nextWakeupTime - potNeighborTable[i].lastListenTime) > MIN_POT_CHECK_INTERVAL)
						freshCheckTimer(potNeighborTable[i].nextWakeupTime);
				}
			}
		}
	}

	void generateInformMsg(int32_t offsetTime) {
		myPseudoPara_t* myPara;
		inform_t* payload = (inform_t*)call Packet.getPayload(&informMsg, sizeof(inform_t));
		payload->offsetTime = - offsetTime;                 
		myPara = call WakeupTime.getPseudoPara();
		memcpy(&payload->para, myPara, sizeof(myPseudoPara_t));
	}

	event void WakeupTime.localWakeup() {
		static uint8_t count = 0; //count 1-8, transmit rendezvous_beacon when count = 8
		count++;   
		if (count > MAX_BEACON_CNT) 
			count = 1;
		if (initFlag == FALSE) {
			uint8_t neighborCnt;
			init_beacon* payload = (init_beacon*) call TLPPacket.getPayload(&tlp, sizeof(init_beacon));
			payload->type = INITBEACON;
			payload->count = count;
			memcpy(&payload->para, call WakeupTime.getPseudoPara(), sizeof(myPseudoPara_t));
			neighborCnt = fillInitBeaconTable(payload->neighborTable, MAX_INITBEACON_TABLE_SIZE);
			payload->neighborCnt = neighborCnt;		
			call NetBeaconPiggyback.set(&tlp, sizeof(init_beacon) + neighborCnt, NDP_ID);
			//printf("InitBeacon @%lu\r\n",call LocalTime.get()+15);
		} else if (count == MAX_BEACON_CNT) {  
			uint8_t neighborCnt;
			rendezvous_beacon* payload = (rendezvous_beacon*) call TLPPacket.getPayload(&tlp, sizeof(rendezvous_beacon));
			payload->type = RENDEZBEACON;
			payload->count = count;
			memcpy(&payload->para, call WakeupTime.getPseudoPara(), sizeof(myPseudoPara_t));
			neighborCnt = fillRendezvousBeaconTable(&(payload->neighborTable), MAX_RENDEZBEACON_TABLE_SIZE);
			payload->neighborCnt = neighborCnt;
		    call NetBeaconPiggyback.set(&tlp, sizeof(rendezvous_beacon) + neighborCnt * sizeof(struct neighbor_info), NDP_ID);
			//printf("RzBeacon @%lu\r\n",call LocalTime.get()+15);
		} else {
			base_beacon* payload = (base_beacon*) call TLPPacket.getPayload(&tlp, sizeof(base_beacon));
			payload->type = BASEBEACON;
			payload->count = count;
			memcpy(&payload->para, call WakeupTime.getPseudoPara(), sizeof(myPseudoPara_t));
			call NetBeaconPiggyback.set(&tlp, sizeof(base_beacon), NDP_ID);
			//printf("BaseBeacon @%lu\r\n",call LocalTime.get()+15);
		}

		if (count == 1) {
			printNeighborTable();
		}
	}

//------------------------------------- LinkEstimate-------------------------------------------

	command int8_t LinkEstimate.getRssi(uint8_t nodeId) {
		uint8_t idx;
		idx = getNodeIdx(nodeId);
		if (idx == INVALID_INDEX) {}
			return 0;
		
		return neighborTable[idx].rssi;
	}

	command int8_t LinkEstimate.getPathloss(uint8_t nodeId) {
		uint8_t idx;
		idx = getNodeIdx(nodeId);
		if (idx == INVALID_INDEX)
			return 0;

		return neighborTable[idx].pathLoss;
	}

	command uint16_t LinkEstimate.getOutQuality(uint8_t nodeId) {
		uint8_t idx;
		idx = getNodeIdx(nodeId);
		if (idx == INVALID_INDEX)
			return 0;

		return neighborTable[idx].outQuality;
	}

	command uint16_t LinkEstimate.getInQuality(uint8_t nodeId) {
		uint8_t idx;
		idx = getNodeIdx(nodeId);
		if (idx == INVALID_INDEX)
			return 0;

		return neighborTable[idx].inQuality;
	}

//---------------------------------------------------------------------------------------------

	neighbor_tab_t* isRestarted(neighbor_tab_t* neig, int32_t offsetTime) {
		if (neig) {
			int32_t deltaOffsetTime = offsetTime - call NeighborParameterManage.getClockDifference(neig->nodeId);
			//如果off——b的绝对值大于30，那该节点应该是重启了，需要去删除
			if (abs(deltaOffsetTime) > 30) {
				delNode(neig->nodeId);
				neig = NULL;
			} else {
				call NeighborParameterManage.setClockDifference(neig->nodeId, offsetTime);
			}
		}
		return neig;
	}

	void updateLinkQuality(neighbor_tab_t* neig) {
		if (neig->listenBeaconCnt == WINDOW_LENGTH) {
			if (!neig->inQualityEstimated) {
				neig->inQualityEstimated = TRUE;
				neig->inQuality = (100 * neig->rcvBeaconCnt)/neig->listenBeaconCnt;
			} else {
				uint8_t lastInQuality = neig->inQuality;
				uint8_t nowInQuality = 100 * neig->rcvBeaconCnt/neig->listenBeaconCnt;
				neig->inQuality = (CO_ALPHA * lastInQuality + (10 - CO_ALPHA) * nowInQuality)/10;
			}
			neig->listenBeaconCnt = 0;
			neig->rcvBeaconCnt = 0;
			printf("inQuality = %u\r\n", neig->inQuality);
		}
	}

	void printNeighborTable() {
		uint8_t i;
		uint8_t result[200] = "";
		uint8_t tmp[40];
     //  printf("-------------neig list info------------\r\n");
        for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
            if (neighborTable[i].nodeId != INVALID_NODE) {
				//printf("---id = %u, inQuality = %u linkflag = %u\r\n", neighborTable[i].nodeId, neighborTable[i].inQuality, neighborTable[i].flag);
                sprintf(tmp,"--id = %u ,Lq = %u (%u)\r\n",neighborTable[i].nodeId, neighborTable[i].inQuality, neighborTable[i].flag);
	            strcat(result,tmp);		
			} 
         } 
		 //sprintf(str,"\r\n");
		// strcat(result,"\r\n");
		 printf("%s",result);
    //     printf("--------------------------------------\r\n");
	}

 	void initNeighborTable() {
		uint8_t i;
        for (i=0; i < NEIGHBOR_TABLE_SIZE; i++) {
            neighborTable[i].nodeId = INVALID_NODE;
			neighborTable[i].recoverCnt = 0;
			neighborTable[i].flag = LOST;
			neighborTable[i].listenBeaconCnt = 0;
			neighborTable[i].rcvBeaconCnt = 0;
			neighborTable[i].rssi = 0;
			neighborTable[i].pathLoss = 0;
			neighborTable[i].inQuality = 0;
			neighborTable[i].outQuality = 0;
			neighborTable[i].lqi = 0;
			neighborTable[i].inQualityEstimated = FALSE;
			neighborTable[i].sendDataLastBeacon = FALSE;
        }
	}

	void initPotNeighborTable() {
		uint8_t i;
		for (i=0; i<POTNEIGHBOR_TABLE_SIZE; i++) {
            potNeighborTable[i].nodeId = INVALID_NODE; 
			potNeighborTable[i].listenBeaconCnt = 0;
			potNeighborTable[i].rcvBeaconCnt = 0;   
			potNeighborTable[i].nextWakeupTime = 0;
        }
	}

	void delNode(uint8_t nodeId) {
		uint8_t idx;
		idx = getNodeIdx(nodeId);
		if (idx == INVALID_INDEX) {
			printf("can't find node, delNode fail!\r\n");
			return;
		}
		printf("delete node %u ,idx %u\r\n", nodeId, idx);
		neighborTable[idx].nodeId = INVALID_NODE;
		call NeighborParameterManage.delete(nodeId);
	}

	void delPotNode(uint8_t nodeId) {
		uint8_t idx;
		idx = getPotNodeIdx(nodeId);
		if (idx == INVALID_INDEX) {
			return;
		}
		printf("delete potNode %u\r\n", nodeId);

		potNeighborTable[idx].nodeId = INVALID_NODE;
	}

	uint8_t getNodeIdx(uint8_t nodeId) {
		uint8_t idx, i;
		idx = hash(nodeId, NEIGHBOR_TABLE_SIZE);
		if (neighborTable[idx].nodeId == nodeId) 
			return idx;

		i = (idx+1) % NEIGHBOR_TABLE_SIZE;
		while (i != idx) {
			if (neighborTable[i].nodeId == nodeId) {
         		return i;
			}
     		 i = (i+1) % NEIGHBOR_TABLE_SIZE;
		}

		return INVALID_INDEX;
	}

	uint8_t getPotNodeIdx(uint8_t nodeId) {
		uint8_t idx, i;
		idx = hash(nodeId, POTNEIGHBOR_TABLE_SIZE);
		if (potNeighborTable[idx].nodeId == nodeId)
			return idx;

		i = (idx+1) % POTNEIGHBOR_TABLE_SIZE;
		while (i != idx) {
			if (potNeighborTable[i].nodeId == nodeId)
         		return i;
     		 i = (i+1) % POTNEIGHBOR_TABLE_SIZE;
		}

		return INVALID_INDEX;
	}

	neighbor_tab_t* getNeig(uint8_t nodeId) {
		neighbor_tab_t* neig;
		uint8_t idx = getNodeIdx(nodeId);
		if (idx != INVALID_INDEX ) 
			neig = &neighborTable[idx];
		else
			neig = NULL;
	
		return neig;
	}

	two_hop_neighbor_tab_t* getPotNeig(uint8_t nodeId) {
		uint8_t idx = getPotNodeIdx(nodeId);
		if (idx != INVALID_INDEX)
			return &potNeighborTable[idx];
		else 
			return NULL;
	} 


	neighbor_tab_t* insertNode(uint8_t nodeId, myPseudoPara_t* para, int32_t offsetTime) {
		uint8_t idx;
		idx = getEmptyIdx(nodeId);	
		if (idx == INVALID_INDEX) 
			return NULL;

		call NeighborParameterManage.insert(nodeId, para, offsetTime);
		neighborTable[idx].nodeId = nodeId;
		neighborTable[idx].recoverCnt = 0;
		neighborTable[idx].flag = ONEWAY;
		neighborTable[idx].listenBeaconCnt = 0;
		neighborTable[idx].rcvBeaconCnt = 0;
		neighborTable[idx].rssi = 0;
		neighborTable[idx].pathLoss = 0;
		neighborTable[idx].inQuality = 0;
		neighborTable[idx].outQuality = 0;
		neighborTable[idx].lqi = 0;
		neighborTable[idx].inQualityEstimated = FALSE;
		neighborTable[idx].sendDataLastBeacon = FALSE;
		printf("Insert node %u offsetTime %ld\r\n", nodeId, offsetTime);

		return &neighborTable[idx];
	}

	two_hop_neighbor_tab_t* insertPotNode(uint8_t nodeId, uint32_t nextWakeupTime) {
		uint8_t idx;
		idx = getPotEmptyIdx(nodeId);
		if (idx == INVALID_INDEX) 
			return NULL;

		potNeighborTable[idx].nodeId = nodeId;
		potNeighborTable[idx].nextWakeupTime = nextWakeupTime;
		printf("Insert potNode %u nextWakeupTime %lu\r\n",nodeId, nextWakeupTime);
		return &potNeighborTable[idx];
	}

	uint8_t getEmptyIdx(uint8_t nodeId) {
		uint8_t idx, i;
		uint8_t worstNodeIdx = INVALID_INDEX;
		uint8_t minInquality = 0xFF;
		idx = hash(nodeId, NEIGHBOR_TABLE_SIZE);		
		if (neighborTable[idx].nodeId == INVALID_NODE) 
			return idx;
		i = (idx+1) % NEIGHBOR_TABLE_SIZE;
		while (i != idx) {
			if (neighborTable[i].nodeId == INVALID_NODE)
         		return i;
     		 i = (i+1) % NEIGHBOR_TABLE_SIZE;
		}

		printf("test, error\r\n");
		//如果没找到空位，说明表已经满了，把链路质量最差的那个点给删了,没测
		for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
			if (neighborTable[i].inQuality < minInquality) {
				minInquality = neighborTable[i].inQuality;
				worstNodeIdx = i;
			}
		}

		delNode(neighborTable[worstNodeIdx].nodeId);
		return worstNodeIdx;
	}

	uint8_t getPotEmptyIdx(uint8_t nodeId) {
		uint8_t idx, i;
		idx = hash(nodeId, POTNEIGHBOR_TABLE_SIZE);
		if (potNeighborTable[idx].nodeId == INVALID_NODE)
			return idx;

		i = (idx+1) % POTNEIGHBOR_TABLE_SIZE;
		while (i != idx) {
			if (potNeighborTable[i].nodeId == INVALID_NODE)
         		return i;
     		 i = (i+1) % POTNEIGHBOR_TABLE_SIZE;
		}

		return INVALID_INDEX;
	}

	uint8_t hash(uint8_t nodeId, uint8_t size) {
		uint8_t value;
		value = nodeId % size;
		return value;
	}

	uint8_t fillInitBeaconTable(void* neighborSet, uint8_t maxSize) {
		uint8_t idx = 0;
		uint8_t* pset = (uint8_t*)neighborSet;
		uint8_t i;

		for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
			if (neighborTable[i].nodeId != INVALID_NODE) {
				 pset[idx++] = neighborTable[i].nodeId;
			}
			if (idx >= maxSize) 
				break;
		}
		return idx;
	}

    //填充预约点beacon里的neighborTable,返回邻节点个数
	uint8_t fillRendezvousBeaconTable(void* neighborSet, uint8_t maxSize) {
		uint8_t idx = 0;
		struct neighbor_info* pset = (struct neighbor_info*)neighborSet;
		uint8_t i;

		for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
			if (neighborTable[i].nodeId != INVALID_NODE) {
				pset[idx].nodeId = neighborTable[i].nodeId;
				pset[idx].rssi = neighborTable[i].rssi;
				pset[idx].inQuality = neighborTable[i].inQuality; 
				pset[idx].nextWakeupTime = call WakeupTime.getRemoteNextWakeupTime(pset[idx].nodeId);
				pset[idx].offsetTime = call NeighborParameterManage.getClockDifference(pset[idx].nodeId);
				if (pset[idx].nextWakeupTime < call LocalTime.get()) 
					printf("Error: fillRendezvousBeaconTable()\r\n");
				idx++;
			} 
			/*else 
				pset[idx].nodeId = INVALID_NODE;*/
			if(idx >= maxSize) 
				break;
		}
		return idx;
	}





//---------------------------------------------------------------------------------------------.
//	async event void BeaconSnoop.receive(message_t * msg, void * payload, uint8_t len) {
// 		uint8_t nodeId, idx, isInSet = 0;
// 		void* beaconPayload;	//get beaconPayload by detaching payload, its first byte represents the type of beacon				
//         uint8_t payloadLen;     //length of payload, not used yet 
// 		neighbor_tab_t* neig;
// 		cc2420_header_t *header = (cc2420_header_t *)call CC2420PacketBody.getHeader(msg);	
// 		uint32_t receiveTime = call PacketTimeStampMilli.timestamp(msg);
// 		uint32_t sendTime = ((init_beacon_t*)payload)->currentTime;
// 	    int32_t offsetTime = sendTime - receiveTime;		
// //printf("BeaconSnoop.receive type = %u\r\n", header->type);
//         return ;//-----------------------------------------------------------------------------------------------//
// 		if(header->type == INIT_BEACON) {
// printf("BeaconSnoop.receive INIT_BEACON\r\n");
// 			beaconPayload = call TLPPacket.detachPayload(payload + ((init_beacon_t*)payload)->length, &payloadLen, NDP_ID);
// 		 	if (beaconPayload == NULL) 
// 				return;

// 			if (call PacketTimeStampMilli.isValid(msg)) {
// 				uint8_t neighborCnt;
// 				uint8_t _idx = 0;
// 				uint8_t* pset;
// printf("PacketTimeStampMilli.isValid(msg)\r\n");
// 				neighborCnt = ((init_beacon*)beaconPayload)->neighborCnt;
// 				pset = ((init_beacon*)beaconPayload)->neighborTable;
// 				nodeId = header->src;
// 				idx = getNodeIdx(nodeId);
// 				if (idx != INVALID_INDEX)
// 					neig = &neighborTable[idx];
// 				else
// 					neig = NULL;

// 				while (_idx < neighborCnt && pset[_idx] != INVALID_NODE) {
// 					if (pset[_idx] == TOS_NODE_ID) {
// 						isInSet = 1;
// 						break;
// 					}
// 					_idx++;
// 				}
// 				if (neig == NULL) {
// 					uint8_t __idx;
// 					delPotNode(nodeId);
// 					__idx = insertNode(nodeId, &(((init_beacon*)beaconPayload)->para), offsetTime); 
// 					if (__idx == INVALID_INDEX)
// 						return ;
// 					neig = &neighborTable[__idx];
// 					call WakeupTime.updateInitListenBeaconCnt(nodeId, &neig->listenBeaconCnt);	
// 					neig->flag = isInSet?TWOWAY:ONEWAY;
// 					neig->recoverCnt = 0;											        //recoverCnt
// 					neig->rcvBeaconCnt = 1;													//rcvBeaconCnt
// 					neig->rssi = call CC2420Packet.getRssi(msg) - 45;						//rssi
// 					neig->pathLoss = SEND_POWER - neig->rssi;								//pathLoss
// 					if (neig->inQuality != 0)
// 						neig->inQuality = (100 * neig->rcvBeaconCnt)/neig->listenBeaconCnt;
// 					else 
// 						printf("Error: BeaconSnoop.receive 1\r\n");
// 				} else {
// 					uint8_t nowRssi = call CC2420Packet.getRssi(msg) - 45;
// 					call NeighborParameterManage.update(nodeId, &((init_beacon*)beaconPayload)->para, offsetTime);
// 					call WakeupTime.updateInitListenBeaconCnt(nodeId, &neig->listenBeaconCnt);
// 					neig->flag = isInSet?TWOWAY:ONEWAY;									//flag
// 					neig->recoverCnt = 0;													//recoverCnt
// 					neig->rcvBeaconCnt++;													//rcvBeaconCnt							
// 					neig->rssi = 0.5 * nowRssi + 0.5 * neig->rssi;						    //rssi
// 					neig->pathLoss = SEND_POWER - neig->rssi;								//pathLoss
// 					if (neig->listenBeaconCnt != 0 ) 
// 						neig->inQuality = (100 * neig->rcvBeaconCnt)/neig->listenBeaconCnt;
// 					else 
// 						printf("Error: BeaconSnoop.receive 2\r\n");
// 				}	
// 				neig->expiredTime = call WakeupTime.getRemoteNthWakeupTime(nodeId, MAX_BEACON_CNT - (((init_beacon*)beaconPayload)->count) % MAX_BEACON_CNT);
// 			}
// 		}
// 	}

//---------------------------------------------------------------------------------------------

			//uint8_t idx;
			// if (nodeId != TOS_NODE_ID) {
			// 	idx = getNodeIdx(nodeId);
			// 	if (idx == INVALID_INDEX) {	//if node is found in neighborTable, do nothing
			// 		idx = getPotNodeIdx(nodeId);
			// 		if (idx == INVALID_INDEX) {//if node is not found in both neighborTable and potNeighborTable, add it to potNeighborTable
			// 			uint32_t nextWakeupTime;
			// 			nextWakeupTime = pset[record.cnt].nextWakeupTime - record.offsetTime;
			// 			idx = insertPotNode(nodeId, nextWakeupTime);
			// 		} else 					
			// 			potNeighborTable[idx].nextWakeupTime = pset[record.cnt].nextWakeupTime - record.offsetTime;
			// 		if (potNeighborTable[idx].listenBeaconCnt < MAX_POT_CHECKCNT || ((potNeighborTable[idx].nextWakeupTime - potNeighborTable[idx].lastListenTime) > MIN_POT_CHECK_INTERVAL))
			// 			freshCheckTimer(potNeighborTable[idx].nextWakeupTime);
			// 	}
			// }
					//printf("StartListen at %lu\r\n", firedTime);

		//radioOnTime = firedTime;

		// for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
		// 	if (neighborTable[i].nodeId != INVALID_NODE) {
		// 		uint32_t nextWakeupTime = call WakeupTime.getRemoteNextWakeupTime(neighborTable[i].nodeId);
		// 		//uint32_t nextWakeupTime = call WakeupTime.getRemoteNthWakeupTime(neighborTable[i].nodeId);
		// 		//printf("LocalfiredTime = %lu, LocalAdvanceCheckTime = %lu\r\n", firedTime, nextWakeupTime);
		// 		if (nextWakeupTime >= firedTime && (nextWakeupTime < (firedTime + LISTEN_CHECK_DURATION))) { 
		// 			neighborTable[i].listenBeaconCnt++;
		// 			//printf("listenBeacon = %u\r\n", neighborTable[i].listenBeaconCnt);
		// 		}
		// 	}
		// }
}
