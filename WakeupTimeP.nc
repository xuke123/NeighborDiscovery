#include "./WakeupTime.h"
#include "NeighborDiscovery.h"
#include "/opt/tinyos-2.1.2/wustl/upma/lib/macs/RbMac/RbMac.h"

module WakeupTimeP {
    provides interface WakeupTime;
    provides interface NeighborParameterManage;
    uses interface LocalWakeup;                    
    uses interface LocalTime<TMilli> as LocalTime; 
    uses interface Timer<TMilli> as NWTimer; 
    uses interface Random;
}

implementation {
//---------------------------------------Global Variables---------------------------------------------
    uint32_t advanceTime = 15;
    neighbor_parameter_t neigParaTable[NEIGHBOR_TABLE_SIZE]; 
    myPseudoPara_t myself;
//----------------------------------------------------------------------------------------------------

//-------------------------------------function declarations-------------------------------------------
    void initNeighborTable();
    uint8_t getEmptyIndex(uint8_t nodeId);
    uint8_t getNodeIndex(uint8_t nodeId);
    uint8_t hash(uint8_t nodeId);
//-----------------------------------------------------------------------------------------------------

//-----------------------------------------NodeWakeTimer-----------------------------------------------
    task void nodeWakeup() {
        myself = call LocalWakeup.getPseudoWakeupPara();        
        call NWTimer.startOneShot(myself.nextWakeupTime - call LocalTime.get() - advanceTime);            
    }

    event void NWTimer.fired() {
	    myself.randomState = (myself.a * myself.randomState + myself.c) % myself.m;
    	myself.randomState += MIN_INTERVAL;
    	myself.nextWakeupTime += myself.randomState;   
        call NWTimer.startOneShot(myself.nextWakeupTime - call LocalTime.get() - advanceTime);
        signal WakeupTime.localWakeup();
    }
//-----------------------------------------------------------------------------------------------------	

//------------------------------------------WakeupTime-------------------------------------------------

    command myPseudoPara_t* WakeupTime.getPseudoPara() {
       return &myself;
    }

    command error_t WakeupTime.setNotifyAdvanceTime(uint8_t tms) {
        advanceTime = tms;
        return SUCCESS;
    }

    //获得邻节点的下次唤醒时间，这个时间是邻节点的唤醒时间转换到本地时间，再减一个提前量．
    command uint32_t WakeupTime.getRemoteNextWakeupTime(uint8_t nodeId) {
        uint8_t idx;
	    uint32_t neigTime;
	    uint32_t nodeCurrentTime = call LocalTime.get();

        uint8_t _a, _c;
        uint16_t _m;
        uint32_t _randomState, _nextWakeupTime;

        idx = getNodeIndex(nodeId);
        if (idx == INVALID_INDEX) 
            return 0;
        
        _a = neigParaTable[idx].a;
        _c = neigParaTable[idx].c;
        _m = neigParaTable[idx].m;
        _randomState = neigParaTable[idx].randomState;
        _nextWakeupTime = neigParaTable[idx].nextWakeupTime;

		neigTime = (uint32_t)(nodeCurrentTime + neigParaTable[idx].offsetTime);
		while (_nextWakeupTime <= (neigTime + advanceTime)) {
                _randomState = (_a * _randomState + _c) % _m;
                _randomState += MIN_INTERVAL;
                _nextWakeupTime += _randomState;
		}
        neigParaTable[idx].randomState = _randomState;
        neigParaTable[idx].nextWakeupTime = _nextWakeupTime;
        
        if (neigParaTable[idx].nextWakeupTime > (neigTime + advanceTime)) {
            return (uint32_t)(neigParaTable[idx].nextWakeupTime - advanceTime - neigParaTable[idx].offsetTime);
        }
		return MAX_TIME;
    }
   
    command uint32_t WakeupTime.getRemoteNthWakeupTime(uint8_t nodeId, uint8_t cnt) {
        uint8_t idx;
        uint32_t nodeCurrentTime = call LocalTime.get();
        uint32_t neigTime;

        uint8_t _a, _c;
        uint16_t _m;
        uint32_t _randomState, _nextWakeupTime;

        idx = getNodeIndex(nodeId);
        if (idx == INVALID_INDEX)  
            return 0;

        _a = neigParaTable[idx].a;
        _c = neigParaTable[idx].c;
        _m = neigParaTable[idx].m;
        _randomState = neigParaTable[idx].randomState;
        _nextWakeupTime = neigParaTable[idx].nextWakeupTime;

        neigTime = (uint32_t)(nodeCurrentTime + neigParaTable[idx].offsetTime);//本地节点此时时刻对应的邻节点时间
		while (_nextWakeupTime <= (neigTime + advanceTime)) {
            _randomState = (_a * _randomState + _c) % _m;
            _randomState += MIN_INTERVAL;
            _nextWakeupTime += _randomState;
		}
        neigParaTable[idx].randomState = _randomState;
        neigParaTable[idx].nextWakeupTime = _nextWakeupTime;

        if (neigParaTable[idx].nextWakeupTime > (neigTime + advanceTime)) {
            uint32_t nowInterval = neigParaTable[idx].randomState;
            uint32_t beginTime = neigParaTable[idx].nextWakeupTime - neigParaTable[idx].offsetTime;
            uint32_t totalInterval = 0;
            while (--cnt) {
                nowInterval = (neigParaTable[idx].a * nowInterval +  \
                        neigParaTable[idx].c) % neigParaTable[idx].m + MIN_INTERVAL;
                totalInterval += nowInterval;
            }
            return beginTime + totalInterval - advanceTime;
        }
		return MAX_TIME;
    }
	
    //没调
    command uint32_t WakeupTime.remoteNearestWakeupTime() {
        uint8_t idx;
	    uint32_t neigTime;
	    uint32_t nodeCurrentTime = call LocalTime.get();

        uint32_t latestWakeupTime = 0xFFFFFFFF;

        for (idx=0; idx<NEIGHBOR_TABLE_SIZE; idx++) {
            if (neigParaTable[idx].nodeId == INVALID_NODE)  
                continue;
            neigTime = (uint32_t)(nodeCurrentTime + neigParaTable[idx].offsetTime);

            while (neigParaTable[idx].nextWakeupTime <= neigTime + advanceTime) {
                neigParaTable[idx].randomState = (neigParaTable[idx].a * neigParaTable[idx].randomState\
                                            + neigParaTable[idx].c) % neigParaTable[idx].m;
                neigParaTable[idx].randomState += MIN_INTERVAL;
                neigParaTable[idx].nextWakeupTime +=  neigParaTable[idx].randomState;
	        }
            latestWakeupTime = latestWakeupTime < (neigParaTable[idx].nextWakeupTime - neigParaTable[idx].offsetTime) ? latestWakeupTime:(neigParaTable[idx].nextWakeupTime - neigParaTable[idx].offsetTime);
        }

        if (latestWakeupTime == 0xFFFFFFFF) 
            return MAX_TIME;
        return (uint32_t)(latestWakeupTime - advanceTime);
    } 

    //没调
    command uint8_t WakeupTime.remoteNearestWakeupNode() {
        uint8_t idx;
	    uint32_t neigTime;
	    uint32_t nodeCurrentTime = call LocalTime.get();

        uint32_t latestWakeupTime = 0xFFFFFFFF;
        uint8_t nearestId = INVALID_NODE;

        for (idx=0; idx<NEIGHBOR_TABLE_SIZE; idx++) {
            if (neigParaTable[idx].nodeId == INVALID_NODE)  
                continue;
            neigTime = (uint32_t)(nodeCurrentTime + neigParaTable[idx].offsetTime);

            while (neigParaTable[idx].nextWakeupTime <= neigTime + advanceTime) {
                neigParaTable[idx].randomState = (neigParaTable[idx].a * neigParaTable[idx].randomState\
                                            + neigParaTable[idx].c) % neigParaTable[idx].m;
                neigParaTable[idx].randomState += MIN_INTERVAL;
                neigParaTable[idx].nextWakeupTime +=  neigParaTable[idx].randomState;
	        }

            if (latestWakeupTime>(neigParaTable[idx].nextWakeupTime - neigParaTable[idx].offsetTime)) {
                latestWakeupTime = (neigParaTable[idx].nextWakeupTime - neigParaTable[idx].offsetTime);
                nearestId = idx;
            }
        }

        if (nearestId == INVALID_NODE) 
            return INVALID_NODE;
        return nearestId;
    }

    // command void WakeupTime.updateInitListenBeaconCnt(uint8_t nodeId, uint8_t* listenBeaconCnt) {
    //     uint8_t idx;
    //     uint8_t _a, _c;
    //     uint16_t _m;
    //     uint32_t _randomState, _nextWakeupTime;
    //     uint32_t neigTime;
     
    //     idx = getNodeIndex(nodeId);
    //     if (idx == INVALID_INDEX) 
    //         return;
    //     _a = neigParaTable[idx].a;
    //     _c = neigParaTable[idx].c;
    //     _m = neigParaTable[idx].m;
    //     _randomState = neigParaTable[idx].randomState;
    //     _nextWakeupTime = neigParaTable[idx].nextWakeupTime;
    //     neigTime =  (uint32_t)(call LocalTime.get() + neigParaTable[idx].offsetTime);

    //     while (_nextWakeupTime < neigTime) {
    //         _randomState = (_a * _randomState + _c) % _m;
    //         _randomState += MIN_INTERVAL;
    //         _nextWakeupTime += _randomState;
    //         (*listenBeaconCnt)++;
	// 	}
    //     neigParaTable[idx].randomState = _randomState;
    //     neigParaTable[idx].nextWakeupTime = _nextWakeupTime;
    //     //printf("nextWakeupTime = %lu", neigParaTable[idx].nextWakeupTime);
    // }
//------------------------------------------------------------------------------------------------------

//-------------------------------------NeighborParameterManage-----------------------------------------

    command void NeighborParameterManage.init() {
        initNeighborTable();
	    post nodeWakeup();
    }

    command error_t NeighborParameterManage.insert(uint8_t nodeId, myPseudoPara_t *pInfo, int32_t offsetTime) {
        uint8_t idx;
        idx = getEmptyIndex(nodeId);
        if (idx == INVALID_INDEX) {
            printf("Can't find empty pos, insert fail!\r\n");
            return FALSE;
        }

        neigParaTable[idx].nodeId = nodeId;
        neigParaTable[idx].a = pInfo->a;
        neigParaTable[idx].c = pInfo->c;
        neigParaTable[idx].m = pInfo->m;
        neigParaTable[idx].randomState = pInfo->randomState;
        neigParaTable[idx].nextWakeupTime = pInfo->nextWakeupTime;
        neigParaTable[idx].offsetTime = offsetTime;

        return SUCCESS;
    }

    command error_t NeighborParameterManage.update(uint8_t nodeId, myPseudoPara_t *pInfo, int32_t offsetTime) {
        uint8_t idx;
        idx = getNodeIndex(nodeId);
        if (idx == INVALID_INDEX) {
            printf("Can't find node, update fail!\r\n");
            return FALSE;
        } 
        neigParaTable[idx].offsetTime = offsetTime;

        return SUCCESS;
    }

    command void NeighborParameterManage.delete(uint8_t nodeId) {
        uint8_t idx;
        idx = getNodeIndex(nodeId);
        if (idx == INVALID_INDEX) {
            printf("Can't find node, delete fail!\r\n");
            return;
        }
        printf("delete paratable node %u, idx %u!\r\n", nodeId, idx);
        neigParaTable[idx].nodeId = INVALID_NODE;
    }

    command void NeighborParameterManage.setClockDifference(uint8_t nodeId, int32_t offsetTime) {
        uint8_t idx;
        idx = getNodeIndex(nodeId);
        if (idx == INVALID_INDEX) {
            printf("Can't find node, setClockDifference fail!\r\n");
            return;
        }
        neigParaTable[idx].offsetTime = offsetTime;
    }

    command int32_t NeighborParameterManage.getClockDifference(uint8_t nodeId) {
        uint8_t idx;
        idx = getNodeIndex(nodeId);
        if (idx == INVALID_INDEX) {
            printf("Can't find node, getClockDifference fail!\r\n");
            //printf("nodeId %u", nodeId);
            return INVALID_OFFSET;
        } 
        return neigParaTable[idx].offsetTime;
    }

//-----------------------------------------------------------------------------------------------------

//-------------------------------------------functions-------------------------------------------------

    void initNeighborTable() {
        uint8_t i;
        for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
            neigParaTable[i].nodeId = INVALID_NODE ;
        }
    }

    uint8_t getEmptyIndex(uint8_t nodeId) {
        uint8_t idx, i;
        idx = hash(nodeId);
		if (neigParaTable[idx].nodeId == INVALID_NODE)
			return idx;

		i = (idx+1) % NEIGHBOR_TABLE_SIZE;
		while (i != idx) {
			if (neigParaTable[i].nodeId == INVALID_NODE)
         		return i;
     		 i = (i+1) % NEIGHBOR_TABLE_SIZE;
		}

		return INVALID_INDEX;
    }

    uint8_t getNodeIndex(uint8_t nodeId) {
        uint8_t idx, i;
        idx = hash(nodeId);
		if (neigParaTable[idx].nodeId == nodeId)
			return idx;

		i = (idx+1) % NEIGHBOR_TABLE_SIZE;
		while (i != idx) {
			if (neigParaTable[i].nodeId == nodeId)
         		return i;
     		 i = (i+1) % NEIGHBOR_TABLE_SIZE;
		}

        return INVALID_INDEX;
    }

    uint8_t hash(uint8_t nodeId) {
		uint8_t value;
		value = nodeId % NEIGHBOR_TABLE_SIZE;
		return value;
	}
//-----------------------------------------------------------------------------------------------------

//----------------------------------------------debug---------------------------------------------------

    command void NeighborParameterManage.printNeighborTable() {
        uint8_t i;
        printf("-------------neig param info------------\r\n");
        for (i=0; i<NEIGHBOR_TABLE_SIZE; i++) {
            if (neigParaTable[i].nodeId != INVALID_NODE) {
                printf("id: %u, offsetTime: %ld\r\n", 
                    neigParaTable[i].nodeId, neigParaTable[i].offsetTime);
            } 
         } 
         printf("---------------------------------------\r\n");
    }
//------------------------------------------------------------------------------------------------------

}
