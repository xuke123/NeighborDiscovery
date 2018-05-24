#include <string.h>
#include "WakeupTime.h"
#include "LTPMessage.h"

module NeighborDiscoveryP{
    uses {
         interface WakeupTime;
         interface NeighborManage;
         interface NodeWakeup;
   
         interface Receive  as Receive;
    
         interface AMSend as Send;
         interface ListenRemote;
    
         interface BeaconPiggyBack;
		 interface LTPPackect;
    
         interface AMPacket;
         interface Packet;
		 interface PackSup;
		 interface LTPPacket;
    
	     interface CC2420Packet;
     
          interface PacketTimeStamp<TMilli, uint32_t> as PacketTimeStampMilli;
          interface LocalTime<TMilli> as LocalTime;
    
		  interface Timer<TMilli> as CheckActTimer;
		  interface Timer<TMilli> as ListernCtlTimer;
	
	      interface Boot;
		  interface SplitControl;
	}
    provides{
             interface neighborInfo;
             interface LinkEstimate;
    }
}
implementation{
	enum {
       EVICT_EETX_THRESHOLD = 55,
       MAX_AGE = 6,
       BEST_EETX = 0,
       INVALID_RVAL = 0xff,
       INVALID_NEIGHBOR_ADDR = 0xff,
       VERY_LARGE_EETX_VALUE = 0xff,
       ALPHA = 9,
       DLQ_PKT_WINDOW = 3,
       BLQ_PKT_WINDOW = 1,
       LARGE_EETX_VALUE = 60
  };
    neighbor_table_entry_t NeighborTable[NEIGHBOR_TABLE_SIZE];
    potneighbor_table_entry_t PotneighborTable[POTNEIGHBOR_TABLE_SIZE];

    uint32_t check_uppertime; //检查的上限时间

	uint8_t InitPhaseCompFlag=false;
    LIST(CheckTaskQueue);

	message_t NotifyData_msg;
	ltp_message_t NdpBeaconP;

    //协同机制记录信息
	typedef struct {
		uint8_t locked;
		uint8_t neigId;
		uint8_t rssi;   
		uint8_t cnt;    
		int32_t offsettime;  
		struct Neighbor set[SETSIZE]；	
    } CoMechanismRecord;

	CoMechanismRecord CoMeRecord;
   //-------------------------------------------------------------------------
   //初始化结束标志
   event void SplitControl.startDone(error_t err){
	   InitPhaseCompFlag=true;
   }
   //-----------------------------------------------------------------------
   //需要参考4bit哪些怎样做
   void refreshlinkQ(struct neig_tab_t * neig,uint8_t loss){
   }
   //------------------------------------------------------------------------
    void clear_node(uint8_t nodeId){
        uint8_t i=0;
		for(i=0;i<NEIGHBOR_TABLE_SIZE;i++){
			if(NeighborTable[i].nodeId==nodeId){
                list_remove(CheckTaskQueue,&NeighborTable[i]);
				NeighborTable[i].nodeId=INVALID_RVAL;
				call NeighborManage.neigDelete(nodeId);
				break;
			}
		}
    }
	void clear_potnode(uint8_t nodeId){
		uint8_t i=0; 
		for(i=0;i<POTNEIGHBOR_TABLE_SIZE;i++){
			if(PotneighborTable[i].nodeId==nodeId){
                list_remove(CheckTaskQueue,&PotneighborTable[i]);
				PotneighborTable[i].nodeId=INVALID_RVAL;
				break;
			}
		}
	}
  //-----------------------------------------------------------------
    uint8_t find_neig_Idx(uint8_t target) {
       uint8_t i;
       for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
            if(NeighborTable[i].nodeId==target)
                return i;
        }
       return INVALID_RVAL;
    }
	uint8_t find_potneig_Idx(uint8_t target) {
       uint8_t i;
       for (i = 0; i < POTNEIGHBOR_TABLE_SIZE; i++) {
            if(PotneighborTable[i].nodeId==target)
                return i;
        }
       return INVALID_RVAL;
    }
	uint8_t find_emptyneig_idx(){
	   uint8_t i;
       for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
            if(NeighborTable[i].nodeId==INVALID_RVAL)
                return i;
        }
       return INVALID_RVAL;
	}
	uint8_t find_emptypotneig_idx(){
	   uint8_t i;
       for (i = 0; i < POTNEIGHBOR_TABLE_SIZE; i++) {
            if(PotneighborTable[i].nodeId== INVALID_RVAL)
                return i;
        }
       return INVALID_RVAL;
	}
//-----------------------------------------------------------------------------
uint8_t initPotneighbor(uint8_t target,uint32_t cktime){
     uint8_t idx,i;
	 idx=find_emptypotneig_idx();
	 if(idx==INVALID_RVAL) return INVALID_RVAL;

	 PotneighborTable[idx].type=UNKNOWN;
	 PotneighborTable[idx].nodeId=target;
	 PotneighborTable[idx].check_time=cktime;
	 PotneighborTable[idx].checking=CHECK_PROCESSING;
	 PotneighborTable[idx].handle_times=0;
	 for(i=0;i<4;i++){
		 PotneighborTable[idx].referees[i]=INVALID_RVAL;
	 }
     insertNode(CheckTaskQueue,neig);

	  return idx;
}
//节点编号，节点工作参数，节点偏移时间，返回值为数组下标索引,其他信息自己补-------------------------------
uint8_t initNeighbor(uint8_t targetId,PseudoPa *para,uint8_t b){
	        uint8_t idx;
            call NeighborManage.neigInsertOrUpdate(sourceId,Para,b);
            idx=find_emptyneig_idx();
             
			if(idx==INVALID_RVAL) return INVALID_RVAL;

            NeighborTable[idx].nodeId=sourceId;
            NeighborTable[idx].type=KNOWN;
            NeighborTable[idx].last_random_state=pload->para.random_state;
		    NeighborTable[idx].check_time=para->nextWakeupTime-b;//这个不用计算
            NeighborTable[idx].checking=CHECK_NOPROCESS;

            NeighborTable[idx].link_status=TWOWAY;
		    NeighborTable[idx].

            insertNode(CheckTaskQueue,neig);
			fresh_checktimer();

			return idx;
	}
//-------------------------------------------------------------------
    void generateNotifyData(int32_t offsetTime){
		NodeWorkPara* myInfo;
        struct NotifyData *pload=\
		             (struct NotifyData *)call Packet.getPayload(&NotifyData_msg,sizeof(struct NotifyData));

		pload->offsetTime= offsetTime;                 

		myInfo=call NodeWakeup.getPara();
		memcpy(&pload->para,myInfo,sizeof(NodeWorkPara));
		//设置发送时间为0，即立即发送
		call PacketSup.setTxInterval(&NotifyData_msg,0);
        call CC2420Packet.setPower(&NotifyData_msg,SETPOWER);
	}
	//---------------------------------------------------------------
   event void Boot.booted(){
	   uint8_t i;
	   //初始化四部分 CheckTaskQueue，NeighborTable PotneighborTable WakeupTimeC
	   list_init(CheckTaskQueue);
       for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
		   NeighborTable[i].nodeId=INVALID_RVAL;
        }
       for(i=0;i<POTNEIGHBOR_TABLE_SIZE;i++){
			PotneighborTable[i].nodeId=INVALID_RVAL;    
		}
		call NeighborManage.init();
	   
   }
	//----------------------------------------------------------
	event message_t* Receive.receive(message_t *msg,void *payload,uint8_t len){ 
		cc2420_header_t *header;
		struct NotifyData *pload;
		uint32_t preBeaconTime;
		struct PseudoPara* neigInfo;

		uint8_t idx,sourceId;
		neighbor_table_entry_t *neig;

		sourceId=call AMPacket.source(msg);

        idx=find_neig_Idx(sourceId);
        if(idx!=INVALID_RVAL)
            neig=&NeighborTable[idx];
        else
            neig=NULL;

		pload=(struct NotifyData *)payload; 
      
//如果节点存在，判断节点是否重启，如果重启就删除节点
		if(neig){
			int32_t _off_b=pload->offsetTime-call NeighborManage.getClockDifference(sourceId);
			if(abs(_off_b)>30){
				clear_node(sourceId);
				neig=NULL;
			}else{
                   call NeighborManage.updateClockDifference(sourceId,-pload->offsetTime);//互为相反数
            }
		}
		if(!neig){
			uint8_t idx;
			clear_potnode(sourceId);
            initNeighbor(sourceId,pload->para,-pload->offsetTime);
		}
		return msg;
	}

//---------------------------------------------------------------------
   //Beacon的处理（InitBeacon BaseBeacon RzBeacon)-----------------------------------------------------------------------
    event message_t* Snoop.receive(message_t *msg,void *mpayload,uint8_t len){
        uint8_t sourceId,idx,Inset;
        uint8_t BeaconType;
        uint32_t receive_time,neig_send_time；
        int32_t offsetTime;
		void *payload;
		uint8_t plen;
        neighbor_table_entry_t *neig;

        if(AMPacket.type(msg)!=BEACON_ID) 
            return msg;

		payload=call LTPPacket.detachPayload(mpayload,&plen,NDP_ID);
		if(payload==NULL)
		    return msg;

        sourceId=call AMPacket.source(msg);

        //查找节点
        idx=find_neig_Idx(sourceId);
        if(idx!=INVALID_RVAL)
            neig=&NeighborTable[idx];
        else
            neig=NULL;

        //获取接收和发送时间，并计算时间差
        receive_time=call PacketTimeSyncOffset.get(msg);
        neig_send_time=call PacketTimeStampMilli.timestamp(msg);
        offsetTime=neig_send_time-receive_time;//偏移时间为邻居节点时间-自身接收时间

       //判断节点是否重启，重启删除，不重启更新节点偏移时间
		if(neig){
			int32_t _off_b=offsetTime-call NeighborManage.getClockDifference(sourceId);
			if(abs(_off_b)>30){
					clear_node(sourceId);
				    neig=NULL;
			}else{
                   call NeighborManage.updateClockDifference(sourceId,offsetTime);
            }
		}

        //判断Beacon类型
		BeaconType=*(uint8_t *)payload;

		//先根据beacon类型，提取节点集合是否有我,用于单双向处理办法
		if(beaconType==INIT_BEACON){       
			uint8_t index=0;
			uint8_t *pset=((InitBeaconP*)payload) ->set;
			while(index<INITSETSIZE&&pset[index]!=FREENODE){
				if(pset[index]==TOS_NODE_ID){
					InSet=1;break;
				}
				index++;
			}
		}else if(beaconType==RENDEZ){
			uint8_t index=0;
			struct Neighbor *pset=((RendezBeaconP*)payload)->set;
			while(index<INITSETSIZE&&pset[index].nodeId!=FREENODE){
				if(pset[index].nodeId==TOS_NODE_ID){
					InSet=1;break;
				}
				index++;
			}
		}
		//处理链路变化,并决定下一步动作
		if(neig==NULL){
			uint8_t idx;
            //先删掉在PotNeighborTable的节点，WakeupQueue节点也要删除
            clear_potnode();
            //在NeighorTable WakeupTimeC 中插入节点
            idx=initNeighbor(sourceId,pload->para,-pload->offsetTime);
            //如果插入节点成功，将节点插入WakeupQueue，随后刷新检查队列，触发检查任务
            if(idx==INVALID_RVAL) return msg;
            //更改节点链路状态
            if(beaconType==BASEBEACON){
                  NeighborTable[idx].link_status=ONEWAY;
            }else{
                  NeighborTable[idx].link_status=isInSet?TWOWAY:ONEWAY;
            }
			//记录节点的rssi值
			NeighborTable[idx].rssi=call CC2420Packet.getRssi(msg)-45;
            //RENDEZBEACON BASEBEACON （除了INITBEACON）单向链路发送数据,当前状态也不能是初始化状态
            if(BeaconType!=INITBEACON&&neig_te->link_status=ONEWAY&&InitPhaseCompFlag){
                generateNotifyData(node_offsetTime);
				call ListenBeacon.send(srcId,&NotifyData_msg,sizeof(struct NotifyData));
            }
			neig=&NeighborTable[idx];
		}else{
            //非BEASEBEACON要更新链路状态
			if(BeaconType!=BASEBEACON){
				neig->link_status=isInSet?TWOWAY:ONEWAY;
				neig->fail_time=0;
			}
			neig->rssi=call CC2420Packet.getRssi(msg)-45;
			//BASE RENDEZ认为单向时，需要判断是否重新发送数据包，更新链路状态------>判断办法！！！！
			 if(BeaconType!=INITBEACON&&neig_te->link_status=ONEWAY&&InitPhaseCompFlag){
                  if(){
					 generateNotifyData(node_offsetTime);
				     call ListenBeacon.send(srcId,&NotifyData_msg,sizeof(struct NotifyData)); 
				  }
			 }
        }
		//更新预约机制
			if(neig&&neig->checking==CHECK_PROCESSING){
				neig->check_time=receive_time-ADVANCE2CHECK;
				neig->next_check_cnt=MAX_BEACONCOUNT-base->count+1;
				neig->checking=CHECK_SUCCESS;
			}
		//}
        //提取协同机制必要信息，post协同检查的任务
		if(neig&&BeaconType==RENDZEBEACON||CoMeRecord.locked==false){
            CoMeRecord.locked=true;
			CoMeRecord.neigId=neig->nodeId;
            CoMeRecord.rssi=neig->rssi;
			CoMeRecord.cnt=0;
			CoMeRecord.offsettime=call NeighborManage.getClockDifference(sourceId);
			memcpy(CoMeRecord.set,((RendezBeaconP*)payload)->set,sizeof(struct Neighbor)*SETSIZE);
			post obtainPotNodeOnebyOne(); 
		}
        return msg;
    }
	//----------------------------------------------------------------------------------
	uint8_t ThisRefereeResult(uint8_t neigRssi,uint8_t potneigRssi,uint8_t cosaRef){
            uint8_t alp=PSEND-neigRssi;
			float L23=pow(10,(PSEND-potneigRssi)/alp);
			float Lthr=pow(10,(PSEND-PTHR)/alp);
			if(10+L23*L23-L23*cosa_refs[cosaRef]>Lthr*Lthr)
					 return 1;
			return 0;
	}
	//------------------------------------------------------------------------------------
	void CheckOrNot(uint8_t idx){
		uint8_t ri;
		uint8_t JudgRes=0;
		struct Neighbor* BeaconPotneig=&CoMeRecord.set[CoMeRecord.cnt];  //Beacon中的潜在节点位置
		potneighbor_table_entry_t* TablePotneig=&PotneighborTable[idx];  //PotneighborTable中节点位置，裁判是发送Beacon的节点
		for(ri=0;ri<4;ri++){
            if(TablePotneig-> referees[ri]==CoMeRecord.neigId)//已知节点是裁判
			     if(abs(TablePotneig->rssi[ri]-BeaconPotneig->rssi)>10){
						TablePotneig->cosa[ri]=5;
				 }else{
						TablePotneig->handle_times++;
						if(TablePotneig->handle_times>=CHECKUNKNOWNTIMES){
							if(TablePotneig->cosa[ri]>0) TablePotneig->cosa[ri]--;
								 TablePotneig->handle_times=0;
						 }
				 }
				TablePotneig->rssi[ri]=BeaconPotneig->rssi;
				TablePotneig->judgments[ri]=
				            ThisRefereeResult(CoMeRecord.rssi,BeaconPotneig->rssi,TablePotneig->cosa[ri]);    
				break;
			}else if(potneig_entry->referees[ri]==INVALID_RVAL){
                TablePotneig->referees[ri]=CoMeRecord.neigId;
		        TablePotneig->cosa[ri]=5;//开五档
		        TablePotneig->rssi[ri]=BeaconPotneig->rssi;
		        TablePotneig->judgments[ri]=
				           ThisRefereeResult(CoMeRecord.rssi,BeaconPotneig->rssi,TablePotneig->cosa[ri]);
                break;
			}
		}

		if(ri==4){//裁判已满,换掉第一个裁判
           TablePotneig->referees[0]=CoMeRecord.neigId;
		   TablePotneig->cosa[0]=5;//开五档
		   TablePotneig->rssi[0]=BeaconPotneig->rssi;
		   TablePotneig->judgments[0]=
		                   ThisRefereeResult(CoMeRecord.rssi,BeaconPotneig->rssi,TablePotneig->cosa[ri]);
		}
       //综合裁判给的结果
	   for(ri=0;ri<4;ri++){
          if(TablePotneig->judgments[ri])
		     JudgRes++;
	   }
	   if(JudgRes>=2){
		  PotneighborTable[idx].check_time=pset[CoMeRecord.cnt].offsetTime+CoMeRecord.offsettime;
	      insertNode(CheckTaskQueue,&PotneighborTable[idx]);
	   }
	}
	//------------------------------------------------------------------------------------
	task void obtainPotNodeOnebyOne(){
		struct Neighbor* pset=CoMeRecord.set;
		if(CoMeRecord.cnt!=SETSIZE&&pset[CoMeRecord.cnt]!=INVALID_RVAL){
			uint8_t target=pset[CoMeRecord.cnt].nodeId;
			uint8_t idx;
            
			if(target!=TOS_NODE_ID){
				idx=find_neig_Idx(target);
				if(idx==INVALID_RVAL){//邻居节点表没找到
                    idx=find_potneig_Idx(target);
					if(idx==INVALID_RVAL){//pot邻节点也没找到
					    uint32_t cktime;
					    cktime=pset[CoMeRecord.cnt].offsetTime+CoMeRecord.offsettime;
                        idx=initPotneighbor(target,cktime);
					}else{//在pot找到
                        CheckOrNot(idx);
					}
				}
			}
			CoMeRecord.cnt++;
			post obtainPotNodeOnebyOne();
		}else{
			CoMeRecord.locked=false;
			fresh_checktimer();
		}
	}
	//-----------------------------------------------OK(未测)-----------------------------------------------
	void fresh_checktimer(){
		uint32_t nowtime,nextcktime;
		struct sche_tab_t* te,*tenext;
		nowtime=call LocalTime.get();
		te=list_head(wakeup_manager);

		while(nowtime+10 > te->check_time){
			tenext=list_item_next(te);
			list_remove(wakeup_manager,te);
			if(te->type==KNOWNNODE){
				uint32_t checktime=te->check_time;
				te->check_time=call WakeupTime.MultiRemoteWakeupTime(te->nodeId,checktime,&(te->last_random_state),3);
		     	insertNode(CheckTaskQueue,te);
				te->checking=CHECK_NOPROCESS;
		    }else{
                //未知节点参数更新
				te->checking=CHECK_NOPROCESS;
			}
			te=tenext;
		}
		te=list_head(CheckTaskQueue);
		if(te){
			nextcktime=te->check_time-nowtime;
			call CheckTaskTimer.stop();
			call CheckTaskTimer.startOneShot(nextcktime);
		}

	}
	//--------------------------------------------------------------------------------------------------------------
	event void checkTimer.fired(){
		struct sche_tab_t *te;
	
	   check_uppertime=call LocalTime.get()+CHECKUPPERTIME;

		te=list_head(CheckTaskQueue);
		for(;te!=NULL;te=list_item_next(te)){
			if(te->check_time < uppertime){
				te->checking=CHECK_PROCESSING;
			}
		}
		if(call ListenRemote.start()!=SUCCESS){
		}
		call radio.startOneShot(LISTEN_CHECK_DURATION);
	}
	//-------------------------------------------------------------------------------------------
	event void radioTimer.fired(){
	   struct sche_tab_t* te,*tenext;
	   struct neig_tab_t* tn;

       call ListenRemote.stop();

	   te=list_head(CheckTaskQueue);

	   for(;te!=NULL;te=tenext){
			tenext=list_item_next(te);
			if(te->checktime > check_uppertime) break;
			if(te->type==KNOWNNODE){
				tn=(struct neighbor_table_entry_t *)te;
				if(tn->checking==CHECK_SUCCESS){
                   refreshlinkQ(neig,1);
				}else if(tn->checking==CHECK_PROCESSING){
				   uint8_t checkcnt=0;
				   uint32_t checktime;
				   refreshlinkQ(neig,0); 

				  tn->fail_time++;
				  if(tn->fail_time>=5){
					    checkcnt=MAX_BEACONCOUNT;
				  }else if(tn->linkQ<=THRESHODLINKQ){
                        checkcnt=MAX_BEACONCOUNT/2;
				  else{
				        checkcnt=1;
				  } 
				  checktime=te->check_time;
				  te->check_time=
					    call WakeupTime.MultiRemoteWakeupTime(te->nodeId,checktime,&(te->last_random_state),checkcnt);
				}else {//这里有些莫名其妙，可能是新添加的节点,这里只能靠fresh_checktimer 帮忙了
				}
				tn->checking=CHECK_NOPROCESS;
			}else{
				//更新相关相关参数
				list_remove(CheckTaskQueue, te);
				te->checking=CHECK_NOPROCESS;
			}	
	   }
	   fresh_checktimer();
	}
	//------------------------------------------------OK（未测）-------------------------------------------------------
	uint8_t FillNeighborSet_withTime(void* set,uint8_t sz){
		uint8_t index=0;
        struct Neighbor* pset=(struct Neighbor*)set;
		uint8_t i=0;
		uint8_t max_index=sz/sizeof(struct Neighbor);

		for(i=0;i<NEIGHBOR_TABLE_SIZE;i++){
	    	if(NeighborTable[i].nodeId!=INVALID_RVAL){
				if(NeighborTable[i].linkQ<=THRESHODLINKQ) continue;
			    pset[index].nodeId=NeighborTable[i].nodeId;
			    pset[index].rssi=NeighborTable[i].rssi;
			    pset[index].offsetTime=NeighborTable[i].check_time;
		     	index++; 
	    	}

			if(index>=max_index) break;
	    }
        if(index<sz) pset[index++].nodeId=INVALID_RVAL;
		return index*sizeof(struct Neighbor);
	}
	//--------------------------------------------------------------------------------------------------------------
	uint8_t FillNeighborSet_withoutTime(void* set,uint8_t sz){
		uint8_t index=0;
	    uint8_t *pset=(void *)set;
     	uint8_t i;

	   for(i=0;i<NEIGHBOR_TABLE_SIZE;i++){
	    	if(NeighborTable[i].nodeId!=INVALID_RVAL){
			  pset[index++]=NeighborTable[i].nodeId;
			  if(index>=sz) break;
	    	}
	    }
	   if(index<sz) pset[index++]=INVALID_RVAL;
	    return index;
	}
	//----------------------------------------------------------------------------------------------------
	//这里上层单独维护一个缓冲？？？还是什么？？？
    event void  NodeWakeup.advanceWakeup(){
		static uint8_t count=0;
		count++;    
		if(count>MAX_BEACONCOUNT) count=0;
		//如果节点为初始化状态
		if(InitPhaseCompFlag==false){
            InitBeaconP *pload=(InitBeaconP*)call LTPPacket.getPayload(&NdpBeaconP,sizeof(InitBeaconP));
			pload->type=INITBEACON;
			pload->count=count;
			memcpy(pload->para,call NodeWakeup.getPara(),sizeof(NodeWorkPara));
            FillNeighborSet_withoutTime(pload->set,INITSETSIZE);
		}else{
		//如果节点占空比状态
		   if(count==0){  //预约点
			RendezBeaconP *pload=(RendezBeaconP*)call LTPPacket.getPayload(&NdpBeaconP,sizeof(RendezBeaconP));//这里极有可能超过空间,需要算一算
			pload->type=RENDEZBEACON;
			pload->count=count;
			memcpy(pload->para,call NodeWakeup.getPara(),sizeof(NodeWorkPara));
            FillNeighborSet_withTime(pload->set,SETSIZE*sizeof(struct Neighbor));
	    	}else{
			BaseBeaconP *pload=(BaseBeaconP*)call LTPPacket.getPayload(&NdpBeaconP,sizeof(BaseBeaconP));
			pload->type=BASEBEACON;
			pload->count=count;
			memcpy(pload->para,call NodeWakeup.getPara(),sizeof(NodeWorkPara));
	    	}
		}
		call NetBeaconPiggyback.set(&NdpBeaconP,sizeof(NodeWorkPara),NDP_ID);
	}
 
    //---------------------------------------------------------------------------------------------
	void insertNode(list_t list,void *nnode){
		struct sche_tab_t *te=list_head(list); 
		struct sche_tab_t *node=(struct sche_tab_t*)nnode;     
		if(!te) list_add(list,node);
		else{
			if(te->check_time > node->check_time) {
				list_push(list,node);
			}else{
				while(te->next!=NULL){
					if( ((struct sche_tab_t *)(te->next))->check_time > node->check_time){
						list_insert(list,te,node);
						break;
					}
					te=list_item_next(te);
				}
				if(te->next==NULL) te->next=(void*)node;
			}
		}
	}

    //--------------------------链表操作-------------------------------------------
	void list_init(list_t list){*list = NULL;}
	void *list_head(list_t list){return *list;}
	void *list_tail(list_t list){
		struct list *l;
		if(*list == NULL) {
			return NULL;
		}
		for(l = *list; l->next != NULL; l = l->next);
		return l;
	}

	void list_add(list_t list, void *item){
		struct list *l;
		list_remove(list, item);
		((struct list *)item)->next = NULL;
		l = list_tail(list);
		if(l == NULL) {
			*list = item;
		} else {
			l->next = item;
		}
	}

	void list_push(list_t list, void *item){
		((struct list *)item)->next = *list;
		*list = item;
	}

	void list_insert(list_t list, void *previtem, void *newitem){
		if(previtem == NULL) {
			list_push(list, newitem);
		} else {
			((struct list *)newitem)->next = ((struct list *)previtem)->next;
			((struct list *)previtem)->next = newitem;
		}
	}

	void *list_item_next(void *item){
		return item == NULL? NULL: ((struct list *)item)->next;
	}

	void list_remove(list_t list, void *item){
		struct list *l, *r;
		if(*list == NULL) {
			return;
		}
		r = NULL;
		for(l = *list; l != NULL; l = l->next) {
			if(l == item) {
				if(r == NULL) {
					*list = l->next;
				} else {
					r->next = l->next;
				}
				l->next = NULL;
				return;
			}
			r = l;
		}
	}

//以下为链路质量计算办法------------------------------------------------------------------------
  void updateLinkqEntryIdx(uint8_t idx, uint8_t loss) {
	   uint8_t i, totalPkt;
        neighbor_table_entry_t *ne;
       uint8_t newEst;
       uint8_t minPkt;

         minPkt = BLQ_PKT_WINDOW;
         if (NeighborTable[idx].flags & INIT_ENTRY) {
			 NeighborTable[idx].flags &= ~INIT_ENTRY;
		 }

		 NeighborTable[idx].inage = MAX_AGE;

		 if(loss)
		    NeighborTable[idx].failcnt++
		 else
		    NeighborTable[idx].rcvcnt++;
		
		if (NeighborTable[idx].rcvcnt >= BLQ_PKT_WINDOW){
			ne = &NeighborTable[i];
			if (ne->flags & VALID_ENTRY) {
	           if (ne->inage > 0)
	             ne->inage--;
	          if (ne->outage > 0)
	             ne->outage--;
	  
	         if ((ne->inage == 0) && (ne->outage == 0)) {
	                ne->flags ^= VALID_ENTRY;
	                ne->inquality = ne->outquality = 0;
	         } else {
	           ne->flags |= MATURE_ENTRY;
	           totalPkt = ne->rcvcnt + ne->failcnt;
	          if (totalPkt < minPkt) {
	              totalPkt = minPkt;
	         }
	         if (totalPkt == 0) {
	             ne->inquality = (ALPHA * ne->inquality) / 10;
	         } else {
	             newEst = (255 * ne->rcvcnt) / totalPkt;
	             ne->inquality = (ALPHA * ne->inquality + (10-ALPHA) * newEst + 5)/10;
	         }
	         ne->rcvcnt = 0;
	         ne->failcnt = 0;
	      }
	      updateEETX(ne, computeBidirEETX(ne->inquality, ne->outquality));
	   }
	}	    
  } 
	uint8_t findIdx(am_addr_t ll_addr) {
    uint8_t i;
    for (i = 0; i < NEIGHBOR_TABLE_SIZE; i++) {
      if (NeighborTable[i].flags & VALID_ENTRY) {
	if (NeighborTable[i].ll_addr == ll_addr) {
	  return i;
	}
      }
    }
    return INVALID_RVAL;
  }  
  //----------------------------------------------------------------------------
  void updateReverseQuality(am_addr_t neighbor, uint8_t outquality) {
    uint8_t idx;
    idx = findIdx(neighbor);
    if (idx != INVALID_RVAL) {
      NeighborTable[idx].outquality = outquality;
      NeighborTable[idx].outage = MAX_AGE;
    }
  }

  void updateEETX(neighbor_table_entry_t *ne, uint16_t newEst) {
    ne->eetx = (ALPHA * ne->eetx + (10 - ALPHA) * newEst + 5)/10;
  }

  //----------------------------------------------------------------------------
  void updateDEETX(neighbor_table_entry_t *ne) {
    uint16_t estETX;

    if (ne->data_success == 0) {
      estETX = (ne->data_total - 1)* 10;
    } else {
      estETX = (10 * ne->data_total) / ne->data_success - 10;
      ne->data_success = 0;
      ne->data_total = 0;
    }
    updateEETX(ne, estETX);
  }
//-----------------------------------------------------------------------------
 uint8_t computeEETX(uint8_t q1) {
    uint16_t q;
    if (q1 > 0) {
      q =  2550 / q1 - 10;
      if (q > 255) {
	q = VERY_LARGE_EETX_VALUE;
      }
      return (uint8_t)q;
    } else {
      return VERY_LARGE_EETX_VALUE;
    }
  }
  //----------------------------------------------------------------------------
  uint8_t computeBidirEETX(uint8_t q1, uint8_t q2) {
    uint16_t q;
    if ((q1 > 0) && (q2 > 0)) {
      q =  65025u / q1;
      q = (10*q) / q2 - 10;
      if (q > 255) {
	q = LARGE_EETX_VALUE;
      }
      return (uint8_t)q;
    } else {
      return LARGE_EETX_VALUE;
    }
  }

	// return bi-directional link quality to the neighbor
	command uint16_t LinkEstimator.getLinkQuality(am_addr_t neighbor){
		uint8_t idx;
        idx = findIdx(neighbor);
       if (idx == INVALID_RVAL) {
         return VERY_LARGE_EETX_VALUE;
      } else {
      if (NeighborTable[idx].flags & MATURE_ENTRY) {
	//	dbg("node","LinkEstimator.getLinkQuality:%u\r\n",NeighborTable[idx].eetx);
     	return NeighborTable[idx].eetx;
      } else {
	   return VERY_LARGE_EETX_VALUE;
       }
     }

	}
	// return the quality of the link: self->neighbor
    command uint16_t LinkEstimator.getForwardQuality(am_addr_t neighbor) {
		uint8_t idx;
        idx = findIdx(neighbor);
       if (idx == INVALID_RVAL) {
         return VERY_LARGE_EETX_VALUE;
      } else {
      if (NeighborTable[idx].flags & MATURE_ENTRY) {
		//dbg("node","qua of node %hu:%hhu\r\n",neighbor,computeEETX(NeighborTable[idx].inquality));
     	return computeEETX(NeighborTable[idx].inquality);
      } else {
	   return VERY_LARGE_EETX_VALUE;
       }
     }
    }
	// return the quality of the link: neighor->self
    command uint16_t LinkEstimator.getReverseQuality(am_addr_t neighbor) {
       uint8_t idx;
      idx = findIdx(neighbor);
    if (idx == INVALID_RVAL) {
      return VERY_LARGE_EETX_VALUE;
    } else {
      if (NeighborTable[idx].flags & MATURE_ENTRY) {
	    return computeEETX(NeighborTable[idx].outquality);
      } else {
	  return VERY_LARGE_EETX_VALUE;
       }
     }
    }
	// insert the neighbor at any cost (if there is a room for it)
    // even if eviction of a perfectly fine neighbor is called for
    command error_t LinkEstimator.insertNeighbor(am_addr_t neighbor) {
         return FAIL;//不允许添加节点
    }
	 // pin a neighbor so that it does not get evicted
    command error_t LinkEstimator.pinNeighbor(am_addr_t neighbor) {
        uint8_t nidx = findIdx(neighbor);
       if (nidx == INVALID_RVAL) {
         return FAIL;
      }
       NeighborTable[nidx].flags |= PINNED_ENTRY;
      return SUCCESS;
	}
	// pin a neighbor so that it does not get evicted
   command error_t LinkEstimator.unpinNeighbor(am_addr_t neighbor) {
	   uint8_t nidx = findIdx(neighbor);
      if (nidx == INVALID_RVAL) {
      return FAIL;
     }
     NeighborTable[nidx].flags &= ~PINNED_ENTRY;
     return SUCCESS;
   }
    // called when an acknowledgement is received; sign of a successful
   // data transmission; to update forward link quality
   command error_t LinkEstimator.txAck(am_addr_t neighbor) {
     neighbor_table_entry_t *ne;
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }
    ne = &NeighborTable[nidx];
    ne->data_success++;
    ne->data_total++;
    if (ne->data_total >= DLQ_PKT_WINDOW) {
      updateDEETX(ne);
    }
    return SUCCESS;
   }
   // called when an acknowledgement is not received; could be due to
  // data pkt or acknowledgement loss; to update forward link quality
  command error_t LinkEstimator.txNoAck(am_addr_t neighbor){
	neighbor_table_entry_t *ne;
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }

    ne = &NeighborTable[nidx];
    ne->data_total++;
    if (ne->data_total >= DLQ_PKT_WINDOW) {
      updateDEETX(ne);
    }
    return SUCCESS;
  }
  // called when the parent changes; clear state about data-driven link quality
  command error_t LinkEstimator.clearDLQ(am_addr_t neighbor) {
	neighbor_table_entry_t *ne;
    uint8_t nidx = findIdx(neighbor);
    if (nidx == INVALID_RVAL) {
      return FAIL;
    }
    ne = &NeighborTable[nidx];
    ne->data_total = 0;
    ne->data_success = 0;
    return SUCCESS;
  }
  //节点被删除或者被替换了
  signal LinkEstimator.evicted(NeighborTable[nidx].ll_addr);
}