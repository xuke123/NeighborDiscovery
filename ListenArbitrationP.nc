//这个组件没用，直接连到了底层
// module ListenArbitrationP {
// 	provides interface ListenRemote;
//     uses interface SubListenRemote;
// }

// implementation {
//     enum {
//         LISTEN_ON = 0,
//         LISTEN_OFF = 1,
//     };
//     uint8_t startCnt = 0;
//     uint8_t listenState = LISTEN_OFF;
//     command error_t ListenRemote.startListen() {
//         uint32_t wakeupTime1 = (call WakeupTime.getPseudoPara())->nextWakeupTime;
// 		uint32_t wakeupTime2 = wakeupTime1 - (call WakeupTime.getPseudoPara())->randomState;
//         uint32_t nowTime = call LocalTime.get();
//         if ((wakeupTime1 > (nowTime - 10) && wakeupTime1 < (nowTime + 10)) || 
// 			               (wakeupTime2 > (nowTime - 10) && wakeupTime2 < (nowTime + 10))) {
//             return FAIL;
//         }

//         if (ListenState == LISTEN_OFF) {
//             if (call SubListenRemote.start() == SUCCESS) {
//                 start_cnt++;
//                 listenState = LISTEN_ON;
//             }
//             else {
//                 return FAIL;//返回失败
//             }
//         } else {
//             startCnt++;
//         }
//         return SUCCESS;//返回成功
//     }

//     event error_t SubListenRemote.startDone() {
//         signal ListenRemote.startDone();
//     }

//     command error_t ListenRemote.stopListen() {
//         if (startCnt == 0) {
//             if (call SubListenRemote.stopListen() == SUCCESS) {
//                  listenState = LISTEN_OFF; 
//                 return SUCCESS;
//             } else {
//                 return FAIL;
//             }
//         } else {
//             startCnt--;
//             if (startCnt == 0) {
//                 if (call SubListenRemote.stopListen() == SUCCESS) {//这里关闭不了，会有什么后果
//                   ListenState = LISTEN_OFF;
//                   return SUCCESS;
//                } else {
//                   return FAIL;
//                }
//             }
//             return SUCCESS;
//         }  
//     }

//    event error_t SubListenRemote.stopDone() {
//         signal ListenRemote.stopDone();
//     }
// }