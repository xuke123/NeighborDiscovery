configuration NetworkDispatherC{
    provides interface Receive;
    provides interface Snoop;
	uses interface Receice as SubSnoop[am_id_t id];
    uses interface Receive as SubReceive[am_id_t id];
}
implementation{
   components NetworkDispatherP as DispatherP;

   DispatherP.SubSnoop=SubSnoop;
   DispatherP.SubReceive=SubReceive;

   Receive=DispatherP;
   Snoop=DispatherP;
}