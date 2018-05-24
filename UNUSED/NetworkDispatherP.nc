module NetworkDispatherC{
    provides interface Receive;
    provides interface Snoop;
	uses interface Receice as SubSnoop;
    uses interface Receive as SubReceive;
}
implementation{
    event message_t * SubReceive.receive(message_t * message, void * payload, uint8_t length)
	{
		signal Receice..receive(message, payload, length);
		return message;
	}
    event message_t * SubSnoop.receive(message_t * message, void * payload, uint8_t length)
	{
		signal Snoop.receive(message, payload, length);
		return message;
	}
}