#include "TankMessage.h"

#define MAX_NODE_NUM 256

module ReceiverC {
	uses interface Boot;
	uses interface Leds;
	uses interface Packet as Packet;
	uses interface AMSend as AMSend;
	uses interface Packet as SPacket;
	uses interface AMSend as SAMSend;
	uses interface Receive;
	uses interface SplitControl as RadioControl;
	uses interface SplitControl as SerialControl;
}

implementation {
	bool Sbusy;
	message_t pkt;
	message_t spkt;
	uint32_t i;

	event void Boot.booted() {
		Sbusy = FALSE;
		call RadioControl.start();
		call SerialControl.start();
	}

	event void RadioControl.startDone(error_t err) {
		if (err != SUCCESS) {
			call RadioControl.start();
		}
	}

	event void RadioControl.stopDone(error_t err) {}

	event void SerialControl.startDone(error_t err) {
		if (err != SUCCESS) {
			call SerialControl.start();
		}
	}

	event void SerialControl.stopDone(error_t err) {}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		TankMsg* rcvPayload;
		TankMsg* ssndPck;

		call Leds.led0Toggle();

		rcvPayload = (TankMsg*)payload;
		ssndPck = (TankMsg*)(call SPacket.getPayload(&spkt, sizeof(TankMsg)));
		ssndPck->nodeid = rcvPayload->nodeid;
		ssndPck->type = rcvPayload->type;
		ssndPck->action = rcvPayload->action;
		ssndPck->data = rcvPayload->data;
		if (call SAMSend.send(AM_BROADCAST_ADDR, &spkt, sizeof(TankMsg)) == SUCCESS) {
			Sbusy = TRUE;
		}

		call Leds.led0Toggle();
		return msg;
	}

	event void SAMSend.sendDone(message_t* msg, error_t err) {
		Sbusy = FALSE;
	}

	event void AMSend.sendDone(message_t* msg, error_t err) {
		
	}
}