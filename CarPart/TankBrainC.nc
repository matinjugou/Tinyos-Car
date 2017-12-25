#include "TankMessage.h"
#include "CommandNode.h"

#define MIDANGLE 3000

module TankBrainC {
	uses interface Boot;
	uses interface Leds;

	uses interface Car; 

	uses interface Timer<TMilli> as clockTimer;

	uses interface Packet as Packet;
	uses interface AMSend as AMSend;
	uses interface Receive as Receive;

	uses interface Packet as SPacket;
	uses interface AMSend as SAMSend;

	uses interface SplitControl as RadioControl;
	uses interface SplitControl as SerialControl;

}

implementation {
	bool busy;
	message_t pkt;
	message_t spkt;

	bool excuting;
	uint16_t initStep;
	uint16_t cmdHead;
	uint16_t cmdTail;
	CommandNode CmdQueue[128];

	event void Boot.booted() {
		cmdHead = 0;
		cmdTail = 0;
		excuting = FALSE;
		initStep = 0;
		call Car.start();
		call RadioControl.start();
		call SerialControl.start();
		call Car.InitMaxSpeed(3000);
		call Car.InitMinSpeed(200);
	}

	event void clockTimer.fired() {
		if (initStep == 1)
		{
			call clockTimer.stop();
			call Car.InitAngle_Second(MIDANGLE);
		}
		else if (initStep == 2) {
			call clockTimer.stop();
			call Car.InitAngle_Third(MIDANGLE);
		}

	}

	void ExcuteCommand() {
		if (CmdQueue[cmdHead].action == 1) {
			excuting = TRUE;
			call Leds.led0On();
			call Leds.led1Off();
			call Leds.led2Off();
			call Car.Angle(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 2) {
			excuting = TRUE;
			call Leds.led0Off();
			call Leds.led1On();
			call Leds.led2Off();
			call Car.Forward(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 3) {
			excuting = TRUE;
			call Leds.led0On();
			call Leds.led1On();
			call Leds.led2Off();
			call Car.Back(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 4) {
			excuting = TRUE;
			call Leds.led0Off();
			call Leds.led1Off();
			call Leds.led2On();
			call Car.Left(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 5) {
			excuting = TRUE;
			call Leds.led0On();
			call Leds.led1Off();
			call Leds.led2On();
			call Car.Right(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 6) {
			excuting = TRUE;
			call Leds.led0Off();
			call Leds.led1On();
			call Leds.led2On();
			call Car.Pause();
		}
		else if (CmdQueue[cmdHead].action == 7) {
			excuting = TRUE;
			call Leds.led0On();
			call Leds.led1On();
			call Leds.led2On();
			call Car.Angle_Second(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 8) {
			excuting = TRUE;
			call Leds.led0On();
			call Leds.led1On();
			call Leds.led2On();
			call Car.Angle_Third(CmdQueue[cmdHead].data);
		}
		else if (CmdQueue[cmdHead].action == 9) {
			excuting = TRUE;
			call Leds.led0On();
			call Leds.led1On();
			call Leds.led2On();
			call Car.InitAngle(MIDANGLE);
		}
	}

	event error_t Car.InitAngleDone() {
		initStep = 1;
		call clockTimer.startPeriodic(100);
	}

	event error_t Car.InitAngle_SecondDone() {
		initStep = 2;
		call clockTimer.startPeriodic(100);
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
		else {
			//call clockTimer.startPeriodic(100);
		}
	}

	event void SerialControl.stopDone(error_t err) {}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		TankMsg* rcvPkt;
		//TankMsg* ssndPck;
		
		rcvPkt = (TankMsg*)payload;
		
		//ssndPck = (TankMsg*)(call SPacket.getPayload(&spkt, sizeof(TankMsg)));
		//ssndPck->nodeid = rcvPkt->nodeid;
		//ssndPck->type = rcvPkt->type;
		//ssndPck->action = rcvPkt->action;
		//ssndPck->data = rcvPkt->data;
		//call SAMSend.send(AM_BROADCAST_ADDR, &spkt, sizeof(TankMsg));
		if ((rcvPkt->nodeid != TOS_NODE_ID) && (rcvPkt->type == 0)) {
			CmdQueue[cmdTail].action = rcvPkt->action;
			CmdQueue[cmdTail].data = rcvPkt->data;
			cmdTail = (cmdTail + 1) % 128;
		}
		if (!excuting) {
			ExcuteCommand();
		}		
		return msg;
	}

	event void Car.operationDone(uint8_t type) {
		TankMsg* sndPck;
		call Leds.led0Off();
		call Leds.led1Off();
		call Leds.led2Off();
		
		sndPck = (TankMsg*)(call Packet.getPayload(&pkt, sizeof(TankMsg)));
		if (sndPck == NULL) {
			return;
		}
		sndPck->nodeid = TOS_NODE_ID;
		sndPck->type = 1;
		sndPck->action = type;
		sndPck->data = 0;
		if (!busy) {	
			if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(TankMsg)) == SUCCESS) {
				busy = TRUE;
			}
		}
		
		cmdHead = (cmdHead + 1) % 128;
		if ((cmdHead != cmdTail) && (!excuting)) {
			ExcuteCommand();
		}
		else {
			excuting = FALSE;
		}
	}

	event void AMSend.sendDone(message_t* m, error_t err) {
		busy = FALSE;
	}

	event void SAMSend.sendDone(message_t* m, error_t err) {
	}
}