#include "TankMessage.h"

#define MAXSPEED 3000
#define MINSPEED 500
#define MAXANGLE 4500
#define MINANGLE 500

module RemoteC {
	uses interface Boot;
	uses interface Leds;

	uses interface Timer<TMilli> as checkTimer;

	uses interface Button;
	uses interface Read<uint16_t> as adcReadX;
	uses interface Read<uint16_t> as adcReadY;

	uses interface Packet as Packet;
	uses interface AMSend as AMSend;
	uses interface Receive as Receive;

	uses interface Packet as SPacket;
	uses interface AMSend as SAMSend;

	uses interface SplitControl as RadioControl;
	uses interface SplitControl as SerialControl;

}

implementation {
	bool pinA;
	bool pinB;
	bool pinC;
	bool pinD;
	bool pinE;
	bool pinF;
	uint16_t Xvalue;
	uint16_t Yvalue;

	bool ADone;
	bool BDone;
	bool CDone;
	bool DDone;
	bool EDone;
	bool FDone;
	bool XDone;
	bool YDone;

	bool busy;
	bool Sbusy;
	message_t pkt;
	message_t spkt;

	bool stopped;
	uint16_t currentSpeed;
	uint16_t initPos1;
	uint16_t initPos2;
	uint16_t initPos3;
	uint16_t stepAngle;

	uint8_t action_type;
	uint16_t action_data;

	void send_command() {
		TankMsg* sndPck;
		call Leds.led1Toggle();
		sndPck = (TankMsg*)(call Packet.getPayload(&pkt, sizeof(TankMsg)));
		sndPck->nodeid = TOS_NODE_ID;
		sndPck->type = 0;
		sndPck->action = action_type;
		sndPck->data = action_data;
		if ((call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(TankMsg))) == SUCCESS) {
			busy = TRUE;
		}
		/*
		DebugMsg* ssndPck;
		ssndPck = (DebugMsg*)(call Packet.getPayload(&pkt, sizeof(DebugMsg)));
		ssndPck->pinA = pinA;
		ssndPck->pinB = pinB;
		ssndPck->pinC = pinC;
		ssndPck->pinD = pinD;
		ssndPck->pinE = Xvalue;
		ssndPck->pinF = Yvalue;
		if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(DebugMsg)) == SUCCESS) {
			busy = TRUE;
		}
		*/
	}

	void make_operation() {
		if ((!pinA) || (!pinB) || (!pinC) 
			|| (!pinE) || (!pinF)
			|| (Xvalue == 0xfff) || (Xvalue == 0x000)
			|| (Yvalue == 0xfff) || (Yvalue == 0x000)) {
			call Leds.led2Toggle();
			//action_type = 2;
			//action_data = 500;

			//Angle1 add angle
			if ((pinA) && (!pinB) && (pinC) 
				&& (pinE) && (pinF)
				&& (Xvalue == 0x000)) {
				if (initPos1 + stepAngle < MAXANGLE) {
					initPos1 += stepAngle;
				}
				action_type = 1;
				action_data = initPos1;
				if (!busy) {
					send_command();
				}
			}
			else if ((pinA) && (!pinB) && (pinC) // Angle1 minus angle 
				&& (pinE) && (pinF)
				&& (Xvalue == 0xfff)) {
				if (initPos1 - stepAngle > MINANGLE) {
					initPos1 -= stepAngle;
				}
				action_type = 1;
				action_data = initPos1;
				if (!busy) {
					send_command();
				}
			}
			else if ((pinA) && (pinB) && (pinC) // Angle2 add angle 
				&& (pinE) && (!pinF)
				&& (Xvalue == 0x000)) {
				if (initPos2 + stepAngle < MAXANGLE) {
					initPos2 += stepAngle;
				}
				action_type = 7;
				action_data = initPos2;
				if (!busy) {
					send_command();
				}
			}
			else if ((pinA) && (pinB) && (pinC) // Angle2 minus angle 
				&& (pinE) && (!pinF)
				&& (Xvalue == 0xfff)) {
				if (initPos2 - stepAngle > MINANGLE) {
					initPos2 -= stepAngle;
				}
				action_type = 7;
				action_data = initPos2;
				if (!busy) {
					send_command();
				}
			}
			else if ((pinA) && (!pinB) && (pinC) // Angle3 add angle 
				&& (pinE) && (pinF)
				&& (Xvalue == 0x000)) {
				if (initPos3 + stepAngle < MAXANGLE) {
					initPos3 += stepAngle;
				}
				action_type = 8;
				action_data = initPos3;
				if (!busy) {
					send_command();
				}
			}
			else if ((pinA) && (!pinB) && (pinC) // Angle3 minus angle 
				&& (pinF) && (pinE)
				&& (Xvalue == 0xfff)) {
				if (initPos3 - stepAngle > MINANGLE) {
					initPos3 -= stepAngle;
				}
				action_type = 8;
				action_data = initPos3;
				if (!busy) {
					send_command();
				}
			}
			else if ((pinB) && (pinF)) {
				// accelerate
				if ((!pinA) && (pinC)) {
					if (currentSpeed + 10 < MAXSPEED) {
						currentSpeed += 10;
					}
				}
				else if ((pinA) && (!pinE)) {
					if (currentSpeed - 10 > MINSPEED) {
						currentSpeed -= 10;
					}
				}
				if ((Yvalue == 0x000) && (Xvalue != 0x000) 
					&& (Xvalue != 0xfff)) {
					action_type = 4;
					action_data = currentSpeed;
					if (!busy) {
						send_command();
					}
				}
				else if ((Yvalue == 0xfff) && (Xvalue != 0x000) 
					&& (Xvalue != 0xfff)) {
					action_type = 5;
					action_data = currentSpeed;
					if (!busy) {
						send_command();
					}
				}
				else if ((Xvalue == 0x000) && (Yvalue != 0x000) 
					&& (Yvalue != 0xfff)) {
					action_type = 2;
					action_data = currentSpeed;
					if (!busy) {
						send_command();
					}
				}
				else if ((Xvalue == 0xfff) && (Yvalue != 0x000) 
					&& (Yvalue != 0xfff)) {
					action_type = 3;
					action_data = currentSpeed;
					if (!busy) {
						send_command();
					}
				}
			}
		}
		else {
			if (!stopped) {
				action_type = 6;
				if (!busy) {
					send_command();
				}
			}
		}
	}

	event void AMSend.sendDone(message_t* m, error_t err) {
		busy = FALSE;
		call Leds.led1Toggle();
	}

	event void SAMSend.sendDone(message_t* m, error_t err) {
		Sbusy = FALSE;
	}


	event void Boot.booted() {
		stopped = TRUE;
		currentSpeed = 500;
		initPos1 = 3000;
		initPos2 = 3000;
		initPos3 = 3000;
		stepAngle = 50;
		call Button.start();
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
		else {
			call checkTimer.startPeriodic(30);
		}
	}

	event void SerialControl.stopDone(error_t err) {}
	
	event void checkTimer.fired() {
		ADone = FALSE;
		BDone = FALSE;
		CDone = FALSE;
		DDone = FALSE;
		EDone = FALSE;
		FDone = FALSE;
		XDone = FALSE;
		YDone = FALSE;
		call Button.pinvalueA();
		call Button.pinvalueB();
		call Button.pinvalueC();
		call Button.pinvalueD();
		call Button.pinvalueE();
		call Button.pinvalueF();
		call adcReadX.read();
		call adcReadY.read();
	}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		TankMsg* rcvPck;
		rcvPck = (TankMsg*)payload;
		if ((rcvPck->nodeid != TOS_NODE_ID) && (rcvPck->type == 1)) {
			if (rcvPck->action == 6) {
				stopped = TRUE;
			}
		}
		return msg;
	}

	event void Button.startDone(bool value) {

	}

	event void Button.stopDone(bool value) {

	}

	event void Button.pinvalueADone(bool value) {
		pinA = value;
		ADone = TRUE;
		if (ADone && BDone && CDone 
			&& DDone && EDone && FDone
			&& XDone && YDone) {
			make_operation();
		}
	}

	event void Button.pinvalueBDone(bool value) {
		pinB = value;
		BDone = TRUE;
		if (ADone && BDone && CDone 
			&& DDone && EDone && FDone
			&& XDone && YDone) {
			make_operation();
		}
	}

	event void Button.pinvalueCDone(bool value) {
		pinC = value;
		CDone = TRUE;
		if (ADone && BDone && CDone 
			&& DDone && EDone && FDone
			&& XDone && YDone) {
			make_operation();
		}
	}

	event void Button.pinvalueDDone(bool value) {
		pinD = value;
		DDone = TRUE;
		if (ADone && BDone && CDone 
			&& DDone && EDone && FDone
			&& XDone && YDone) {
			make_operation();
		}
	}

	event void Button.pinvalueEDone(bool value) {
		pinE = value;
		EDone = TRUE;
		if (ADone && BDone && CDone 
			&& DDone && EDone && FDone
			&& XDone && YDone) {
			make_operation();
		}
	}

	event void Button.pinvalueFDone(bool value) {
		pinF = value;
		FDone = TRUE;
		if (ADone && BDone && CDone 
			&& DDone && EDone && FDone
			&& XDone && YDone) {
			make_operation();
		}
	}

	event void adcReadX.readDone(error_t result, uint16_t val){
        if(result == SUCCESS){
            Xvalue = val;
            XDone = TRUE;
			if (ADone && BDone && CDone 
				&& DDone && EDone && FDone
				&& XDone && YDone) {
				make_operation();
			}
        }
        else{
            Xvalue = 0xfff;
        }
    }

	event void adcReadY.readDone(error_t result, uint16_t val){
        if(result == SUCCESS){
            Yvalue = val;
        	YDone = TRUE;
			if (ADone && BDone && CDone 
				&& DDone && EDone && FDone
				&& XDone && YDone) {
				make_operation();
			}
        }
        else{
            Yvalue = 0xfff;
        }
    }
}