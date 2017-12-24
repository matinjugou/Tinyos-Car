module CarP {
    provides {
        interface Car;
    }

    uses {
    	interface HplMsp430Usart;
    	interface HplMsp430UsartInterrupts;
    	interface HplMsp430GeneralIO;
    	interface Resource;
    	interface Leds;
	}
}

implementation {
	msp430_uart_union_config_t uart_config = {
		{
			utxe: 1,
			urxe: 1,
			ubr: UBR_1MHZ_115200,
			umctl: UMCTL_1MHZ_115200,
			ssel: 0x02,
			pena: 0,
			pev: 0,
			spb: 0,
			clen: 1,
			listen: 0,
			mm: 0,
			ckpl: 0,
			urxse: 0,
			urxeie: 0,
			urxwie: 0,
			utxe: 1,
			urxe: 1
		}
	};
	uint8_t next_op_type;
	uint16_t next_op_data;
	uint16_t maxspeed, minspeed;

	uint16_t step;

	void sendOp() {
		if (step == 0) {
			call HplMsp430Usart.tx(0x01);
		}
		else if (step == 1) {
			call HplMsp430Usart.tx(0x02);
		}
		else if (step == 2) {
			call HplMsp430Usart.tx(next_op_type);
			//call HplMsp430Usart.tx(0x02);
		}
		else if (step == 3) {
			call HplMsp430Usart.tx(next_op_data / 256);
		}
		else if (step == 4) {
			call HplMsp430Usart.tx(next_op_data % 256);
		}
		else if (step == 5) {
			call HplMsp430Usart.tx(0xFF);
		}
		else if (step == 6) {
			call HplMsp430Usart.tx(0xFF);
		}
		else if (step == 7) {
			call HplMsp430Usart.tx(0x00);
		}
		while (!call HplMsp430Usart.isTxEmpty()) {}
		step++;
		if (step != 8) {
			sendOp();
		}
		else {
			call Leds.led0Toggle();
			call Resource.release();
			signal Car.operationDone(next_op_type);
		}
	}

	event void Resource.granted() {
		call HplMsp430Usart.setModeUart(&uart_config);
		call HplMsp430Usart.enableUart();
		U0CTL &= ~SYNC;
		step = 0;
		sendOp();
	}

	async event void HplMsp430UsartInterrupts.rxDone(uint8_t data) {}
	async event void HplMsp430UsartInterrupts.txDone() {}

    command void Car.start() {
        return;
    }

	command error_t Car.Angle(uint16_t value) {
		next_op_type = 1;
		next_op_data = value;
		call Resource.request();
	}

	command error_t Car.Angle_Second(uint16_t value) {
		next_op_type = 7;
		next_op_data = value;
		call Resource.request();
	}

	command error_t Car.Angle_Third(uint16_t value) {
		next_op_type = 8;
		next_op_data = value;
		call Resource.request();
	}

	command error_t Car.Forward(uint16_t value) {
		next_op_type = 2;
		next_op_data = value;
		if (value > maxspeed) {
			value = maxspeed;
		}
		if (value < minspeed) {
			value = minspeed;
		}
		call Resource.request();
	}

	command error_t Car.Back(uint16_t value) {
		next_op_type = 3;
		next_op_data = value;
		if (value > maxspeed) {
			value = maxspeed;
		}
		if (value < minspeed) {
			value = minspeed;
		}
		call Resource.request();
	}

	command error_t Car.Left(uint16_t value) {
		next_op_type = 4;
		next_op_data = value;
		if (value > maxspeed) {
			value = maxspeed;
		}
		if (value < minspeed) {
			value = minspeed;
		}
		call Resource.request();
	}

	command error_t Car.Right(uint16_t value) {
		next_op_type = 5;
		next_op_data = value;
		if (value > maxspeed) {
			value = maxspeed;
		}
		if (value < minspeed) {
			value = minspeed;
		}
		call Resource.request();
	}

	command error_t Car.Pause() {
		next_op_type = 6;
		next_op_data = 0x00;
		call Resource.request();
	}

	command error_t Car.InitMaxSpeed(uint16_t value) {
		maxspeed = value;
		next_op_type = 0xc;
		next_op_data = maxspeed;
		call Resource.request();
	}

	command error_t Car.InitMinSpeed(uint16_t value) {
		minspeed = value;
		next_op_type = 0xd;
		next_op_data = minspeed;
		call Resource.request();
	}   
}