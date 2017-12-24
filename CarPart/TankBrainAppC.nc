configuration TankBrainAppC {}
implementation {
	components MainC, CarC, LedsC;
	components TankBrainC as App;
	components ActiveMessageC;
	components new TimerMilliC() as clockTimer;
	components SerialActiveMessageC;

	App.Boot -> MainC;
	App.Leds -> LedsC;	
	App.Car -> CarC.Car;

	App.clockTimer -> clockTimer;

	App.Packet -> ActiveMessageC;
	App.AMSend -> ActiveMessageC.AMSend[AM_TANKMSG];
	App.Receive -> ActiveMessageC.Receive[AM_TANKMSG];

	App.SPacket -> SerialActiveMessageC;
	App.SAMSend -> SerialActiveMessageC.AMSend[AM_TANKMSG];

	App.RadioControl -> ActiveMessageC;
	App.SerialControl -> SerialActiveMessageC;
}