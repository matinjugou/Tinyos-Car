configuration RemoteAppC {}
implementation {
	components MainC, ButtonC, LedsC, JoyStickC;
	components RemoteC as App;
	components ActiveMessageC;
	components SerialActiveMessageC;
	components new TimerMilliC() as checkTimer;

	App.Boot -> MainC;
	App.Leds -> LedsC;	
	App.Button -> ButtonC.Button;
	App.adcReadX -> JoyStickC.ReadX;
	App.adcReadY -> JoyStickC.ReadY;
	App.checkTimer -> checkTimer;

	App.Packet -> ActiveMessageC;
	App.AMSend -> ActiveMessageC.AMSend[AM_TANKMSG];
	App.Receive -> ActiveMessageC.Receive[AM_TANKMSG];

	App.SPacket -> SerialActiveMessageC;
	App.SAMSend -> SerialActiveMessageC.AMSend[AM_DEBUGMSG];

	App.RadioControl -> ActiveMessageC;
	App.SerialControl -> SerialActiveMessageC;
}