configuration AMWatcherAppC {}
implementation {
	components MainC, LedsC;
	components AMWatcherC as App;
	components ActiveMessageC;
	components SerialActiveMessageC;

	App.Boot -> MainC.Boot;
	App.Leds -> LedsC.Leds;
	App.Packet -> ActiveMessageC;
	App.AMSend -> ActiveMessageC.AMSend[AM_TANKMSG];
	App.Receive -> ActiveMessageC.Receive[AM_TANKMSG];
	App.SPacket -> SerialActiveMessageC;
	App.SAMSend -> SerialActiveMessageC.AMSend[AM_DEBUGMSG];
	App.RadioControl -> ActiveMessageC;
	App.SerialControl -> SerialActiveMessageC;
}