configuration JoyStickC {
	provides {
		interface Read<uint16_t> as ReadX;
		interface Read<uint16_t> as ReadY;
	}
}
implementation {
	components JoyStickP;
	components new AdcReadClientC() as AdcClientX;
	components new AdcReadClientC() as AdcClientY;
	ReadX = AdcClientX;
	ReadY = AdcClientY;
	AdcClientX.AdcConfigure -> JoyStickP.AdcX;
	AdcClientY.AdcConfigure -> JoyStickP.AdcY;

}