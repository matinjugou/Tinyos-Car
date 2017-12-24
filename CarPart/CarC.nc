configuration CarC {
	provides interface Car;
}
implementation {
	components CarP;
	components HplMsp430Usart0C;
	components new Msp430Uart0C();
	components HplMsp430GeneralIOC;
	components LedsC;
	Car = CarP.Car;

	CarP.Leds -> LedsC;
	CarP.Resource -> Msp430Uart0C;
	CarP.HplMsp430Usart -> HplMsp430Usart0C;
	CarP.HplMsp430UsartInterrupts -> HplMsp430Usart0C;
	CarP.HplMsp430GeneralIO -> HplMsp430GeneralIOC.Port20;
}