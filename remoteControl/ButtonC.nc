configuration ButtonC {
	provides interface Button;
}
implementation {
	components ButtonP;
	components HplMsp430GeneralIOC;
	Button = ButtonP;

	ButtonP.PortA -> HplMsp430GeneralIOC.Port60;
	ButtonP.PortB -> HplMsp430GeneralIOC.Port21;
	ButtonP.PortC -> HplMsp430GeneralIOC.Port61;
	ButtonP.PortD -> HplMsp430GeneralIOC.Port23;
	ButtonP.PortE -> HplMsp430GeneralIOC.Port62;
	ButtonP.PortF -> HplMsp430GeneralIOC.Port26;
}