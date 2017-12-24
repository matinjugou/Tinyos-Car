module ButtonP {
    provides {
        interface Button;
    }

    uses {
    	interface HplMsp430GeneralIO as PortA;
    	interface HplMsp430GeneralIO as PortB;
    	interface HplMsp430GeneralIO as PortC;
    	interface HplMsp430GeneralIO as PortD;
    	interface HplMsp430GeneralIO as PortE;
    	interface HplMsp430GeneralIO as PortF;
	}
}

implementation {
	bool pinA;
	bool pinB;
	bool pinC;
	bool pinD;
	bool pinE;
	bool pinF;

	error_t err;

	command void Button.start() {
		call PortA.clr();
		call PortA.makeInput();
		call PortB.clr();
		call PortB.makeInput();
		call PortC.clr();
		call PortC.makeInput();
		call PortD.clr();
		call PortD.makeInput();
		call PortE.clr();
		call PortE.makeInput();
		call PortF.clr();
		call PortF.makeInput();
		signal Button.startDone(SUCCESS);
	}
	
	command void Button.stop() {
		signal Button.startDone(SUCCESS);
	}
	
	command void Button.pinvalueA() {
		pinA = call PortA.get();
		signal Button.pinvalueADone(pinA);
	}

	command void Button.pinvalueB() {
		pinB = call PortB.get();
		signal Button.pinvalueBDone(pinB);
	}

	command void Button.pinvalueC() {
		pinC = call PortC.get();
		signal Button.pinvalueCDone(pinC);
	}

	command void Button.pinvalueD() {
		pinD = call PortD.get();
		signal Button.pinvalueDDone(pinD);
	}

	command void Button.pinvalueE() {
		pinE = call PortE.get();
		signal Button.pinvalueEDone(pinE);
	}

	command void Button.pinvalueF() {
		pinF = call PortF.get();
		signal Button.pinvalueFDone(pinF);
	}
}