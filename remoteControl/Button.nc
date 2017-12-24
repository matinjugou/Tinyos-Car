interface Button {
	command void start();
	event void startDone(error_t err);

	command void stop();
	event void stopDone(error_t err);
	
	command void pinvalueA();
	event void pinvalueADone(bool value);

	command void pinvalueB();
	event void pinvalueBDone(bool value);

	command void pinvalueC();
	event void pinvalueCDone(bool value);

	command void pinvalueD();
	event void pinvalueDDone(bool value);

	command void pinvalueE();
	event void pinvalueEDone(bool value);

	command void pinvalueF();
	event void pinvalueFDone(bool value);
}
