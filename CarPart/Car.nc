interface Car {
	command void start();
	command error_t Angle(uint16_t value);
	command error_t Angle_Second(uint16_t value);
	command error_t Angle_Third(uint16_t value);
	command error_t Forward(uint16_t value);
	command error_t Back(uint16_t value);
	command error_t Left(uint16_t value);
	command error_t Right(uint16_t value);
	command error_t Pause();
	command error_t InitMaxSpeed(uint16_t value);
	command error_t InitMinSpeed(uint16_t value);
	command error_t InitAngle(uint16_t value);
	event error_t InitAngleDone();
	command error_t InitAngle_Second(uint16_t value);
	event error_t InitAngle_SecondDone();
	command error_t InitAngle_Third(uint16_t value);

	event void operationDone(uint8_t type);
}
