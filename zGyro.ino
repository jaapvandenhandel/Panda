

void GyroHandler(uint32_t delta)
{
	if (useCMPS)
	{
		//Get the Z gyro
		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x16);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 2);
		while (Wire.available() < 2);

		gyro = int16_t(Wire.read() << 8 | Wire.read());

		//Complementary filter
		gyroSum = 0.93 * gyroSum + 0.07 * gyro;

		//roll
		Wire.beginTransmission(CMPS14_ADDRESS);
		Wire.write(0x1C);
		Wire.endTransmission();

		Wire.requestFrom(CMPS14_ADDRESS, 2);
		while (Wire.available() < 2);

		roll = int16_t(Wire.read() << 8 | Wire.read());

		//Complementary filter
		rollSum = 0.9 * rollSum + 0.1 * roll;
	}
	else if (useBNO08x)
    {
        if (bno08x.dataAvailable() == true)
        {
            gyro = (bno08x.getGyroZ()) * RAD_TO_DEG; // Get raw yaw rate
            gyro = gyro * -10;

            bno08xHeading = (bno08x.getYaw()) * RAD_TO_DEG; // Convert yaw / heading to degrees
            bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data

            if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180�;180�] to [0;360�]
            {
                bno08xHeading = bno08xHeading + 360;
            }

            roll = (bno08x.getRoll()) * RAD_TO_DEG;
            pitch = (bno08x.getPitch()) * RAD_TO_DEG;
            pitch = pitch * -1;

            roll = roll * 10;
            pitch = pitch * 10;
            bno08xHeading10x = (int16_t)(bno08xHeading * 10);

            //Complementary filter
            rollSum = 0.2 * rollSum + 0.8 * roll;
            pitchSum = 0.9 * pitchSum + 0.1 * pitch;
            gyroSum = 0.93 * gyroSum + 0.07 * gyro;
        }
        /*  SerialAOG.print(roll);
          SerialAOG.print(",");
          SerialAOG.print(rollSum);
          SerialAOG.print(" * ");
          SerialAOG.print(pitch);
          SerialAOG.print(",");
          SerialAOG.print(pitchSum);
          SerialAOG.print(" * ");
          SerialAOG.print(bno08xHeading10x);
          SerialAOG.print(" * ");
          SerialAOG.print(gyro);
          SerialAOG.print(",");
          SerialAOG.println(gyroSum);*/
    }

	//save time to check for 10 msec
	lastGyroTime = systick_millis_count;
}