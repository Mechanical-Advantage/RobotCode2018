package org.usfirst.frc.team6328.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Interface to the maxbotix ultrasonic sensor
 * Only works if console out is disabled in rio settings
 */
public class MaxbotixUltrasonic extends Subsystem {

	private SerialPort port;
	private boolean readingValue;
	private StringBuilder inProgressValue = new StringBuilder();
	private int lastValue;
	private boolean receivedData = false;
	
	public MaxbotixUltrasonic(SerialPort.Port port) {
		this.port = new SerialPort(9600, port);
	}
	
	@Override
	public void periodic() {
		while (port.getBytesReceived() > 0) {
			String currentByte = port.readString(1);
			receivedData = true;
			if (currentByte.equals("R")) {
				readingValue = true;
				inProgressValue = new StringBuilder();
			} else if (currentByte.equals("\r")) {
				readingValue = false;
				lastValue = Integer.parseInt(inProgressValue.toString());
			} else if (readingValue) {
				inProgressValue.append(currentByte);
			}
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
	
	public int getDistance() {
		return lastValue;
	}
	
	public boolean isSensorConnected() {
		return receivedData;
	}
}

