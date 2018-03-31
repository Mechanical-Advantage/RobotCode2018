package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Controls front and back cameras
 */
public class CameraSystem extends Subsystem {

	private UsbCamera frontCamera;
	private UsbCamera secondCamera;
	private boolean serverCreated = false;
	private boolean frontCameraAdded = false;
	private boolean secondCameraAdded = false;
	private int frontCameraID;
	private int secondCameraID;

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}

	public CameraSystem() {
		switch (RobotMap.robot) {
		case ROBOT_2017:
			frontCameraID = 2;
			secondCameraID = 0;
			break;
		case ORIGINAL_ROBOT_2018:
			frontCameraID = 0;
			secondCameraID = 2;
			break;
		case EVERYBOT_2018:
		case PRACTICE:
		default:
			break;
		}
	}

	// To get network tables publishing, must use startAutomaticCapture and have it create the UsbCamera.
	// This should not have to work this way, but CameraServer is finicky, and this was the only way it worked
	// Written for WPILib 2017.3.1
	// Note: Second Camera is just internal, selecting on dashboard will have no effect
	private UsbCamera setupServer(int id) {
		serverCreated = true;
		return CameraServer.getInstance().startAutomaticCapture("Video Feed", id);
	}

	public void useFrontCamera() {
		if (!frontCameraAdded) {
			if (!serverCreated) {
				frontCamera = setupServer(frontCameraID);
			} else {
				frontCamera = new UsbCamera("Second Camera", frontCameraID);
			}
			frontCamera.setResolution(320, 240);
			frontCamera.setFPS(15);
			frontCameraAdded = true;
		}
		CameraServer.getInstance().getServer().setSource(frontCamera);
		System.out.println("Switching to front camera");
	}

	public void useSecondCamera() {
		if (RobotMap.robot == RobotType.ROBOT_2017 || RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			if (!secondCameraAdded) {
				if (!serverCreated) {
					secondCamera = setupServer(secondCameraID);
				} else {
					secondCamera = new UsbCamera("Second Camera", secondCameraID);
				}
				secondCamera.setResolution(320, 240);
				secondCamera.setFPS(15);
				secondCameraAdded = true;
			}
			CameraServer.getInstance().getServer().setSource(secondCamera);
			System.out.println("Switching to second camera");
		}
	}
}

