package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyException;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyPacket;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive to a cube
 */
public class DriveToCube extends Command {

	private static final double cameraHorizFOV = 79.84523585564033;
	private static final double cameraVertFOV = 47;
	private static final double cameraHeight = 24.5;
	private static final double cameraVertAngle = 14; // How far down the camera is pointed
	private static final int cameraWidthPixels = 320; // pixels
	private static final int cameraHeightPixels = 200; // 0 at top

	private double kPDistance;
	private double kIDistance;
	private double kDDistance;
	private double kFDistance;
	static final double kUpdatePeriodDistance = 0.02;

	private double kPAngle;
	private double kIAngle;
	private double kDAngle;
	private double kFAngle;
	static final double kTurnCorrectionAmount = 0.2;
	static final double kUpdatePeriodAngle = 0.05;

	// PID output will be limited to negative to positive this. Multiplied by RobotMap maxVelocity to get target
	static final double kMaxOutput = 0.9;

	// Calculated constants
	private static final int halfCameraHeightPixels = cameraHeightPixels/2;
	private static final int halfCameraWidthPixels = cameraWidthPixels/2;
	private static final double cameraHorizTan = Math.tan(Math.toRadians(cameraHorizFOV/2));
	private static final double cameraVertTan = Math.tan(Math.toRadians(cameraVertFOV/2));

	private PIDController distanceController;
	private PIDController turnController;
	private DistancePIDSource pidSourceDistance = new DistancePIDSource();
	private AnglePIDSource pidSourceAngle = new AnglePIDSource();
	private PIDOutputter pidOutputDistance = new PIDOutputter();
	private PIDOutputter pidOutputAngle = new PIDOutputter();
	private double angle;
	private double distance;

	public DriveToCube() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.pixy);
		requires(Robot.driveSubsystem);
		switch (RobotMap.robot) {
		case PRACTICE:
			kPDistance = 0.017;
			kIDistance = 0;
			kDDistance = 0;
			kFDistance = 0.5;
			kPAngle = 0.07;
			kIAngle = 0;
			kDAngle = 0;
			kFAngle = 0;
			break;
		case ROBOT_2017:
			kPDistance = 0.032;
			kIDistance = 0.000000;
			kDDistance = 0;
			kFDistance = 0;
			kPAngle = 0.07;
			kIAngle = 0;
			kDAngle = 0;
			kFAngle = 0;
			break;
		case ROBOT_2018:
			break;
		}
		distanceController = new PIDController(kPDistance, kIDistance, kDDistance, kFDistance, pidSourceDistance, pidOutputDistance, kUpdatePeriodDistance);
		turnController = new PIDController(kPAngle, kIAngle, kDAngle, kFAngle, pidSourceAngle, pidOutputAngle, kUpdatePeriodAngle);
		distanceController.setOutputRange(-1, 1);
		turnController.setOutputRange(-1, 1);
		turnController.setInputRange(-180, 180); // should this be field of view?
		turnController.setContinuous();
		distanceController.setSetpoint(0);
		turnController.setSetpoint(0);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		turnController.enable();
		distanceController.enable();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		try {
			PixyPacket cube = Robot.pixy.readPacket(1);
			if (cube != null) {
				// See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
				int cubeBottom = cube.Y + (cube.Height/2);
				angle = Math.toDegrees(
						Math.atan(((cube.X-halfCameraWidthPixels)*cameraHorizTan
								/halfCameraWidthPixels)));
				double angleV = (Math.toDegrees(
						Math.atan(((cubeBottom-halfCameraHeightPixels)*-1*cameraVertTan
								/halfCameraHeightPixels)))) - cameraVertAngle;
				distance = Math.tan(Math.toRadians(90-Math.abs(angleV))) * cameraHeight;
				double horizDistance = Math.tan(Math.toRadians(angle)) * distance;
				System.out.print("H: " + horizDistance);
				System.out.print(" HA: " + angle);
				System.out.print(" D: " + distance);
				System.out.print(" VA: " + angleV);
				System.out.print(" X: " + cube.X);
				System.out.println(" Y: " + cubeBottom);
			}
		} catch (PixyException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		
		// Inverted velocity because PID is trying to push input lower
		double outputVelocity = pidOutputDistance.getPIDRate()*(kMaxOutput-kTurnCorrectionAmount)*-1;
		double outputTurnVelocity = pidOutputAngle.getPIDRate()*kTurnCorrectionAmount;
		Robot.driveSubsystem.drive(outputVelocity-outputTurnVelocity, outputVelocity+outputTurnVelocity);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
		turnController.disable();
		distanceController.disable();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

	private class DistancePIDSource implements PIDSource {
		@Override
		public double pidGet() {
			return distance;
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {

		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}
	
	private class AnglePIDSource implements PIDSource {
		@Override
		public double pidGet() {
			return angle;
		}

		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {

		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
	}

	private class PIDOutputter implements PIDOutput {
		private double PIDRate;
		public double getPIDRate() {
			return PIDRate;
		}
		@Override
		public void pidWrite(double output) {
			PIDRate = output;
		}
	}
}
