package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives straight using the current single drive velocity from OI and gyro correction
 */
public class DriveWithJoystickOnHeading extends Command {
	
	//should be the same as DriveDistanceOnHeading
	static final double kPAngle = 0.07;
    static final double kIAngle = 0.000;
    static final double kDAngle = 0.0;
    static final double kFAngle = 0.0;
    static final double kToleranceDegrees = 0.5f;
    static final int kToleranceBufSamplesAngle = 10;
    static final double kTurnCorrectionAmount = 0.2;
    
    private double targetAngle;
    private boolean useStartingYaw;
    private PIDController turnController;
    private PIDOutputter pidOutputAngle = new PIDOutputter();
    
    public DriveWithJoystickOnHeading() {
    	this(0);
    	useStartingYaw = true;
    }

    public DriveWithJoystickOnHeading(double heading) {
    	super("DriveWithJoystickOnHeading");
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.driveSubsystem);
    	targetAngle = (heading>180) ? 180 : heading;
        targetAngle = (targetAngle<-180) ? -180 : targetAngle;
        useStartingYaw = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("Driver Control", true);
    	if (useStartingYaw) {
    		targetAngle = Robot.ahrs.getYaw();
    	}
    	turnController = new PIDController(kPAngle, kIAngle, kDAngle, kFAngle, Robot.ahrs, pidOutputAngle);
    	turnController.setOutputRange(-1, 1);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-1.0, 1.0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setToleranceBuffer(kToleranceBufSamplesAngle);
        turnController.setContinuous(true);
        turnController.setSetpoint(targetAngle);
        turnController.enable();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double outputVelocity = Math.pow(Robot.oi.getSingleDriveAxis(), 3)*-1;
    	double outputTurnVelocity = pidOutputAngle.getPIDRate()*kTurnCorrectionAmount;
		// subtract from right side, add to left side (drive left on positive)
		Robot.driveSubsystem.drive(outputVelocity+outputTurnVelocity, outputVelocity-outputTurnVelocity);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	turnController.disable();
    	Robot.driveSubsystem.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	SmartDashboard.putBoolean("Driver Control", false);
    	turnController.disable();
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
