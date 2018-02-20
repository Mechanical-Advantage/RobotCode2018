package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Drives with the joystick from OI
 */
public class DriveWithJoystick extends Command {
	
	private final double deadband = 0.05;
	private static final boolean alwaysUseHighMaxVel = true; // Whether to always use the max velocity of high gear or of current gear

    public DriveWithJoystick() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	super("DriveWithJoystick");
    	requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	SmartDashboard.putBoolean("Driver Control", true);
    }
    
    private double processJoystickAxis(double joystickAxis) {
		// cube to improve low speed control, multiply by -1 because negative joystick means forward, 0 if within deadband
    		return Math.abs(joystickAxis) > deadband ? joystickAxis*Math.abs(joystickAxis)*-1 : 0;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		double joystickLeft = 0, joystickRight = 0;
    		switch (Robot.joystickModeChooser.getSelected()) {
    		case Tank:
    			joystickRight = processJoystickAxis(Robot.oi.getRightAxis());
    			joystickLeft = processJoystickAxis(Robot.oi.getLeftAxis());
    			break;
    		case SplitArcade:
    			double baseDrive = processJoystickAxis(Robot.oi.getSingleDriveAxis());
    			joystickRight = baseDrive + processJoystickAxis(Robot.oi.getHorizDriveAxis());
    			joystickRight = joystickRight > 1 ? 1 : joystickRight;
    			joystickLeft = baseDrive - processJoystickAxis(Robot.oi.getHorizDriveAxis());
    			joystickLeft = joystickLeft > 1 ? 1 : joystickLeft;
    			break;
    		}
    		Robot.driveSubsystem.drive(joystickLeft, joystickRight, alwaysUseHighMaxVel);
    		//System.out.println("Left: " + Robot.oi.getLeftAxis() + " Right: " + Robot.oi.getRightAxis());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	SmartDashboard.putBoolean("Driver Control", false);
    }
    
    public static enum JoystickMode {
    		Tank, SplitArcade;
    }
}
