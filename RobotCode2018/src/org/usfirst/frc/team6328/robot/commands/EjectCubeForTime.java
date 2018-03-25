package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Eject a cube from the robot
 */
public class EjectCubeForTime extends TimedCommand {
	
	private static final double time = 0.5;
	
	private double speed;
	
	public EjectCubeForTime() {
		super(time);
		requires(Robot.intake);
	}

	public EjectCubeForTime(double speed) {
		super(time);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
		this.speed = speed;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
//		Robot.intake.setRetracted(false);
		if (speed == 0) {
			Robot.intake.eject();
		} else {
			Robot.intake.eject(speed);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Called once after timeout
	protected void end() {
		Robot.intake.stop();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
