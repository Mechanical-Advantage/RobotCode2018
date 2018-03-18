package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Toggles whether the intake is retracted
 */
public class ToggleIntakeRetracted extends TimedCommand {
	
	private static final double runTime = 1;

	public ToggleIntakeRetracted() {
		super(runTime);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			Robot.intake.intake();
			if (Robot.intake.getRetracted()) {
				Robot.intake.setRetracted(false);
			} else {
				Robot.intake.setRetracted(true);
			}
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
