package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Extends (un-retracts) the intake;
 */
public class ExtendIntake extends TimedCommand {
	
	private static final double time = 1;

	public ExtendIntake() {
		super("ExtendIntake", time);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.intake);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.intake.setRetracted(false);
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && Robot.intake.getRetracted()) {
			Robot.intake.intake();
		}
	}

	protected void end() {
		Robot.intake.stop();
	}
}
