package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Toggles the current drive gear
 */
public class ToggleGear extends InstantCommand {

	public ToggleGear() {
		super("ToggleGear");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
	}

	// Called once when the command executes
	protected void initialize() {
		if (Robot.driveSubsystem.getCurrentGear() == DriveGear.LOW) {
			Robot.driveSubsystem.switchGear(DriveGear.HIGH);
		} else if (Robot.driveSubsystem.getCurrentGear() == DriveGear.HIGH) {
			Robot.driveSubsystem.switchGear(DriveGear.LOW);
		}
	}

}
