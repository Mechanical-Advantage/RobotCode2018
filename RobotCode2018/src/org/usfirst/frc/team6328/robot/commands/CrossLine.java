package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Crosses the auto line
 */
public class CrossLine extends CommandGroup {

	public CrossLine() {
		// Add Commands here:
		// e.g. addSequential(new Command1());
		//      addSequential(new Command2());
		// these will run in order.

		// To run multiple commands at the same time,
		// use addParallel()
		// e.g. addParallel(new Command1());
		//      addSequential(new Command2());
		// Command1 and Command2 will run in parallel.

		// A command group will require all of the subsystems that each member
		// would require.
		// e.g. if Command1 requires chassis, and Command2 requires arm,
		// a CommandGroup containing them would require both the chassis and the
		// arm.
		Command intakeCommand = new IntakeCube(false);
		addParallel(intakeCommand);
		addSequential(new DriveDistanceOnHeading(120+12-RobotMap.robotLength, 0));
		addSequential(new CancelCommand(intakeCommand));
	}
}
