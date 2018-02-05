package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.commands.SetElevatorPosition.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Lowers the elevator, engages the climb lock, and raises the elevator
 */
public class ClimbGrab extends CommandGroup {

	public ClimbGrab() {
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
		addSequential(new SetElevatorPosition(ElevatorPosition.GROUND));
		addSequential(new EngageClimbLock());
		addSequential(new SetElevatorPosition(ElevatorPosition.CLIMB_GRAB));
	}
	
	private class EngageClimbLock extends InstantCommand {
		public EngageClimbLock() {
			super("EngageClimbLock");
			requires(Robot.elevator);
		}
		
		@Override
		protected void initialize() {
			Robot.elevator.enableClimbLock(true);
		}
	}
}
