package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.commands.SetElevatorPosition.ElevatorPosition;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 * Make the robot climb using the elevator
 */
public class Climb extends CommandGroup {

	public Climb() {
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
		addSequential(new FinalClimb());
	}
	
	private class FinalClimb extends Command {
		
		private static final double climbPosition = 0;
		
		public FinalClimb() {
			super("FinalClimb");
			requires(Robot.elevator);
		}
		
		@Override
		protected void initialize() {
			Robot.elevator.enableClimbLock(true);
			Robot.elevator.switchGear(ElevatorGear.LOW);
			Robot.elevator.setPosition(climbPosition);
		}
		
		@Override
		protected boolean isFinished() {
			return Robot.elevator.onTarget();
		}
	}
}
