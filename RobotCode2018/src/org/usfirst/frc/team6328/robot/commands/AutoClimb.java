package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Starting against the scale, back up and grab rung
 */
public class AutoClimb extends CommandGroup {
	
	private static final double backupDistance = 10.5;
	private static final double forwardDistance = 7.5;

	public AutoClimb() {
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
		addSequential(new SetLED(true));
		addSequential(new DriveDistanceOnHeading(backupDistance*-1));
		addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
		addSequential(new DriveDistanceOnHeading(forwardDistance));
		addSequential(new SwitchElevatorGear(ElevatorGear.LOW));
		addSequential(new SetLED(false));
	}
	
	private static class SetLED extends InstantCommand {
		
		boolean state;
		
		public SetLED(boolean state) {
			this.state = state;
		}
		
		@Override
		protected void initialize() {
			Robot.oi.updateLED(OILED.CLIMB, state);
		}
	}
	
	@Override
	protected void interrupted() {
		super.interrupted();
		Robot.oi.updateLED(OILED.CLIMB, false);
	}
}
