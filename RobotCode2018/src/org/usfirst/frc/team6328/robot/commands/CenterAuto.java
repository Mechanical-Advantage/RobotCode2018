package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Center auto
 */
public class CenterAuto extends CommandGroup {
	
	private static final double ejectSpeed = 0.4;
	private static final double extendDelay = 2;
	private static final double ejectDelay = 1;
	private static final double backUpDistance = 30;
	private static final double backUpTolerance = 3;
	
	public CenterAuto(OwnedSide switchSide) {
		if (switchSide == OwnedSide.LEFT || switchSide == OwnedSide.RIGHT) {
			String sideString = switchSide.toString().toLowerCase();
			sideString = sideString.replace("l", "L");
			sideString = sideString.replace("r", "R");
//			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addParallel(new IntakeControl());
			addSequential(new RunMotionProfileOnRio("centerTo" + sideString + "Switch", false, true, false, true));
			addSequential(new DriveDistanceOnHeading(-backUpDistance, 0, backUpTolerance, 0, 0));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			double turn = switchSide == OwnedSide.LEFT ? -90 : 90;
			addSequential(new TurnToAngle(turn, true));
		}
	}
	
	private static class IntakeControl extends CommandGroup {
		public IntakeControl() {
			addSequential(new Delay(extendDelay));
			addSequential(new ExtendIntake());
			addSequential(new Delay(ejectDelay));
			addSequential(new EjectCubeForTime(ejectSpeed));
		}
	}
}
