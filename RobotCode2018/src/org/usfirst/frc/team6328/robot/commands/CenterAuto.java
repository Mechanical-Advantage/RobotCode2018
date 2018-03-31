package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Center auto
 */
public class CenterAuto extends CommandGroup {
	
	private static final double ejectSpeed = 0.4;
	private static final double backUpDistance = 30;
	private static final double backUpTolerance = 3;
	
	public CenterAuto(OwnedSide switchSide) {
		if (switchSide == OwnedSide.LEFT || switchSide == OwnedSide.RIGHT) {
			String sideString = switchSide.toString().toLowerCase();
			sideString = sideString.replace("l", "L");
			sideString = sideString.replace("r", "R");
//			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new RunMotionProfileOnRio("centerTo" + sideString + "Switch", false, true, false, true));
			addSequential(new DriveForTime(1, DriveGear.HIGH, 0.15, 0.15));
			addSequential(new ExtendIntake());
			addSequential(new EjectCube(ejectSpeed));
			addSequential(new DriveDistanceOnHeading(-backUpDistance, 0, backUpTolerance, 0, 0));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			double turn = switchSide == OwnedSide.LEFT ? -90 : 90;
			addSequential(new TurnToAngle(turn, true));
		}
	}
}
