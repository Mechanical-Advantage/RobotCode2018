package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.commands.SetElevatorPosition.ElevatorPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Runs a side auto
 */
public class SideAutoScaleAndSwitch extends InstantCommand {
	
	private static final double cubePickUpToSwitchDistance = 12;
	private static final double cubePickUpToScaleDistance = 20;
	private static final double scaleBackUpDistance = -12; // Should be negative
	private static final double scaleCrossDistance = 50; // How far to drive along space between switch and scale
	
	private boolean leftSide;

	public SideAutoScaleAndSwitch(boolean leftSide) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.leftSide = leftSide;
	}

	// Called once when the command executes
	protected void initialize() {
		OwnedSide switchSide = MatchData.getOwnedSide(GameFeature.SWITCH_NEAR);
		OwnedSide scaleSide = MatchData.getOwnedSide(GameFeature.SCALE);
		OwnedSide side = leftSide ? OwnedSide.LEFT : OwnedSide.RIGHT;
		Command sideAutoCommand;
		if (side == switchSide && side == scaleSide) {
			sideAutoCommand = new BothSameSide();
			System.out.println("Running both same side auto");
		} else if (side == switchSide) {
			sideAutoCommand = new SwitchSameSide();
			System.out.println("Running switch same side auto");
		} else if (side == scaleSide) {
			sideAutoCommand = new ScaleSameSide();
			System.out.println("Running scale same side auto");
		} else {
			sideAutoCommand = new BothOppositeSide();
			System.out.println("Running both opposite side auto");
		}
		
		sideAutoCommand.start();
	}
	
	private class BothSameSide extends CommandGroup {
		public BothSameSide() {
			addParallel(new SetElevatorPosition(ElevatorPosition.SCALE));
			addSequential(new RunMotionProfileOnRio("sideToScale", leftSide, true, false));
			addSequential(new EjectCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new TurnToAngle(180, true));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new EjectCube());
		}
	}
	
	private class BothOppositeSide extends CommandGroup {
		public BothOppositeSide() {
			addParallel(new SetElevatorPosition(ElevatorPosition.SCALE));
			addSequential(new RunMotionProfileOnRio("sideToOppositeScale", leftSide, true, false));
			addSequential(new EjectCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new TurnToAngle(180, true));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new EjectCube());
		}
	}
	
	private class SwitchSameSide extends CommandGroup {
		public SwitchSameSide() {
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new RunMotionProfileOnRio("sideToSwitch", leftSide, true, false));
			addSequential(new EjectCube());
			addSequential(new RunMotionProfileOnRio("sideSwitchPrepareCrossing", leftSide, true, true));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new RunMotionProfileOnRio("sideSwitchCross", leftSide, true, false));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SCALE));
			addSequential(new TurnToAngle(0, true));
			addSequential(new DriveDistanceOnHeading(cubePickUpToScaleDistance, 0));
			addSequential(new EjectCube());
			addSequential(new DriveDistanceOnHeading(scaleBackUpDistance, 0));
		}
	}
	
	private class ScaleSameSide extends CommandGroup {
		public ScaleSameSide() {
			addParallel(new SetElevatorPosition(ElevatorPosition.SCALE));
			addSequential(new RunMotionProfileOnRio("sideToScale", leftSide, true, false));
			addSequential(new EjectCube());
			addSequential(new RunMotionProfileOnRio("scalePrepareCrossing", leftSide, true, true));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new DriveDistanceOnHeading(scaleCrossDistance, -90));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new EjectCube());
		}
	}
}