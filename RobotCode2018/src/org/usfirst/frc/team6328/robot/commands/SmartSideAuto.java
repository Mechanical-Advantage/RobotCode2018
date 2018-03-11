package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Runs a side auto based on the passed destinations
 */
public class SmartSideAuto extends InstantCommand {
	
	private static final double switchSideDriveDistance = 28;
	private static final double cubePickUpToSwitchDistance = 12;
	private static final double cubePickUpToScaleDistance = 20;
	private static final double scaleBackUpDistance = -12; // Should be negative
	private static final double scaleCrossDistance = 150; // How far to drive along space between switch and scale
	private static final double scaleFrontSpeed = 0.6;
	private static final double switchFrontSpeed = 0.7;
	private static final double switchEndSpeed = 0.8;
		
	private boolean leftSide;
	private Command command;

	public SmartSideAuto(AutoDestination firstDest, AutoDestination secondDest, boolean leftSide) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.leftSide = leftSide;
		switch (firstDest) {
		case SCALE_OPPOSITE:
			if (secondDest == AutoDestination.SWITCH_OPPOSITE) {
				command = new BothOppositeSide();
			} else {
				command = new ScaleOppositeSide();
			}
			break;
		case SCALE_SAME:
			if (secondDest == AutoDestination.SWITCH_SAME) {
				command = new BothSameSide();
			} else if (secondDest == AutoDestination.SWITCH_OPPOSITE) {
				command = new ScaleSameSideTwoCube();
			} else {
				command = new ScaleSameSide();
			}
			break;
		case SWITCH_OPPOSITE:
			command = new SwitchOppositeSide();
			break;
		case SWITCH_SAME:
			if (secondDest == AutoDestination.SCALE_OPPOSITE) {
				command = new SwitchSameSideTwoCube();
			} else {
				command = new SwitchSameSide();
			}
			break;
		default:
			break;
		}
	}

	// Called once when the command executes
	protected void initialize() {
		command.start();
	}

	private class SwitchOppositeSide extends CommandGroup {
		public SwitchOppositeSide() {
			addParallel(new ExtendIntake());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new RunMotionProfileOnRio("sideToOppositeSwitch", leftSide, true, false, false));
			double heading = leftSide ? -90 : 90;
			addSequential(new TurnToAngle(heading, true));
			addSequential(new DriveDistanceOnHeading(switchSideDriveDistance, heading));
			addSequential(new EjectCube(switchEndSpeed));
		}
	}
	
	private class SwitchSameSide extends CommandGroup {
		public SwitchSameSide() {
			addParallel(new ExtendIntake());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new RunMotionProfileOnRio("sideToSwitch", leftSide, true, false, true));
			addSequential(new EjectCube(switchEndSpeed));
		}
	}
	
	private class ScaleSameSide extends CommandGroup {
		public ScaleSameSide() {
			addParallel(new ExtendIntake());
			addParallel(new SetElevatorPosition(ElevatorPosition.DRIVE));
			addSequential(new RunMotionProfileOnRio("sideToScale", leftSide, true, false, true));
			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
			addSequential(new EjectCube(scaleFrontSpeed));
		}
	}
	
	private class ScaleOppositeSide extends CommandGroup {
		public ScaleOppositeSide() {
			addParallel(new ExtendIntake());
			addParallel(new SetElevatorPosition(ElevatorPosition.DRIVE));
			addSequential(new RunMotionProfileOnRio("sideToOppositeScale", leftSide, true, false, true));
			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
			addSequential(new EjectCube(scaleFrontSpeed));
		}
	}
	
	private class ScaleToSameSwitch extends CommandGroup {
		public ScaleToSameSwitch() {
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new TurnToAngle(180, true));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new EjectCube(switchFrontSpeed));
		}
	}
	
	private class BothSameSide extends CommandGroup {
		public BothSameSide() {
			addSequential(new ScaleSameSide());
			addSequential(new ScaleToSameSwitch());
		}
	}
	
	private class BothOppositeSide extends CommandGroup {
		public BothOppositeSide() {
			addSequential(new ScaleOppositeSide());
			addSequential(new ScaleToSameSwitch());
		}
	}
	
	private class SwitchSameSideTwoCube extends CommandGroup {
		public SwitchSameSideTwoCube() {
			addSequential(new SwitchSameSide());
			addSequential(new RunMotionProfileOnRio("sideSwitchPrepareCrossing", leftSide, true, true, true));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new RunMotionProfileOnRio("sideSwitchCross", leftSide, true, false, true));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.DRIVE));
			addSequential(new TurnToAngle(0, true));
			addSequential(new DriveDistanceOnHeading(cubePickUpToScaleDistance, 0));
			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
			addSequential(new EjectCube(scaleFrontSpeed));
			addSequential(new DriveDistanceOnHeading(scaleBackUpDistance, 0));
		}
	}
	
	private class ScaleSameSideTwoCube extends CommandGroup {
		public ScaleSameSideTwoCube() {
			addSequential(new ScaleSameSide());
			addSequential(new RunMotionProfileOnRio("scalePrepareCrossing", leftSide, true, true, true));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new DriveDistanceOnHeading(scaleCrossDistance, -90));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new EjectCube(switchFrontSpeed));
		}
	}
	
	public enum AutoDestination {
		SWITCH_SAME, SWITCH_OPPOSITE, SCALE_SAME, SCALE_OPPOSITE
	}
}
