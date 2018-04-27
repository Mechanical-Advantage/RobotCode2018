package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.commands.RunMotionProfileOnRio.ConvergenceMode;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Runs a side auto based on the passed destinations
 */
public class SmartSideAuto extends InstantCommand {
	
	private static final double cubePickUpToSwitchDistance = 12;
	private static final double cubePickUpToScaleDistance = 20;
	private static final double scaleBackUpDistance = 42;
	private static final double scaleBackUpTolerance = 2;
	private static final double frontSwitchBackUpDistance = 30;
	private static final double frontSwitchBackUpTolerance = 2;
	private static final double endSwitchBackUpDistance = 30;
	private static final double endSwitchBackUpTolerance = 2;
	private static final double scaleCrossDistance = 150; // How far to drive along space between switch and scale
	private static final double scaleFrontSpeed = 0.3;
	private static final double switchFrontSpeed = 0.5;
	private static final double switchEndSpeed = 0.8;
	private static final double sameSwitchExtendDelay = 2;
	private static final double sameSwitchEjectDelay = 1.5;
		
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
				command = new ScaleOppositeSide(true);
			}
			break;
		case SCALE_SAME:
			if (secondDest == AutoDestination.SWITCH_SAME) {
				command = new BothSameSide();
			} else if (secondDest == AutoDestination.SWITCH_OPPOSITE) {
				command = new ScaleSameSideTwoCube();
			} else {
				command = new ScaleSameSide(true);
			}
			break;
		case SCALE_END:
			command = new ScaleSameSideEnd();
			break;
		case SWITCH_OPPOSITE:
			command = new SwitchOppositeSide();
			break;
		case SWITCH_SAME:
			if (secondDest == AutoDestination.SCALE_OPPOSITE) {
				command = new SwitchSameSideTwoCube();
			} else {
				command = new SwitchSameSide(true);
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
			addSequential(new RunMotionProfileOnRio("sideToOppositeSwitch", leftSide, true, false, false));
			double heading = leftSide ? -90 : 90;
			addSequential(new TurnToAngle(heading, true));
			addParallel(new ExtendIntake());
			addSequential(new DriveForTime(2, DriveGear.HIGH, 0.15, 0.15));
			addSequential(new EjectCubeForTime(switchEndSpeed));
			addSequential(new DriveDistanceOnHeading(-endSwitchBackUpDistance, leftSide ? -90 : 90, endSwitchBackUpTolerance, 0, 0));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new TurnToAngle(0, true));
		}
	}
	
	private class SwitchSameSide extends CommandGroup {
		public SwitchSameSide(boolean enableBackup) {
			addParallel(new SwitchIntakeControl());
			addSequential(new RunMotionProfileOnRio("sideToSwitch", leftSide, true, false, false));
			if (enableBackup) {
				addSequential(new DriveDistanceOnHeading(-endSwitchBackUpDistance, leftSide ? 90 : -90, endSwitchBackUpTolerance, 0, 0));
				addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
				addSequential(new TurnToAngle(0, true));
			}
		}
	}
	
	private class ScaleSameSide extends CommandGroup {
		public ScaleSameSide(boolean enableBackup) {
			addParallel(new RaiseAndExtendIntake());
			addSequential(new RunMotionProfileOnRio("sideToScale", leftSide, true, false, true));
			addSequential(new EjectCubeForTime(scaleFrontSpeed));
			/*addParallel(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
			addSequential(new RunMotionProfileOnRio("sideToScale", leftSide, true, false, true));
			addSequential(new ExtendIntake());
			addSequential(new EjectCubeForTime(scaleFrontSpeed));*/
			if (enableBackup) {
				addSequential(new DriveDistanceOnHeading(-scaleBackUpDistance, 0, scaleBackUpTolerance, 0, 0));
				addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
				int invert = leftSide ? -1 : 1;
				addSequential(new TurnToAngle(-165*invert, true));
			}
		}
	}
	
	private class ScaleSameSideEnd extends CommandGroup {
		public ScaleSameSideEnd() {
			addParallel(new RaiseAndExtendIntake());
			addSequential(new RunMotionProfileOnRio("sideToScaleEnd", leftSide, true, false, false));
			addSequential(new EjectCubeForTime(scaleFrontSpeed));
			addSequential(new DriveDistanceOnHeading(-12, 2, 0, 0));
//			addSequential(new TurnToAngle(180, true));
			addSequential(new SetElevatorPosition(ElevatorPosition.GROUND));
		}
	}
	
	private class ScaleOppositeSide extends CommandGroup {
		public ScaleOppositeSide(boolean enableBackup) {
			addParallel(new TimedLift());
			addSequential(new RunMotionProfileOnRio("sideToOppositeScale", leftSide, true, false, ConvergenceMode.ALWAYS)); // convergence disabled if on platform
			addSequential(new EjectCubeForTime(scaleFrontSpeed));
			/*addParallel(new TimedLift());
			addSequential(new RunMotionProfileOnRio("sideToOppositeScale", leftSide, true, false, ConvergenceMode.IF_FLAT)); // convergence disabled if on platform
			addSequential(new ExtendIntake());
			addSequential(new EjectCubeForTime(scaleFrontSpeed));*/
			if (enableBackup) {
				addSequential(new DriveDistanceOnHeading(-scaleBackUpDistance, 0, scaleBackUpTolerance, 0, 0));
				addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
				int invert = leftSide ? -1 : 1;
				addSequential(new TurnToAngle(160*invert, true));
			}
		}
	}
	
	private class ScaleToSameSwitch extends CommandGroup {
		public ScaleToSameSwitch() {
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new TurnToAngle(180, true));
			addSequential(new DriveToCube());
			addParallel(new IntakeCube(false));
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new DriveForTime(1, DriveGear.HIGH, 0.15, 0.15));
			addSequential(new EjectCubeForTime(switchFrontSpeed));
			addSequential(new DriveDistanceOnHeading(-frontSwitchBackUpDistance, 180, frontSwitchBackUpTolerance, 0, 0));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
		}
	}
	
	private class BothSameSide extends CommandGroup {
		public BothSameSide() {
			addSequential(new ScaleSameSide(false));
			addSequential(new ScaleToSameSwitch());
		}
	}
	
	private class BothOppositeSide extends CommandGroup {
		public BothOppositeSide() {
			addSequential(new ScaleOppositeSide(false));
			addSequential(new ScaleToSameSwitch());
		}
	}
	
	private class SwitchSameSideTwoCube extends CommandGroup {
		public SwitchSameSideTwoCube() {
			addSequential(new SwitchSameSide(false));
			addSequential(new RunMotionProfileOnRio("sideSwitchPrepareCrossing", leftSide, true, true, false));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new RunMotionProfileOnRio("sideSwitchCross", leftSide, true, false, false));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.DRIVE));
			addSequential(new TurnToAngle(0, true));
			addSequential(new DriveDistanceOnHeading(cubePickUpToScaleDistance, 0));
			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
			addSequential(new EjectCubeForTime(scaleFrontSpeed));
			addSequential(new DriveDistanceOnHeading(-scaleBackUpDistance, 0, scaleBackUpTolerance, 0, 0));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new TurnToAngle(180, true));
		}
	}
	
	private class ScaleSameSideTwoCube extends CommandGroup {
		public ScaleSameSideTwoCube() {
			addSequential(new ScaleSameSide(false));
			addSequential(new RunMotionProfileOnRio("scalePrepareCrossing", leftSide, true, true, false));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
			addSequential(new DriveDistanceOnHeading(scaleCrossDistance, -90));
			addSequential(new DriveToCube());
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new DriveDistanceOnHeading(cubePickUpToSwitchDistance, 180));
			addSequential(new DriveForTime(1, DriveGear.HIGH, 0.15, 0.15));
			addSequential(new EjectCubeForTime(switchFrontSpeed));
			addSequential(new DriveDistanceOnHeading(-frontSwitchBackUpDistance, 180, frontSwitchBackUpTolerance, 0, 0));
			addParallel(new SetElevatorPosition(ElevatorPosition.GROUND));
		}
	}
	
	private class TimedLift extends CommandGroup {
		public TimedLift() {
			addSequential(new Delay(5));
			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH, 1));
			addSequential(new ExtendIntake());
//			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
		}
	}
	
	private class RaiseAndExtendIntake extends CommandGroup {
		public RaiseAndExtendIntake() {
			addSequential(new SetElevatorPosition(ElevatorPosition.SCALE_HIGH));
			addSequential(new ExtendIntake());
		}
	}
	
	private class SwitchIntakeControl extends CommandGroup {
		public SwitchIntakeControl() {
			addSequential(new Delay(sameSwitchExtendDelay));
			addSequential(new ExtendIntake());
			addSequential(new Delay(sameSwitchEjectDelay));
			addSequential(new EjectCubeForTime(switchEndSpeed));
		}
	}
	
	public enum AutoDestination {
		SWITCH_SAME, SWITCH_OPPOSITE, SCALE_SAME, SCALE_END, SCALE_OPPOSITE
	}
}
