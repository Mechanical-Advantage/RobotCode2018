package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.subsystems.DriveTrain.DriveGear;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchStraightDelivery extends CommandGroup {

	private static final double switchDistance = 140;
	private static final double ejectSpeed = 0.4;
	private static final double backUpDistance = 30;
	private static final double backUpTolerance = 3;
	
	public SwitchStraightDelivery() {
//		addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
		addSequential(new DriveDistanceOnHeading(switchDistance-RobotMap.robotLength, 0));
		addSequential(new DriveForTime(1, DriveGear.HIGH, 0.15, 0.15));
		addSequential(new ExtendIntake());
		addSequential(new EjectCubeForTime(ejectSpeed));
		addSequential(new DriveDistanceOnHeading(-backUpDistance, 0, backUpTolerance, 0, 0));
		addSequential(new SetElevatorPosition(ElevatorPosition.GROUND));
	}
}