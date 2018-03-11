package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.subsystems.Elevator.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchStraightDelivery extends CommandGroup {

	private static final double switchDistance = 140;
	private static final double ejectSpeed = 0.6;
	
	public SwitchStraightDelivery() {
		addParallel(new ExtendIntake());
		addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
		addSequential(new DriveDistanceOnHeading(switchDistance-RobotMap.robotLength, 0));
		addSequential(new EjectCube(ejectSpeed));
	}
}