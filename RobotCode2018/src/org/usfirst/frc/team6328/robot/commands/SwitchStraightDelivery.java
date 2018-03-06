package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.RobotMap;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchStraightDelivery extends CommandGroup {

	private static final double switchDistance = 140;
	
	public SwitchStraightDelivery() {
		addSequential(new DriveDistanceOnHeading(switchDistance-RobotMap.robotLength, 0));
		addSequential(new EjectCube());
	}
}