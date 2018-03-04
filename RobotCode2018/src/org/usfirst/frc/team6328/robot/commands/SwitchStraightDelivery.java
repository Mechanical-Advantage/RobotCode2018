package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SwitchStraightDelivery extends CommandGroup {

	private static final double crossLineDistance = 130;
	
	public SwitchStraightDelivery() {
		addSequential(new DriveDistanceOnHeading(crossLineDistance, 0));
		addSequential(new EjectCube());
	}
}