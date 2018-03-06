package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.commands.SetElevatorPosition.ElevatorPosition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Center auto
 */
public class CenterAuto extends CommandGroup {
	public CenterAuto(OwnedSide switchSide) {
		if (switchSide == OwnedSide.LEFT || switchSide == OwnedSide.RIGHT) {
			String sideString = switchSide.toString().toLowerCase();
			sideString = sideString.replace("l", "L");
			sideString = sideString.replace("r", "R");
			addParallel(new SetElevatorPosition(ElevatorPosition.SWITCH));
			addSequential(new RunMotionProfileOnRio("centerTo" + sideString + "Switch", false, true, false, true));
			addSequential(new EjectCube());
		}
	}
}
