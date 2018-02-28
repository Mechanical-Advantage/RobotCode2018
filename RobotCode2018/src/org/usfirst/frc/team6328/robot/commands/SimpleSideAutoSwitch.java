package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Simple auto to side of switch that goes in a straight line
 */
public class SimpleSideAutoSwitch extends InstantCommand {
	
	private static final double crossLineDistance = 130;
	
	private boolean leftSide;

	public SimpleSideAutoSwitch(boolean leftSide) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.leftSide = leftSide;
	}

	// Called once when the command executes
	protected void initialize() {
		OwnedSide switchSide = MatchData.getOwnedSide(GameFeature.SWITCH_NEAR);
		OwnedSide robotSide = leftSide ? OwnedSide.LEFT : OwnedSide.RIGHT;
		Command autoCommand;
		if (switchSide == robotSide) {
			autoCommand = new SwitchStraightDelivery();
		} else {
			autoCommand = new DriveDistanceOnHeading(crossLineDistance, 0);
		}
		autoCommand.start();
	}

	private class SwitchStraightDelivery extends CommandGroup {
		public SwitchStraightDelivery() {
			addSequential(new DriveDistanceOnHeading(crossLineDistance, 0));
			addSequential(new EjectCube());
		}
	}
}
