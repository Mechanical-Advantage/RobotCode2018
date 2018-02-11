package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Side auto to deliver a cube to the switch
 */
public class SideAutoSwitch extends InstantCommand {
	
	private static final double switchSideDriveDistance = 12;
	
	private boolean leftSide;

	public SideAutoSwitch(boolean leftSide) {
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
			autoCommand = new SameSideSwitch(leftSide);
		} else {
			autoCommand = new OppositeSideSwitch(leftSide);
		}
		autoCommand.start();
	}

	private class SameSideSwitch extends CommandGroup {
		public SameSideSwitch(boolean leftSide) {
			addSequential(new RunMotionProfileOnRio("sideToSwitch", leftSide, true, false));
		}
	}
	
	private class OppositeSideSwitch extends CommandGroup {
		public OppositeSideSwitch(boolean leftSide) {
			addSequential(new RunMotionProfileOnRio("sideToOppositeSwitch", leftSide, true, false));
			addSequential(new TurnToAngle(-90, true));
			addSequential(new DriveDistanceOnHeading(switchSideDriveDistance, -90));
		}
	}
}
