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
	private static final double crossLineDistance = 60;
	
	private boolean leftSide;
	private boolean switchFront;
	private boolean enableCross;

	public SideAutoSwitch(boolean leftSide, boolean switchFront, boolean enableCross) {
		super("SideAutoSwitch");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		this.leftSide = leftSide;
		this.switchFront = switchFront;
		this.enableCross = enableCross;
	}

	// Called once when the command executes
	protected void initialize() {
		OwnedSide switchSide = MatchData.getOwnedSide(GameFeature.SWITCH_NEAR);
		OwnedSide robotSide = leftSide ? OwnedSide.LEFT : OwnedSide.RIGHT;
		Command autoCommand;
		if (switchSide == robotSide) {
			if (switchFront) {
				autoCommand = new SameSideSwitchFront(leftSide);
			} else {
				autoCommand = new SameSideSwitchEnd(leftSide);
			}
		} else {
			if (enableCross) {
				autoCommand = new OppositeSideSwitch(leftSide);
			} else {
				autoCommand = new DriveDistanceOnHeading(crossLineDistance, 0);
			}
		}
		autoCommand.start();
	}

	private class SameSideSwitchEnd extends CommandGroup {
		public SameSideSwitchEnd(boolean leftSide) {
			addSequential(new RunMotionProfileOnRio("sideToSwitch", leftSide, true, false));
		}
	}
	
	private class SameSideSwitchFront extends CommandGroup {
		public SameSideSwitchFront(boolean leftSide) {
			addSequential(new RunMotionProfileOnRio("sideToSwitchFront", leftSide, true, false));
		}
	}
	
	private class OppositeSideSwitch extends CommandGroup {
		public OppositeSideSwitch(boolean leftSide) {
			addSequential(new RunMotionProfileOnRio("sideToOppositeSwitch", leftSide, true, false));
			double heading = leftSide ? -90 : 90;
			addSequential(new TurnToAngle(heading, true));
			addSequential(new DriveDistanceOnHeading(switchSideDriveDistance, heading));
		}
	}
}
