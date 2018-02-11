package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import openrio.powerup.MatchData;
import openrio.powerup.MatchData.GameFeature;
import openrio.powerup.MatchData.OwnedSide;

/**
 * Center auto
 */
public class CenterAuto extends InstantCommand {
	
	public CenterAuto() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		OwnedSide switchSide = MatchData.getOwnedSide(GameFeature.SWITCH_NEAR);
		Command autoCommand = null;
		switch (switchSide) {
		case LEFT:
			autoCommand = new LeftSwitch();
			break;
		case RIGHT:
			autoCommand = new RightSwitch();
			break;
		case UNKNOWN:
			break;
		default:
			break;
		}
		if (autoCommand != null) {
			autoCommand.start();
		}
	}

	private class LeftSwitch extends CommandGroup {
		public LeftSwitch() {
			addSequential(new RunMotionProfileOnRio("centerToLeftSwitch", false, true, false));
		}
	}
	
	private class RightSwitch extends CommandGroup {
		public RightSwitch() {
			addSequential(new RunMotionProfileOnRio("centerToRightSwitch", false, true, false));
		}
	}
}
