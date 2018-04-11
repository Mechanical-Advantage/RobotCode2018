package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.commands.WindSporkSlack.SlackType;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Deploys the spork
 */
public class DeploySpork extends CommandGroup {

	private static final double deployWait = 1.5;

	public DeploySpork() {
		addSequential(new ReleaseSolenoid());
		addSequential(new Delay(deployWait));
		addSequential(new ZeroEncoders());
		addSequential(new WindSporkSlack(SlackType.AFTER_DEPLOY));
	}

	private class ReleaseSolenoid extends InstantCommand {
		public ReleaseSolenoid() {
			super();
			// Use requires() here to declare subsystem dependencies
			// eg. requires(chassis);
			requires(Robot.spork.leftSpork);
			requires(Robot.spork.rightSpork);
		}

		// Called once when the command executes
		protected void initialize() {
			Robot.spork.releaseLock();
		}
	}

	private class ZeroEncoders extends InstantCommand {
		public ZeroEncoders() {
			super();
			requires(Robot.spork.leftSpork);
			requires(Robot.spork.rightSpork);
		}
		
		@Override
		protected void initialize() {
			Robot.spork.resetEncoders();
		}
	}
}
