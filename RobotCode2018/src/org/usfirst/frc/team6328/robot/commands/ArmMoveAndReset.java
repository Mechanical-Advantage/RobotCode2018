package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;

/* @author: Nicholas Boone
 * @date: 2/12/18
 * This code will move the ScoringArm forward then backward using ThrowCube and ResetArm.
 */
public class ArmMoveAndReset extends CommandGroup {

	public ArmMoveAndReset() {
		super("ArmMoveAndReset");
		addSequential(new SetLED(true));
		addSequential(new ThrowCube());
		addSequential(new ResetArm());
		addSequential(new SetLED(false));
	}
	
	private class SetLED extends InstantCommand {
		
		private boolean on;
		
		public SetLED(boolean on) {
			this.on = on;
		}
		
		@Override
		protected void initialize() {
			Robot.oi.updateLED(OILED.CLIMB, on);
		}
	}
}
