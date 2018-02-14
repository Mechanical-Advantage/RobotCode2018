package org.usfirst.frc.team6328.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/* @author: Nicholas Boone
 * @date: 2/12/18
 * This code will move the ScoringArm forward then backward using ThrowCube and ResetArm.
 */
public class ArmMoveAndReset extends CommandGroup {

	public ArmMoveAndReset() {
		super("ArmMoveAndReset");
		addSequential(new ThrowCube());
		addSequential(new ResetArm());

	}
}
