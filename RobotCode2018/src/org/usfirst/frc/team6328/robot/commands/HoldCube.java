package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Intake.GrabState;

import edu.wpi.first.wpilibj.command.Command;

public class HoldCube extends Command {
	public HoldCube() {
		super("HoldCube");
		requires(Robot.intake);
	}
	
	@Override
	protected void initialize() {
		Robot.intake.setGrabState(GrabState.STRONG);
		Robot.intake.stop();
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}
}
