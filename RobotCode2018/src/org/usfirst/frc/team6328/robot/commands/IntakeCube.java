package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.Intake.GrabState;

import edu.wpi.first.wpilibj.command.Command;

public class IntakeCube extends Command {
	
	private boolean enableSensor;
	
	public IntakeCube(boolean enableSensor) {
		super("IntakeCube");
		requires(Robot.intake);
		this.enableSensor = enableSensor;
	}
	
	@Override
	protected void initialize() {
		Robot.intake.setGrabState(GrabState.WEAK);
		Robot.intake.intake();
	}
	
	@Override
	protected boolean isFinished() {
		return enableSensor && Robot.intake.getSensor();
	}
}