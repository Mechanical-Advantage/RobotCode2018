package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
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
		Robot.intake.setRetracted(false);
		Robot.intake.intake();
		Robot.oi.updateLED(OILED.INTAKE_ON, true);
		Robot.oi.updateLED(OILED.INTAKE_OFF, false);
		Robot.oi.updateLED(OILED.INTAKE_RETRACT, false);
	}
	
	@Override
	protected boolean isFinished() {
		return enableSensor && Robot.intake.getSensor();
	}
	
	@Override
	protected void end() {
		Robot.intake.stop();
		Robot.oi.updateLED(OILED.INTAKE_ON, false);
		Robot.oi.updateLED(OILED.INTAKE_OFF, true);
	}
	
	@Override
	protected void interrupted() {
		end();
	}
}
