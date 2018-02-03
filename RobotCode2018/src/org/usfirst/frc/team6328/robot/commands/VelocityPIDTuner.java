package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.TunableNumber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An auto command for tuning velocity PIDs on the drive train
 */
public class VelocityPIDTuner extends Command {
	
	private static final boolean spin = false;
	
	private TunableNumber P = new TunableNumber("Drive PID/P");
	private TunableNumber I = new TunableNumber("Drive PID/I");
	private TunableNumber D = new TunableNumber("Drive PID/D");
	private TunableNumber F = new TunableNumber("Drive PID/F");
	private TunableNumber setpoint = new TunableNumber("Drive PID/setpoint");

	public VelocityPIDTuner() {
		super("VelocityPIDTuner");
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveSubsystem);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		P.setDefault(Robot.driveSubsystem.getP());
		I.setDefault(Robot.driveSubsystem.getI());
		D.setDefault(Robot.driveSubsystem.getD());
		F.setDefault(Robot.driveSubsystem.getF());
		setpoint.setDefault(0);
		SmartDashboard.putBoolean("Drive PID/enabled", false);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		Robot.driveSubsystem.setP(P.get());
		Robot.driveSubsystem.setI(I.get());
		Robot.driveSubsystem.setD(D.get());
		Robot.driveSubsystem.setF(F.get());
		if (SmartDashboard.getBoolean("Drive PID/enabled", false)) {
			Robot.driveSubsystem.driveInchesPerSec(setpoint.get(), setpoint.get() * (spin ? -1 : 1));
			SmartDashboard.putNumber("Drive velocity", (Robot.driveSubsystem.getVelocityLeft() + Robot.driveSubsystem.getVelocityRight())/2);
		} else {
			Robot.driveSubsystem.stop();
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
