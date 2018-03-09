package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {

	private static final int configTimeout = 0;
	private static final NeutralMode brakeMode = NeutralMode.Brake;
	private static final int sensorGlitchFilter = 50000; // nanoseconds
	
	private double intakeSpeed;
	private boolean ejectSpeedLocked;
	private double ejectSpeed;
	private boolean intakeSpeedLocked;
	private boolean enableCurrentLimit;
	private int continuousCurrentLimit;
	private int peakCurrentLimit;
	private int peakCurrentLimitDuration; // Milliseconds
	private boolean invertLeft;
	private boolean invertRight;

	TalonSRX leftTalon;
	TalonSRX rightTalon;
	DoubleSolenoid retractSolenoid;
	DoubleSolenoid openSolenoid;
	DigitalInput proximitySensor;
	DigitalGlitchFilter sensorFilter;

	public Intake() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			proximitySensor = new DigitalInput(RobotMap.intakeSensor);
			sensorFilter = new DigitalGlitchFilter();
			sensorFilter.add(proximitySensor);
			sensorFilter.setPeriodNanoSeconds(sensorGlitchFilter);
			retractSolenoid = new DoubleSolenoid(RobotMap.intakeRetractPCM, RobotMap.intakeRetractSolenoid1, RobotMap.intakeRetractSolenoid2);
			openSolenoid = new DoubleSolenoid(RobotMap.intakeOpenPCM, RobotMap.intakeOpenSolenoid1, RobotMap.intakeOpenSolenoid2);
			intakeSpeed = 0.5;
			ejectSpeed = -1;
			intakeSpeedLocked = false;
			ejectSpeedLocked = false;
			invertLeft = false;
			invertRight = true;
			enableCurrentLimit = true;
			continuousCurrentLimit = 30;
			peakCurrentLimit = 50;
			peakCurrentLimitDuration = 50;

		}
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			intakeSpeed = 0.2;
			ejectSpeed = -1;
			intakeSpeedLocked = false;
			invertLeft = false;
			invertRight = true;
			enableCurrentLimit = true;
			continuousCurrentLimit = 30;
			peakCurrentLimit = 50;
			peakCurrentLimitDuration = 50;
		}
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 || RobotMap.robot == RobotType.EVERYBOT_2018) {
			leftTalon = new TalonSRX(RobotMap.intakeLeft);
			rightTalon = new TalonSRX(RobotMap.intakeRight);

			leftTalon.enableCurrentLimit(enableCurrentLimit);
			leftTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			leftTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			leftTalon.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			leftTalon.setInverted(invertLeft);
			leftTalon.setNeutralMode(brakeMode);
			rightTalon.enableCurrentLimit(enableCurrentLimit);
			rightTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			rightTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			rightTalon.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			rightTalon.setInverted(invertRight);
			rightTalon.setNeutralMode(brakeMode);
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
	
	public void periodic() {
		Robot.oi.updateLED(OILED.CUBE_SENSE_2, getSensor());
	}

	public void intake() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 || RobotMap.robot == RobotType.EVERYBOT_2018) {
			leftTalon.set(ControlMode.PercentOutput, intakeSpeedLocked ? intakeSpeed : Robot.oi.getIntakeLevel());
			rightTalon.set(ControlMode.PercentOutput, intakeSpeedLocked ? intakeSpeed : Robot.oi.getIntakeLevel());
		}
	}

	public void eject() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 || RobotMap.robot == RobotType.EVERYBOT_2018) {
			leftTalon.set(ControlMode.PercentOutput, ejectSpeedLocked ? ejectSpeed: Robot.oi.getEjectForce()*-1);
			rightTalon.set(ControlMode.PercentOutput, ejectSpeedLocked ? ejectSpeed: Robot.oi.getEjectForce()*-1);
		}
	}

	public void stop() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 || RobotMap.robot == RobotType.EVERYBOT_2018) {
			leftTalon.neutralOutput();
			rightTalon.neutralOutput();
		}
	}

	public boolean getSensor() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && Robot.oi.isCubeSensorEnabled()) {
			return proximitySensor.get();
		} else {
			return false;
		}
	}
	
	public void setRetracted(boolean retracted) {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			retractSolenoid.set(retracted ? Value.kReverse : Value.kForward);
		}
	}
	
	public boolean getRetracted() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return retractSolenoid.get() == Value.kReverse;
		}
		return false;
	}
	
	public void setOpen(boolean open) {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			openSolenoid.set(open ? Value.kForward : Value.kReverse);
			Robot.oi.updateLED(OILED.INTAKE_OPEN, open);
		}
	}
	
	public boolean getOpen() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return openSolenoid.get() == Value.kForward;
		}
		return false;
	}

	public double getCurrent() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 || RobotMap.robot == RobotType.EVERYBOT_2018) {
			return (leftTalon.getOutputCurrent() + rightTalon.getOutputCurrent()) / 2;
		} else {
			return 0;
		}
	}
}

