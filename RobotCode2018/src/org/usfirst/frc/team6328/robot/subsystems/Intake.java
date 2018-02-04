package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.IntakeAndHoldCube;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Intake subsystem
 */
public class Intake extends Subsystem {
	
	private static final boolean enableCurrentLimit = false;
	private static final int continuousCurrentLimit = 0;
	private static final int peakCurrentLimit = 0;
	private static final int peakCurrentLimitDuration = 0; // Milliseconds
	private static final boolean invertLeft = false;
	private static final boolean invertRight = true;
	private static final double intakeSpeed = 0; // Should be negative
	private static final double ejectSpeed = 0;
	private static final int configTimeout = 0;
	private static final NeutralMode brakeMode = NeutralMode.Brake;

	TalonSRX leftTalon;
	TalonSRX rightTalon;
	DoubleSolenoid weakGrab1;
	DoubleSolenoid weakGrab2;
	Solenoid strongGrab;
	DigitalInput proximitySensor;
	
	public Intake() {
		if (RobotMap.robot == RobotType.ROBOT_2018) {
			leftTalon = new TalonSRX(RobotMap.intakeLeft);
			rightTalon = new TalonSRX(RobotMap.intakeRight);
			weakGrab1 = new DoubleSolenoid(RobotMap.intakeWeak1PCM, RobotMap.intakeWeak1Solenoid1, RobotMap.intakeWeak1Solenoid2);
			weakGrab2 = new DoubleSolenoid(RobotMap.intakeWeak2PCM, RobotMap.intakeWeak2Solenoid1, RobotMap.intakeWeak2Solenoid2);
			strongGrab = new Solenoid(RobotMap.intakeStrongPCM, RobotMap.intakeStrongSolenoid);
			proximitySensor = new DigitalInput(RobotMap.intakeSensor);
			
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
		setDefaultCommand(new IntakeAndHoldCube());
	}
	
	public void intake() {
		leftTalon.set(ControlMode.PercentOutput, intakeSpeed);
		rightTalon.set(ControlMode.PercentOutput, intakeSpeed);
	}
	
	public void eject() {
		leftTalon.set(ControlMode.PercentOutput, ejectSpeed);
		rightTalon.set(ControlMode.PercentOutput, ejectSpeed);
	}
	
	public void stop() {
		leftTalon.neutralOutput();
		rightTalon.neutralOutput();
	}
	
	public boolean getSensor() {
		return proximitySensor.get();
	}
	
	public void setGrabState(GrabState state) {
		switch(state) {
		case RETRACTED:
			weakGrab1.set(Value.kReverse);
			weakGrab2.set(Value.kReverse);
			strongGrab.set(false);
			break;
		case STRONG:
			weakGrab1.set(Value.kForward);
			weakGrab2.set(Value.kForward);
			strongGrab.set(true);
			break;
		case WEAK:
			weakGrab1.set(Value.kForward);
			weakGrab2.set(Value.kForward);
			strongGrab.set(false);
			break;
		default:
			break;
		}
	}
	
	public double getCurrent() {
		return (leftTalon.getOutputCurrent() + rightTalon.getOutputCurrent()) / 2;
	}
	
	public enum GrabState {
		RETRACTED, WEAK, STRONG
	}
}

