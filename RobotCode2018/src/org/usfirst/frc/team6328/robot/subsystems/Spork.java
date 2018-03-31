package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Mechanism to lift another robot while climbing
 */
public class Spork extends Subsystem {
	
	private static final int configTimeout = 0;
	private static final boolean enableCurrentLimit = false;
	private static final int continuousCurrentLimit = 35;
	private static final int peakCurrentLimit = 50;
	private static final int peakCurrentLimitDuration = 2000; // ms
	private static final FeedbackDevice encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
	private static final int leftStartingPosition = 0; // Ticks
	private static final int rightStartingPosition = 0; // Ticks
	private static final boolean reverseSensorLeft = false;
	private static final boolean reverseSensorRight = false;
	private static final boolean reverseOutputLeft = false;
	private static final boolean reverseOutputRight = false;
	private static final double liftSpeed = 0.7;
	private static final double resetSpeed = 0.5;
	private static final int resetTarget = 0;
	private static final int resetTolerance = 100;
	private static final int sideDifferenceTolerance = 0;

	private TalonSRX leftTalon;
	private TalonSRX rightTalon;
	private DoubleSolenoid lockSolenoid;
	private boolean leftTalonEnabled = true;
	private boolean rightTalonEnabled = true;
	private double setpoint;
	
	public Spork() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			leftTalon = new TalonSRX(RobotMap.sporkLeft);
			rightTalon = new TalonSRX(RobotMap.sporkRight);
			lockSolenoid = new DoubleSolenoid(RobotMap.sporkLockPCM, RobotMap.sporkLockSolenoid1, 
					RobotMap.sporkLockSolenoid2);
			
			leftTalon.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
			rightTalon.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
			leftTalon.setSelectedSensorPosition(leftStartingPosition, 0, configTimeout);
			rightTalon.setSelectedSensorPosition(rightStartingPosition, 0, configTimeout);
			
			leftTalon.setInverted(reverseOutputLeft);
			rightTalon.setInverted(reverseOutputRight);
			leftTalon.setSensorPhase(reverseSensorLeft);
			rightTalon.setSensorPhase(reverseSensorRight);
			
			// Disable limits
			leftTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, 
					LimitSwitchNormal.Disabled, configTimeout);
			leftTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, 
					LimitSwitchNormal.Disabled, configTimeout);
			rightTalon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, 
					LimitSwitchNormal.Disabled, configTimeout);
			rightTalon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, 
					LimitSwitchNormal.Disabled, configTimeout);
			leftTalon.configForwardSoftLimitEnable(false, configTimeout);
			leftTalon.configReverseSoftLimitEnable(false, configTimeout);
			rightTalon.configForwardSoftLimitEnable(false, configTimeout);
			rightTalon.configReverseSoftLimitEnable(false, configTimeout);
			
			// Current Limiting
			leftTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			leftTalon.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			leftTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			leftTalon.enableCurrentLimit(enableCurrentLimit);
			rightTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			rightTalon.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			rightTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			rightTalon.enableCurrentLimit(enableCurrentLimit);
		}
	}
	
	@Override
	public void periodic() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			int leftPosition = leftTalon.getSelectedSensorPosition(0);
			int rightPosition = rightTalon.getSelectedSensorPosition(0);
			if (leftPosition >= rightPosition + sideDifferenceTolerance) {
				leftTalonEnabled = false;
				rightTalonEnabled = true;
				updateTalonSetpoints();
			} else if (rightPosition >= leftPosition + sideDifferenceTolerance) {
				rightTalonEnabled = false;
				leftTalonEnabled = true;
				updateTalonSetpoints();
			} else {
				rightTalonEnabled = true;
				leftTalonEnabled = true;
				updateTalonSetpoints();
			}
		}
	}
	
	private void updateTalonSetpoints() {
		if (leftTalonEnabled) {
			leftTalon.set(ControlMode.PercentOutput, setpoint);
		} else {
			leftTalon.neutralOutput();
		}
		if (rightTalonEnabled) {
			rightTalon.set(ControlMode.PercentOutput, setpoint);
		} else {
			rightTalon.neutralOutput();
		}
	}
	
	public void releaseLock() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			lockSolenoid.set(Value.kReverse);
		}
	}
	
	public void engageLock() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			lockSolenoid.set(Value.kForward);
		}
	}
	
	public void lift() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && lockSolenoid.get() == Value.kReverse) {
			setpoint = liftSpeed;
			updateTalonSetpoints();
		}
	}
	
	public void reset() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && lockSolenoid.get() == Value.kReverse) {
			setpoint = resetSpeed;
			updateTalonSetpoints();
		}
	}
	
	public boolean isResetComplete() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return Math.abs(leftTalon.getSelectedSensorPosition(0)-resetTarget) <= resetTolerance && 
					Math.abs(rightTalon.getSelectedSensorPosition(0)-resetTarget) <= resetTolerance;
		}
		return false;
	}

	public void stop() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			leftTalon.neutralOutput();
			rightTalon.neutralOutput();
		}
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
}

