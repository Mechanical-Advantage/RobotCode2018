package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.OI.OILED;
import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.JoystickElevatorControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Elevator subsystem
 */
public class Elevator extends Subsystem {
	
	private static final double kPLow = 0;
	private static final double kILow = 0;
	private static final double kDLow = 0;
	private static final double kFLow = 0;
	private static final int kIZoneLow = 0;
	private static final double kPHigh = 0;
	private static final double kIHigh = 0;
	private static final double kDHigh = 0;
	private static final double kFHigh = 0;
	private static final int kIZoneHigh = 0;
	private static final double distancePerRotation = 180/25.4;
	private static final int ticksPerRotation = 4096;
	private static final int cruiseVelocity = 0; // in/s
	private static final double acceleration = 0; // in/s/s
	private static final int topSoftLimit = /*58073*/57800; // Native units
	private static final int bottomSoftLimit = 250; // Native units
	private static final double nominalOutput = 0; // Percent
	private static final boolean enableCurrentLimit = false;
	private static final int continuousCurrentLimit = 0;
	private static final int peakCurrentLimit = 0;
	private static final int peakCurrentLimitDuration = 0; // Milliseconds
	private static final double allowableError = 2;
	private static final int configTimeout = 0;
	private static final double maxClimbLockHeight = 0; // Maximum height climb lock should be allowed to engage at
	private static final double resetSpeed = -0.1;
	private static final int slowTopPoint = topSoftLimit-7000; // Native units
	private static final int slowBottomPoint = 7000;
	private static final double slowLimitSpeed = 0.2;
	private static final double LEDRange = 2; // Range on both sides of position that LED turns on in
	private static final int startingHeight = 22375; // Ticks
	private static final FeedbackDevice encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
	private static final NeutralMode brakeMode = NeutralMode.Brake;

	TalonSRX talonMaster;
	TalonSRX talonSlave1;
	TalonSRX talonSlave2;
	TalonSRX talonSlave3;
	DoubleSolenoid climbLock;
	DoubleSolenoid brake;
	DoubleSolenoid gearSwitch;
	DigitalInput lowerLimit;
	double targetPosition;
	boolean resetCompleted = false;
	boolean limitEnabledLast = true;
	boolean elevatorEnabledLast = true;
	
	@SuppressWarnings("unused")
	public Elevator() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			talonMaster = new TalonSRX(RobotMap.elevatorMaster);
			talonSlave1 = new TalonSRX(RobotMap.elevatorSlave1);
			talonSlave2 = new TalonSRX(RobotMap.elevatorSlave2);
			talonSlave3 = new TalonSRX(RobotMap.elevatorSlave3);
//			climbLock = new DoubleSolenoid(RobotMap.elevatorStageLockPCM, RobotMap.elevatorStageLockSolenoid1, RobotMap.elevatorStageLockSolenoid2);
			brake = new DoubleSolenoid(RobotMap.elevatorBrakePCM, RobotMap.elevatorBrakeSolenoid1, RobotMap.elevatorBrakeSolenoid2);
			gearSwitch = new DoubleSolenoid(RobotMap.elevatorGearPCM, RobotMap.elevatorGearSolenoid1, RobotMap.elevatorGearSolenoid2);
			lowerLimit = new DigitalInput(RobotMap.elevatorLimitSwitch);
			
			talonSlave1.set(ControlMode.Follower, RobotMap.elevatorMaster);
			talonSlave2.set(ControlMode.Follower, RobotMap.elevatorMaster);
			talonSlave3.set(ControlMode.Follower, RobotMap.elevatorMaster);
			
			talonMaster.setInverted(false);
			talonSlave1.setInverted(false);
			talonSlave2.setInverted(true);
			talonSlave3.setInverted(true);
			
			// Current limit setup
			talonMaster.enableCurrentLimit(enableCurrentLimit);
			talonMaster.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			talonMaster.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			talonMaster.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			talonSlave1.enableCurrentLimit(enableCurrentLimit);
			talonSlave1.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			talonSlave1.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			talonSlave1.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			talonSlave2.enableCurrentLimit(enableCurrentLimit);
			talonSlave2.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			talonSlave2.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			talonSlave2.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			talonSlave3.enableCurrentLimit(enableCurrentLimit);
			talonSlave3.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			talonSlave3.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			talonSlave3.configPeakCurrentDuration(peakCurrentLimitDuration, configTimeout);
			
			talonMaster.configNominalOutputForward(nominalOutput, configTimeout);
			talonMaster.configNominalOutputReverse(nominalOutput, configTimeout);
			talonSlave1.configNominalOutputForward(nominalOutput, configTimeout);
			talonSlave1.configNominalOutputReverse(nominalOutput, configTimeout);
			talonSlave2.configNominalOutputForward(nominalOutput, configTimeout);
			talonSlave2.configNominalOutputReverse(nominalOutput, configTimeout);
			talonSlave3.configNominalOutputForward(nominalOutput, configTimeout);
			talonSlave3.configNominalOutputReverse(nominalOutput, configTimeout);
			
			talonMaster.configPeakOutputForward(1, configTimeout);
			talonMaster.configPeakOutputReverse(-1, configTimeout);
			talonSlave1.configPeakOutputForward(1, configTimeout);
			talonSlave1.configPeakOutputReverse(-1, configTimeout);
			talonSlave2.configPeakOutputForward(1, configTimeout);
			talonSlave2.configPeakOutputReverse(-1, configTimeout);
			talonSlave3.configPeakOutputForward(1, configTimeout);
			talonSlave3.configPeakOutputReverse(-1, configTimeout);
			
			setPID(0, kPLow, kILow, kDLow, kFLow, kIZoneLow);
			setPID(1, kPHigh, kIHigh, kDHigh, kFHigh, kIZoneHigh);
			
			talonMaster.configMotionCruiseVelocity((int) (cruiseVelocity/distancePerRotation*ticksPerRotation*10), configTimeout);
			talonMaster.configMotionAcceleration((int) (acceleration/distancePerRotation*ticksPerRotation*10), configTimeout);
			
			talonMaster.configForwardSoftLimitThreshold(topSoftLimit, configTimeout);
			talonMaster.configReverseSoftLimitThreshold(bottomSoftLimit, configTimeout);
			talonMaster.configForwardSoftLimitEnable(true, configTimeout);
			
			talonSlave1.configForwardSoftLimitEnable(false, configTimeout);
			talonSlave2.configForwardSoftLimitEnable(false, configTimeout);
			talonSlave3.configForwardSoftLimitEnable(false, configTimeout);
			talonSlave1.configReverseSoftLimitEnable(false, configTimeout);
			talonSlave2.configReverseSoftLimitEnable(false, configTimeout);
			talonSlave3.configReverseSoftLimitEnable(false, configTimeout);
			
			talonMaster.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonSlave1.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonSlave2.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonSlave3.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonMaster.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonSlave1.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonSlave2.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			talonSlave3.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.NormallyOpen, configTimeout);
			
			talonMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
			
			talonMaster.setNeutralMode(brakeMode);
			talonSlave1.setNeutralMode(brakeMode);
			talonSlave2.setNeutralMode(brakeMode);
			talonSlave3.setNeutralMode(brakeMode);
			
			talonMaster.configAllowableClosedloopError(0, (int) (allowableError/distancePerRotation*ticksPerRotation), configTimeout);
			talonMaster.configAllowableClosedloopError(1, (int) (allowableError/distancePerRotation*ticksPerRotation), configTimeout);
			
			brake.set(Value.kReverse);
//			climbLock.set(Value.kReverse);
			switchGear(ElevatorGear.HIGH);
			
			if (startingHeight == 0) {
				talonMaster.set(ControlMode.PercentOutput, resetSpeed);
				talonMaster.configReverseSoftLimitEnable(false, configTimeout);
			} else {
				talonMaster.setSelectedSensorPosition(startingHeight, 0, configTimeout);
				talonMaster.configReverseSoftLimitEnable(true, configTimeout);
				resetCompleted = true;
			}
		}
	}
	
	public void initLEDs() {
		Robot.oi.updateLED(OILED.ELEVATOR_HIGH_GEAR, true);
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new JoystickElevatorControl());
	}
	
	@SuppressWarnings("unused")
	@Override
	public void periodic() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && getLimitSwitch()) {
			resetCompleted = true;
			if (talonMaster.getMotorOutputPercent() < 0) {
				talonMaster.neutralOutput();
			}
			talonMaster.setSelectedSensorPosition(0, 0, configTimeout);
			if (resetCompleted && startingHeight == 0) {
				talonMaster.configReverseSoftLimitEnable(true, configTimeout);
			}
		}
		if (resetCompleted && Robot.oi.isElevatorLimitEnabled() && Robot.oi.isElevatorEnabled() && talonMaster.getSelectedSensorPosition(0) >= slowTopPoint && talonMaster.getMotorOutputPercent() >= slowLimitSpeed) {
			talonMaster.set(ControlMode.PercentOutput, slowLimitSpeed);
		} else if (resetCompleted && Robot.oi.isElevatorLimitEnabled() && Robot.oi.isElevatorEnabled() && talonMaster.getSelectedSensorPosition(0) <= slowBottomPoint && talonMaster.getMotorOutputPercent() <= slowLimitSpeed*-1) {
			talonMaster.set(ControlMode.PercentOutput, slowLimitSpeed*-1);
		}
		
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && Robot.oi.isElevatorLimitEnabled() != limitEnabledLast && resetCompleted) {
			talonMaster.overrideSoftLimitsEnable(Robot.oi.isElevatorLimitEnabled());
			limitEnabledLast = Robot.oi.isElevatorLimitEnabled();
		}
		
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			if (Robot.oi.isElevatorEnabled() != elevatorEnabledLast) {
				if (!Robot.oi.isElevatorEnabled()) {
					talonMaster.neutralOutput();
				}
				elevatorEnabledLast = Robot.oi.isElevatorEnabled();
			}
			
			/*for (int i = 0; i < ElevatorPosition.values().length; i++) {
				ElevatorPosition position = ElevatorPosition.values()[i];
				OILED led = position.getLED();
				if (led != null) {
					Robot.oi.updateLED(led, Math.abs(getPosition()-getPositionTarget(position)) <= LEDRange);
				}
			}*/
			if (RobotMap.tuningMode) {
				/*SmartDashboard.putNumber("Elevator Master Current", talonMaster.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Slave 1 Current", talonSlave1.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Slave 2 Current", talonSlave2.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Slave 3 Current", talonSlave3.getOutputCurrent());
				SmartDashboard.putNumber("Elevator Master Out Voltage", talonMaster.getMotorOutputVoltage());
				SmartDashboard.putNumber("Elevator Slave 1 Out Voltage", talonSlave1.getMotorOutputVoltage());
				SmartDashboard.putNumber("Elevator Slave 2 Out Voltage", talonSlave2.getMotorOutputVoltage());
				SmartDashboard.putNumber("Elevator Slave 3 Out Voltage", talonSlave3.getMotorOutputVoltage());
				SmartDashboard.putNumber("Elevator Master Input Voltage", talonMaster.getBusVoltage());
				SmartDashboard.putNumber("Elevator Slave 1 Input Voltage", talonSlave1.getBusVoltage());
				SmartDashboard.putNumber("Elevator Slave 2 Input Voltage", talonSlave2.getBusVoltage());
				SmartDashboard.putNumber("Elevator Slave 3 Input Voltage", talonSlave3.getBusVoltage());*/
			}
		}
	}
	
	private void setPID(int slotIdx, double p, double i, double d, double f, int iZone) {
		talonMaster.config_kP(slotIdx, p, configTimeout);
		talonMaster.config_kI(slotIdx, i, configTimeout);
		talonMaster.config_kD(slotIdx, d, configTimeout);
		talonMaster.config_kF(slotIdx, f, configTimeout);
		talonMaster.config_IntegralZone(slotIdx, iZone, configTimeout);
	}
	
	/**
	 * Set the target position of the elevator
	 * @param position Target position in inches
	 * @return Whether the target position was actually applied
	 */
	public boolean setPosition(double position) {
		if (resetCompleted && RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && Robot.oi.isElevatorEnabled()) {
			brake.set(Value.kReverse);
			Robot.oi.updateLED(OILED.ELEVATOR_BRAKE, false);
			talonMaster.set(ControlMode.MotionMagic, position/distancePerRotation*ticksPerRotation);
			targetPosition = position;
			return true;
		} else {
			return false;
		}
	}
	
	public double getPosition() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return (double) talonMaster.getSelectedSensorPosition(0)/(double) ticksPerRotation*distancePerRotation;
		}
		return 0;
	}
	
	public boolean onTarget() {
		return onTarget(targetPosition);
	}

	public boolean onTarget(double target) {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return Math.abs(getPosition()-target) <= allowableError;
		}
		return false;
	}
	
	/**
	 * Cause the elevator to stop motor output and engage brake.
	 * @return Whether the hold actually applied
	 */
	public boolean holdPosition() {
		if (resetCompleted && RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			talonMaster.neutralOutput();
			brake.set(Value.kForward);
			Robot.oi.updateLED(OILED.ELEVATOR_BRAKE, true);
			return true;
		} else {
			return false;
		}
	}
	
	/**
	 * Set a percent speed for the elevator
	 * @param percent The percent speed
	 * @return Whether the change succeeded
	 */
	public boolean driveOpenLoop(double percent) {
		if (resetCompleted && RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && !(getLimitSwitch() && percent < 0) && Robot.oi.isElevatorEnabled()) {
			brake.set(Value.kReverse);
			Robot.oi.updateLED(OILED.ELEVATOR_BRAKE, false);
			if (talonMaster.getSelectedSensorPosition(0) >= slowTopPoint && percent >= slowLimitSpeed && Robot.oi.isElevatorLimitEnabled()) {
				talonMaster.set(ControlMode.PercentOutput, slowLimitSpeed);
			} else if (talonMaster.getSelectedSensorPosition(0) <= slowBottomPoint && percent <= slowLimitSpeed*-1 && Robot.oi.isElevatorLimitEnabled()) {
				talonMaster.set(ControlMode.PercentOutput, slowLimitSpeed*-1);
			} else {
				talonMaster.set(ControlMode.PercentOutput, percent);
			}
			return true;
		} else {
			return false;
		}
	}

	public void switchGear(ElevatorGear gear) {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			switch (gear) {
			case HIGH:
				gearSwitch.set(Value.kForward);
				talonMaster.selectProfileSlot(1, 0);
				if (Robot.oi != null) {
					Robot.oi.updateLED(OILED.ELEVATOR_HIGH_GEAR, true);
					Robot.oi.updateLED(OILED.ELEVATOR_LOW_GEAR, false);
				}
				break;
			case LOW:
				gearSwitch.set(Value.kReverse);
				talonMaster.selectProfileSlot(0, 0);
				if (Robot.oi != null) {
					Robot.oi.updateLED(OILED.ELEVATOR_HIGH_GEAR, false);
					Robot.oi.updateLED(OILED.ELEVATOR_LOW_GEAR, true);
				}
				break;
			default:
				break;
			}
		}
	}
	
	/**
	 * Engage or disengage the climb lock. Elevator must be near bottom.
	 * @param enable Whether to enable the lock
	 * @return Whether the change succeeded
	 */
	public boolean enableClimbLock(boolean enable) {
		if (getPosition() <= maxClimbLockHeight && RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			climbLock.set(enable ? Value.kForward : Value.kReverse);
			return true;
		} else {
			return false;
		}
	}
	
	public boolean getLimitSwitch() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && Robot.oi.isElevatorLimitEnabled()) {
			return !lowerLimit.get();
		}
		return false;
	}
	
	public double getPositionTarget(ElevatorPosition position) {
		switch (position) {
		case CLIMB_GRAB:
			return 86;
		case GROUND:
			return 0;
		case SCALE_HIGH:
			return (double) 56375/(double) ticksPerRotation*distancePerRotation;
		case SCALE_LOW:
			return (double) 43414/(double) ticksPerRotation*distancePerRotation;
		case SCALE_MID:
			return 62;
		case SWITCH:
			return (double) 18469/(double) ticksPerRotation*distancePerRotation;
		case DRIVE:
			return (double) 22121/(double) ticksPerRotation*distancePerRotation;
		default:
			return 0;
		}
	}
	
	public void setDisabledSlave(int slave) {
		switch (slave) {
		case 1:
			enableSlave(2);
			enableSlave(3);
			disableSlave(1);
			break;
		case 2:
			enableSlave(1);
			enableSlave(3);
			disableSlave(2);
			break;
		case 3:
			enableSlave(1);
			enableSlave(2);
			disableSlave(3);
			break;
		}
	}
	
	public void enableAllSlaves() {
		enableSlave(1);
		enableSlave(2);
		enableSlave(3);
	}
	
	private TalonSRX getSlave(int slave) {
		switch (slave) {
		case 1:
			return talonSlave1;
		case 2:
			return talonSlave2;
		case 3:
			return talonSlave3;
		}
		return null;
	}
	
	private void enableSlave(int slave) {
		TalonSRX talonSlave = getSlave(slave);
		talonSlave.setNeutralMode(brakeMode);
		talonSlave.follow(talonMaster);
	}
	
	private void disableSlave(int slave) {
		TalonSRX talonSlave = getSlave(slave);
		talonSlave.setNeutralMode(NeutralMode.Coast);
		talonSlave.neutralOutput();
	}
	
	public enum ElevatorGear {
		LOW, HIGH
	}
	
	public enum ElevatorPosition {
		GROUND, SWITCH, DRIVE, SCALE_LOW, SCALE_MID, SCALE_HIGH, CLIMB_GRAB;
		
		public OILED getLED() {
			switch (this) {
			case CLIMB_GRAB:
				return OILED.CLIMB_GRAB;
			case GROUND:
				return OILED.ELEVATOR_GROUND;
			case SCALE_HIGH:
				return OILED.ELEVATOR_SCALE_HIGH;
			case SCALE_LOW:
				return OILED.ELEVATOR_SCALE_LOW;
			case SCALE_MID:
				return null;
			case SWITCH:
				return OILED.ELEVATOR_SWITCH;
			case DRIVE:
				return OILED.ELEVATOR_DRIVE;
			default:
				return null;
			}
		}
	}
}

