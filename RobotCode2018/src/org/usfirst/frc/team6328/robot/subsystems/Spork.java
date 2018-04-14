package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Mechanism to lift another robot while climbing
 */
public class Spork extends Subsystem {
	
	private static final int configTimeout = 0;
	private static final boolean enableCurrentLimit = true;
	private static final int continuousCurrentLimit = 30;
	private static final int peakCurrentLimit = 50;
	private static final int peakCurrentLimitDuration = 1000; // ms
	private static final FeedbackDevice encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
	private static final int leftStartingPosition = 0; // Ticks
	private static final int rightStartingPosition = 0; // Ticks
	private static final boolean reverseSensorLeft = false;
	private static final boolean reverseSensorRight = false;
	private static final boolean reverseOutputLeft = true;
	private static final boolean reverseOutputRight = false;
	private static final double liftSpeed = 0.7;
	private static final double retractSpeedSlow = -0.2;
	private static final double retractSpeedFast = -0.5;
	private static final int sideDifferenceScalar = 120; // Ticks, difference at which other side drops to zero
	private static final double rightPercent = 0.82; // Right percent of left adjustment
	// Slack parameters are measured from 0 (after deploy)
	private static final int rightDeploySlack = 0;
	private static final int leftDeploySlack = 0;
	private static final int rightLiftSlack = 0;
	private static final int leftLiftSlack = 0;
	private static final double slackWindSpeed = 0.5;

	private TalonSRX leftTalon;
	private TalonSRX rightTalon;
	private DoubleSolenoid lockSolenoid;
	private double leftTalonMultiplier = 1;
	private double rightTalonMultiplier = 1;
	private double leftSetpoint;
	private double rightSetpoint;
	private boolean enableSideLimits = true;
	
	public LeftSpork leftSpork = new LeftSpork();
	public RightSpork rightSpork = new RightSpork();
	
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
			
			lockSolenoid.set(Value.kForward);
		}
	}
	
	@Override
	public void periodic() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && DriverStation.getInstance().isEnabled()) {
			int leftPosition = leftTalon.getSelectedSensorPosition(0);
			int rightPosition = rightTalon.getSelectedSensorPosition(0);
			double lowPercent = Robot.map(Math.abs(rightPosition - leftPosition), 0, sideDifferenceScalar, 1, 0);
//			System.out.println("Motor difference is " + Math.abs(rightPosition - leftPosition));
			lowPercent = lowPercent < 0 ? 0 : lowPercent;
//			lowPercent *= lowPercent;
			if (enableSideLimits && leftPosition > rightPosition) {
				leftTalonMultiplier = lowPercent;
//				System.out.println("Multipling left side by " + lowPercent);
				rightTalonMultiplier = 1;
				updateTalonSetpoints();
			} else if (enableSideLimits && rightPosition > leftPosition) {
				rightTalonMultiplier = lowPercent;
//				System.out.println("Multipling right side by " + lowPercent);
				leftTalonMultiplier = 1;
				updateTalonSetpoints();
			} else {
				rightTalonMultiplier = 1;
				leftTalonMultiplier = 1;
				updateTalonSetpoints();
			}
		}
	}

	private void updateTalonSetpoints() {
		leftTalon.set(ControlMode.PercentOutput, leftSetpoint*leftTalonMultiplier);
		rightTalon.set(ControlMode.PercentOutput, rightSetpoint*rightPercent*rightTalonMultiplier);
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
	
	public boolean isDeployed() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return lockSolenoid.get() == Value.kReverse;
		}
		return false;
	}
	
	public void lift() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			leftSetpoint = liftSpeed;
			rightSetpoint = liftSpeed;
			enableSideLimits = true;
			updateTalonSetpoints();
		}
	}
	
	public void liftLeft() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			leftSetpoint = liftSpeed;
			enableSideLimits = false;
			updateTalonSetpoints();
		}
	}
	
	public void liftRight() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			rightSetpoint = liftSpeed;
			enableSideLimits = false;
			updateTalonSetpoints();
		}
	}
	
	public void retractSlowLeft() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			leftSetpoint = retractSpeedSlow;
			enableSideLimits = false;
			updateTalonSetpoints();
		}
	}
	public void retractSlowRight() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			rightSetpoint = retractSpeedSlow;
			enableSideLimits = false;
			updateTalonSetpoints();
		}
	}
	public void retractFastLeft() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			leftSetpoint = retractSpeedFast;
			enableSideLimits = false;
			updateTalonSetpoints();
		}
	}
	public void retractFastRight() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			rightSetpoint = retractSpeedFast;
			enableSideLimits = false;
			updateTalonSetpoints();
		}
	}
	
	public void windSlackLeft() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			leftSetpoint = slackWindSpeed;
			updateTalonSetpoints();
		}
	}
	public void windSlackRight() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && isDeployed()) {
			rightSetpoint = slackWindSpeed;
			updateTalonSetpoints();
		}
	}

	public void stop() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			leftSetpoint = 0;
			rightSetpoint = 0;
			updateTalonSetpoints();
		}
	}
	public void stopLeft() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			leftSetpoint = 0;
			updateTalonSetpoints();
		}
	}
	public void stopRight() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			rightSetpoint = 0;
			updateTalonSetpoints();
		}
	}
	
	public void resetEncoders() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			leftTalon.setSelectedSensorPosition(0, 0, configTimeout);
			rightTalon.setSelectedSensorPosition(0, 0, configTimeout);
		}
	}
	
	public boolean isLeftDeploySlackWound() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && 
				leftTalon.getSelectedSensorPosition(0) >= leftDeploySlack) {
			return true;
		}
		return false;
	}
	public boolean isRightDeploySlackWound() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && 
				rightTalon.getSelectedSensorPosition(0) >= rightDeploySlack) {
			return true;
		}
		return false;
	}
	public boolean isLeftLiftSlackWound() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && 
				leftTalon.getSelectedSensorPosition(0) >= leftLiftSlack) {
			return true;
		}
		return false;
	}
	public boolean isRightLiftSlackWound() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && 
				rightTalon.getSelectedSensorPosition(0) >= rightLiftSlack) {
			return true;
		}
		return false;
	}

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
	}
	
	// These subsystems do nothing but should be used as requirements so left and right are managed separately
	public static class LeftSpork extends Subsystem {
		
		private LeftSpork() {
			
		}

		@Override
		protected void initDefaultCommand() {
			
		}
	}
	public static class RightSpork extends Subsystem {
		
		private RightSpork() {
			
		}

		@Override
		protected void initDefaultCommand() {
			
		}
	}
}

