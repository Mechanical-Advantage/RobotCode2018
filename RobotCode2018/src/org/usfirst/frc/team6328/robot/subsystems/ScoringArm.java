package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.ArmJoystickControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;


/*@author: Nicholas Boone
 *@date started: 2/11/18
 * Subsystem for the scoring arm for the switch. Will need:
 * -Two Limit Switches (up and down)
 * -connection to Robo-Rio or a Tallon
 *
 */
public class ScoringArm extends Subsystem {
	
	private static final boolean enableCurrentLimit = false;
	private static final int continuousCurrentLimit = 50;
	private static final int peakCurrentLimit = 100;
	private static final int peakCurrentDuration = 100;
	private static final NeutralMode brakeMode = NeutralMode.Brake;
	private static final int configTimeout = 0;
	
	TalonSRX armTalon;

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new ArmJoystickControl());
	}

	public ScoringArm() {
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			armTalon = new TalonSRX(RobotMap.scoringArm);
			armTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, configTimeout);
			armTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, configTimeout);
			armTalon.configContinuousCurrentLimit(continuousCurrentLimit, configTimeout);
			armTalon.configPeakCurrentLimit(peakCurrentLimit, configTimeout);
			armTalon.configPeakCurrentDuration(peakCurrentDuration, configTimeout);
			armTalon.enableCurrentLimit(enableCurrentLimit);
			armTalon.setNeutralMode(brakeMode);
		}
	}

	public void moveArm(double speed) {
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			armTalon.set(ControlMode.PercentOutput, speed);
		}
	}

	public boolean getForwardLimit() {
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			return armTalon.getSensorCollection().isFwdLimitSwitchClosed();
		}
		return false;
	}

	public boolean getBackwardLimit() {
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			return armTalon.getSensorCollection().isRevLimitSwitchClosed();
		}
		return false;
	}
}

