package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.ArmJoystickControl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
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
	
	TalonSRX armTalon;
	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new MySpecialCommand());
		setDefaultCommand(new ArmJoystickControl());
	}

	public ScoringArm() {
		if (RobotMap.robot == RobotType.EVERYBOT_2018) {
			armTalon = new TalonSRX(RobotMap.scoringArm);
			armTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
			armTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
		}
	}

	public void moveArm(double speed) {
		if(RobotMap.robot == RobotType.EVERYBOT_2018)
			armTalon.set(ControlMode.PercentOutput, speed);
	}

	public boolean getForwardLimit() {
		if(RobotMap.robot == RobotType.EVERYBOT_2018)
			return armTalon.getSensorCollection().isFwdLimitSwitchClosed();
		return false;
	}

	public boolean getBackwardLimit() {
		if(RobotMap.robot == RobotType.EVERYBOT_2018)
			return armTalon.getSensorCollection().isRevLimitSwitchClosed();
		return false;
	}
}

