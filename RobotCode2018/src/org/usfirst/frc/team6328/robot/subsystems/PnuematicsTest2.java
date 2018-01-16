package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Test pnuematics
 */
public class PnuematicsTest2 extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	DoubleSolenoid solenoid;

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public PnuematicsTest2() {
    		if (RobotMap.robot == RobotType.ROBOT_2017) {
			solenoid = new DoubleSolenoid(RobotMap.gearExpellerSolenoid1, RobotMap.gearExpellerSolenoid2);
		}
    }
    
    public void extend() {
    		solenoid.set(DoubleSolenoid.Value.kForward);
    }
    
    public void retract() {
		solenoid.set(DoubleSolenoid.Value.kReverse);
    }
    
    public void neutral() {
    		solenoid.set(DoubleSolenoid.Value.kOff);
    }
}

