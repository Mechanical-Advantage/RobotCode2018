package org.usfirst.frc.team6328.robot.subsystems;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.RobotMap;
import org.usfirst.frc.team6328.robot.RobotMap.RobotType;
import org.usfirst.frc.team6328.robot.commands.DriveWithJoystick;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Robot Drive Train
 */
public class DriveTrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
	
	/*
	 * Talon SRX Unit Notes:
	 * Firmware in Phoenix does not support scaling
	 * 
	 * CTRE Mag Encoder Relative: 4096 ticks/native units per rotation
	 * Conversion factor: 6.8266 native units/100ms = 1rpm (for encoder with 4096 ticks only)

	 * Native units are ticks, native units in velocity is ticks per 100ms
	 * 
	 * See Talon SRX Software Reference Manual sections 17.1, 17.2
	 */
	
	/*
	 * Talon Slots:
	 * 0: Velocity Low Gear
	 * 1: Velocity High Gear
	 * 2: Motion profiling/magic
	 * Low gear used on single speed robots
	 */
		
	private double kPLow;
	private double kILow;
	private double kDLow;
	private double kFLow;
	private int kIZoneLow;
	private double kPHigh;
	private double kIHigh;
	private double kDHigh;
	private double kFHigh;
	private int kIZoneHigh;
	private double kPMP; // the MP settings are also used for distance close loop
	private double kIMP;
	private double kDMP;
	private double kFMP;
	private int kIZoneMP;
	private int kAllowableErrorDistance; // ticks sent to talon as allowable error for distance close loop
	
	private static final double sniperMode = 0.25; // multiplied by velocity in sniper mode
	private static final boolean sniperModeLocked = false; // when set, sniper mode uses value above, when unset, value comes from throttle control on joystick
	private static final int currentLimit = 50;
	private static final boolean enableCurrentLimit = false;
	@SuppressWarnings("unused")
	private static final int requiredTrajPoints = 5; // point to send to talon before starting profile
	private static final int configTimeout = 0;
	
	private TalonSRX rightTalonMaster;
	private TalonSRX rightTalonSlave;
	private TalonSRX rightTalonSlave2;
	private TalonSRX leftTalonMaster;
	private TalonSRX leftTalonSlave;
	private TalonSRX leftTalonSlave2;
	private DoubleSolenoid leftGearSolenoid;
	private DoubleSolenoid rightGearSolenoid;
	private FeedbackDevice encoderType;
	private int ticksPerRotation; // getEncPosition values in one turn
	private double wheelDiameter; // inches
	@SuppressWarnings("unused")
	private double wheelBaseWidth; // inches, distance between left and right wheels
	private boolean reverseSensorLeft;
	private boolean reverseSensorRight;
	private boolean reverseOutputLeft;
	private boolean reverseOutputRight;
	private double nominalOutputVoltage;
	private DriveControlMode currentControlMode = DriveControlMode.STANDARD_DRIVE; // enum defined at end of file
	private DriveGear currentGear;
//	private ProcessTalonMotionProfileBuffer processTalonMotionProfile = new ProcessTalonMotionProfileBuffer();
//	private Notifier processMotionProfileNotifier = new Notifier(processTalonMotionProfile);
//	private double motionProfileNotifierUpdateTime;
	
	public DriveTrain() {
		rightTalonMaster = new TalonSRX(RobotMap.rightMaster);
		rightTalonSlave = new TalonSRX(RobotMap.rightSlave);
		leftTalonMaster = new TalonSRX(RobotMap.leftMaster);
		leftTalonSlave = new TalonSRX(RobotMap.leftSlave);
		switch (RobotMap.robot) {
		case PRACTICE:
			encoderType = FeedbackDevice.QuadEncoder;
			ticksPerRotation = 1440;
			wheelDiameter = 5.9000000002; // 6
			wheelBaseWidth = 26.3; // 27.875
			reverseSensorRight = false;
			reverseSensorLeft = false;
			reverseOutputLeft = false;
			reverseOutputRight = true;
			kPLow = 2;
			kILow = 0;
			kDLow = 40;
			kFLow = 1.07;
			kIZoneLow = 0;
			kPMP = 2;
			kIMP = 0;
			kDMP = 40;
			kFMP = 1.0768;
			kIZoneMP = 0;
			kAllowableErrorDistance = 8;
			break;
		case ROBOT_2017:
			rightTalonSlave2 = new TalonSRX(RobotMap.rightSlave2);
			leftTalonSlave2 = new TalonSRX(RobotMap.leftSlave2);
			encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
			ticksPerRotation = 4096;
//			wheelDiameter = 4.1791666667; // before worlds, from end of build season
//			wheelDiameter = 4.24881941; // 7:48 AM worlds
			wheelDiameter = 4.0541069007; // Measured 1/27/2018
			reverseSensorRight = true;
			reverseSensorLeft = true;
			reverseOutputLeft = false;
			reverseOutputRight = true;
//			wheelBaseWidth = 22.5; // 18
			wheelBaseWidth = 18;
			kPLow = 0.6;
			kILow = 0.0007;
			kDLow = 6;
			kFLow = 0.2842;
			kIZoneLow = 4096*50/600;
			kPMP = 2;
			kIMP = 0.0007;
			kDMP = 45;
			kFMP = 0.2842;
			kIZoneMP = 4096*50/600;
			kAllowableErrorDistance = 24;
			break;
		case ORIGINAL_ROBOT_2018:
			encoderType = FeedbackDevice.CTRE_MagEncoder_Relative;
			ticksPerRotation = 4096;
			wheelDiameter = 4.25;
			reverseSensorRight = false;
			reverseSensorLeft = false;
			reverseOutputLeft = true;
			reverseOutputRight = false;
			kPLow = 0.5;
			kILow = 0.003;
			kIZoneLow = 300;
			kDLow = 30;
			kFLow = 0.3145756458;
			kPHigh = 0.8;
			kIHigh = 0;
			kDHigh = 10;
			kFHigh = 0.1449829932;
			nominalOutputVoltage = 0;
			leftGearSolenoid = new DoubleSolenoid(RobotMap.leftDriveGearPCM, RobotMap.leftDriveGearSolenoid1, RobotMap.leftDriveGearSolenoid2);
			rightGearSolenoid = new DoubleSolenoid(RobotMap.rightDriveGearPCM, RobotMap.rightDriveGearSolenoid1, RobotMap.rightDriveGearSolenoid2);
			setPID(1, kPHigh, kIHigh, kDHigh, kFHigh, kIZoneHigh);
			switchGear(DriveGear.HIGH);
			break;
		case EVERYBOT_2018:
			encoderType = FeedbackDevice.QuadEncoder;
			ticksPerRotation = 1440;
			wheelDiameter = 5.9000000002; // 6
			wheelBaseWidth = 26.3; // 27.875
			reverseSensorRight = false;
			reverseSensorLeft = false;
			reverseOutputLeft = false;
			reverseOutputRight = true;
			kPLow = 2;
			kILow = 0;
			kDLow = 40;
			kFLow = 1.07;
			kIZoneLow = 0;
			kPMP = 2;
			kIMP = 0;
			kDMP = 40;
			kFMP = 1.0768;
			kIZoneMP = 0;
			kAllowableErrorDistance = 8;
			break;
		default:
			break;
		}
		setPID(2, kPMP, kIMP, kDMP, kFMP, kIZoneMP);
		setPID(0, kPLow, kILow, kDLow, kFLow, kIZoneLow);
		rightTalonMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
//		rightTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f); // currently set in useClosedLoop so that motion profiling can change this
//		rightTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		rightTalonMaster.enableCurrentLimit(enableCurrentLimit);
		rightTalonMaster.configContinuousCurrentLimit(currentLimit, configTimeout);
		rightTalonMaster.configMotionCruiseVelocity(RobotMap.maxVelocityLow, configTimeout);
		rightTalonMaster.configMotionAcceleration(RobotMap.maxAcceleration, configTimeout);
		rightTalonMaster.setInverted(reverseOutputRight);
		rightTalonMaster.setSensorPhase(reverseSensorRight);
		rightTalonMaster.configNominalOutputForward(nominalOutputVoltage/12, configTimeout);
		rightTalonMaster.configNominalOutputReverse(nominalOutputVoltage/12*-1, configTimeout);
		rightTalonMaster.configPeakOutputForward(1, configTimeout);
		rightTalonMaster.configPeakOutputReverse(-1, configTimeout);
		leftTalonMaster.configSelectedFeedbackSensor(encoderType, 0, configTimeout);
//		leftTalonMaster.configNominalOutputVoltage(+0.0f, -0.0f);
//		leftTalonMaster.configPeakOutputVoltage(+12.0f, -12.0f);
		leftTalonMaster.enableCurrentLimit(enableCurrentLimit);
		leftTalonMaster.configContinuousCurrentLimit(currentLimit, configTimeout);
		leftTalonMaster.configMotionCruiseVelocity(RobotMap.maxVelocityLow, configTimeout);
		leftTalonMaster.configMotionAcceleration(RobotMap.maxAcceleration, configTimeout);
		leftTalonMaster.setInverted(reverseOutputLeft);
		leftTalonMaster.setSensorPhase(reverseSensorLeft);
		leftTalonMaster.configNominalOutputForward(nominalOutputVoltage/12, configTimeout);
		leftTalonMaster.configNominalOutputReverse(nominalOutputVoltage/12*-1, configTimeout);
		leftTalonMaster.configPeakOutputForward(1, configTimeout);
		leftTalonMaster.configPeakOutputReverse(-1, configTimeout);
		resetPosition();
		rightTalonSlave.set(ControlMode.Follower, RobotMap.rightMaster);
		rightTalonSlave.enableCurrentLimit(enableCurrentLimit);
		rightTalonSlave.configContinuousCurrentLimit(currentLimit, configTimeout);
		rightTalonSlave.setInverted(reverseOutputRight);
		leftTalonSlave.set(ControlMode.Follower, RobotMap.leftMaster);
		leftTalonSlave.enableCurrentLimit(enableCurrentLimit);
		leftTalonSlave.configContinuousCurrentLimit(currentLimit, configTimeout);
		leftTalonSlave.setInverted(reverseOutputLeft);
		if (RobotMap.robot == RobotType.ROBOT_2017){
			rightTalonSlave2.set(ControlMode.Follower, RobotMap.rightMaster);
			rightTalonSlave2.enableCurrentLimit(enableCurrentLimit);
			rightTalonSlave2.configContinuousCurrentLimit(currentLimit, configTimeout);
			rightTalonSlave2.setInverted(reverseOutputRight);
			leftTalonSlave2.set(ControlMode.Follower, RobotMap.leftMaster);
			leftTalonSlave2.enableCurrentLimit(enableCurrentLimit);
			leftTalonSlave2.configContinuousCurrentLimit(currentLimit, configTimeout);
			leftTalonSlave2.setInverted(reverseOutputLeft);
		}
		enableBrakeMode(true);
		
		/*rightTalonMaster.setSafetyEnabled(false);
		rightTalonSlave.setSafetyEnabled(false);
		leftTalonMaster.setSafetyEnabled(false);
		leftTalonSlave.setSafetyEnabled(false);*/
	}
		
//	@Override
//	public void periodic() {
//		if (RobotMap.tuningMode) {
////			SmartDashboard.putNumber("Drive Error", (rightTalonMaster.getClosedLoopError(0)+leftTalonMaster.getClosedLoopError(0))/2);
////			SmartDashboard.putNumber("Drive I accum", (rightTalonMaster.getIntegralAccumulator(0)+leftTalonMaster.getIntegralAccumulator(0))/2);
//		}
//	}

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithJoystick());
    }
    
    /**
     * Drive the robot with speed specified as inches per second
     * @param left Left inches per second
     * @param right Right inches per second
     */
    public void driveInchesPerSec(int left, int right) {
    		driveInchesPerSec((double)right, (double)left);
    }
    
    /**
     * Drive the robot with speed specified as inches per second
     * @param left Left inches per second
     * @param right Right inches per second
     */
    public void driveInchesPerSec(double left, double right) {
    		int maxVelocity;
		if (RobotMap.robot != RobotType.ORIGINAL_ROBOT_2018 || currentGear == DriveGear.LOW) {
			maxVelocity = RobotMap.maxVelocityLow;
		} else {
			maxVelocity = RobotMap.maxVelocityHigh;
		}
    		drive((left/(wheelDiameter*Math.PI))*ticksPerRotation/10/maxVelocity, (right/(wheelDiameter*Math.PI))*ticksPerRotation/10/maxVelocity);
    }
    
    private double calcActualVelocity(double input) {
    		int minVelocity;
		if (RobotMap.robot != RobotType.ORIGINAL_ROBOT_2018 || currentGear == DriveGear.LOW) {
			minVelocity = RobotMap.minVelocityLow;
		} else {
			minVelocity = RobotMap.minVelocityHigh;
		}
	    	if (input>=-0.1 && input<=0.1) {
	    		return 0;
	    	}
	    	else if (input>0.1 && input<minVelocity) {
	    		return minVelocity;
	    	}
	    	else if (input<-0.1 && input>minVelocity*-1) {
	    		return minVelocity*-1;
	    	}
	    	else {
	    		return input;
	    	}
    }
    
    /**
     * Make the robot drive
     * @param left Left percent speed
     * @param right Right percent speed
     */
    public void drive(double left, double right) {
    		drive(left, right, false);
    }
    
    /**
     * Make the robot drive
     * @param left Left percent speed
     * @param right Right percent speed
     * @param alwaysHighMaxVel Whether to always use the max velocity of high gear or of current gear
     */
    public void drive(double left, double right, boolean alwaysHighMaxVel) {
    		if (Robot.oi.getDriveEnabled() && currentControlMode == DriveControlMode.STANDARD_DRIVE) {
	    		if (Robot.oi.getSniperMode()) {
	    			if (sniperModeLocked) {
	    				left*=sniperMode;
	    				right*=sniperMode;
	    			} else {
	    				left*=Robot.oi.getSniperLevel();
	    				right*=Robot.oi.getSniperLevel();
	    			}
	    		}
	    		int maxVelocity;
	    		// actualMaxVelocity is used to make open loop scaling accurate
	    		int actualMaxVelocity;
	    		if (RobotMap.robot != RobotType.ORIGINAL_ROBOT_2018) {
	    			maxVelocity = RobotMap.maxVelocityLow;
	    			actualMaxVelocity = RobotMap.maxVelocityLow;
	    		}
	    		else if (currentGear == DriveGear.LOW) {
	    			if (!alwaysHighMaxVel) {
	    				maxVelocity = RobotMap.maxVelocityLow;
	    			} else {
	    				maxVelocity = RobotMap.maxVelocityHigh;
	    			}
	    			actualMaxVelocity = RobotMap.maxVelocityLow;
	    		} else {
	    			maxVelocity = RobotMap.maxVelocityHigh;
	    			actualMaxVelocity = RobotMap.maxVelocityHigh;
	    		}
	    		left*=maxVelocity;
	    		right*=maxVelocity;
	    		left = calcActualVelocity(left);
	    		right = calcActualVelocity(right);
	    		
	    		if (Robot.oi.getOpenLoop()) {
	    			rightTalonMaster.set(ControlMode.PercentOutput, right/actualMaxVelocity);
	    			leftTalonMaster.set(ControlMode.PercentOutput, left/actualMaxVelocity);
	    		} else {
	    			rightTalonMaster.set(ControlMode.Velocity, right);
	    			leftTalonMaster.set(ControlMode.Velocity, left);
	    		}
    		} else if (!Robot.oi.getDriveEnabled()) {
	    		stop();
	    	}
    }
    
    public void stop() {
	    	if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
	    		if (Robot.oi.getOpenLoop()) {
		    		rightTalonMaster.set(ControlMode.PercentOutput, 0);
		    		leftTalonMaster.set(ControlMode.PercentOutput, 0);
	    		} else {
	    			rightTalonMaster.set(ControlMode.Velocity, 0);
	    			leftTalonMaster.set(ControlMode.Velocity, 0);
	    		}
	    	}
    }
    
    public void enableBrakeMode(boolean enable) {
    		NeutralMode mode;
    		if (enable) {
    			mode = NeutralMode.Brake;
    		} else {
    			mode = NeutralMode.Coast;
    		}
    		rightTalonMaster.setNeutralMode(mode);
		leftTalonMaster.setNeutralMode(mode);
		rightTalonSlave.setNeutralMode(mode);
		leftTalonSlave.setNeutralMode(mode);
		if (RobotMap.robot == RobotType.ROBOT_2017) {
			rightTalonSlave2.setNeutralMode(mode);
			leftTalonSlave2.setNeutralMode(mode);
		}
    }
    
    public void resetPosition() {
    		rightTalonMaster.setSelectedSensorPosition(0, 0, configTimeout);
		leftTalonMaster.setSelectedSensorPosition(0, 0, configTimeout);
    }
    
    // values are cast to doubles to prevent integer division
    // If sensors are reversed, talon reverses reported value so don't need to reverse here
    public double getRotationsLeft() {
    	double rotLeft = (double)leftTalonMaster.getSelectedSensorPosition(0)/(double)ticksPerRotation;
    	return rotLeft;
    }
    
    public double getRotationsRight() {
    	double rotRight = (double)rightTalonMaster.getSelectedSensorPosition(0)/(double)ticksPerRotation;
    	return rotRight;
    }
    
    // diameter times pi equals circumference times rotations equals distance
    public double getDistanceRight() {
    		return wheelDiameter*Math.PI*getRotationsRight();
    }
    
    public double getDistanceLeft() {
    		return wheelDiameter*Math.PI*getRotationsLeft();
    }
    
    /**
     * Get the current velocity for the right side of the robot
     * @return current velocity in inches per second
     */
    public double getVelocityRight() {
    		return (double)rightTalonMaster.getSelectedSensorVelocity(0)/(double)ticksPerRotation*wheelDiameter*Math.PI*10;
    }
    
    /**
     * Get the current velocity for the left side of the robot
     * @return current velocity in inches per second
     */
    public double getVelocityLeft() {
    		return (double)leftTalonMaster.getSelectedSensorVelocity(0)/(double)ticksPerRotation*wheelDiameter*Math.PI*10;
    }
    
    // average current of left and right masters
    public double getCurrent() {
    		return (rightTalonMaster.getOutputCurrent()+leftTalonMaster.getOutputCurrent())/2;
    }
    
    /**
     * Sets the PID parameters for the current control mode, useful for tuning.
     * Calling this effects everything using the subsystem, use with care.
     * @param p P
     * @param i I
     * @param d D
     * @param f F
     * @param iZone Integral zone
     */
    public void setPID(double p, double i, double d, double f, int iZone) {
    		int slot;
    		if (currentControlMode == DriveControlMode.STANDARD_DRIVE && currentGear == DriveGear.LOW) {
    			slot = 0;
    		} else if (currentControlMode == DriveControlMode.STANDARD_DRIVE && currentGear == DriveGear.HIGH) {
    			slot = 1;
    		} else {
    			slot = 2;
    		}
    		setPID(slot, p, i, d, f, iZone);
    }
    
    /**
     * Sets the PID parameters for the given slot on the talon, used during setup
     * @param slotIdx Which slot to write to
     * @param p P
     * @param i I
     * @param d D
     * @param f F
     * @param iZone Integral zone
     */
    private void setPID(int slotIdx, double p, double i, double d, double f, int iZone) {
    		rightTalonMaster.config_kP(slotIdx, p, configTimeout);
		rightTalonMaster.config_kI(slotIdx, i, configTimeout);
		rightTalonMaster.config_kD(slotIdx, d, configTimeout);
		rightTalonMaster.config_kF(slotIdx, f, configTimeout);
		rightTalonMaster.config_IntegralZone(slotIdx, iZone, configTimeout);
		leftTalonMaster.config_kP(slotIdx, p, configTimeout);
		leftTalonMaster.config_kI(slotIdx, i, configTimeout);
		leftTalonMaster.config_kD(slotIdx, d, configTimeout);
		leftTalonMaster.config_kF(slotIdx, f, configTimeout);
		leftTalonMaster.config_IntegralZone(slotIdx, iZone, configTimeout);
    }
    
    public double getP() {
		return kPLow;
	}

	public double getI() {
		return kILow;
	}

	public double getD() {
		return kDLow;
	}

	public double getF() {
		return kFLow;
	}

	public void changeStatusRate(int ms) {
    		leftTalonMaster.setStatusFramePeriod(StatusFrame.Status_1_General, ms, configTimeout);
    		rightTalonMaster.setStatusFramePeriod(StatusFrame.Status_1_General, ms, configTimeout);
    }
    public void resetSensorRate() {
    		leftTalonMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 10, configTimeout);
		rightTalonMaster.setStatusFramePeriod(StatusFrame.Status_1_General, 10, configTimeout);
    }
    
    public void changeControlRate(int ms) {
		leftTalonMaster.setControlFramePeriod(ControlFrame.Control_3_General, ms);
		rightTalonMaster.setControlFramePeriod(ControlFrame.Control_3_General, ms);
	}
	public void resetControlRate() {
		leftTalonMaster.setControlFramePeriod(ControlFrame.Control_3_General, 10);
		rightTalonMaster.setControlFramePeriod(ControlFrame.Control_3_General, 10);
	}
	
	public void switchGear(DriveGear gear) {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018 && (Robot.oi == null || Robot.oi.isShiftingEnabled())) {
			switch (gear) {
			case HIGH:
				leftGearSolenoid.set(Value.kForward);
				rightGearSolenoid.set(Value.kForward);
				leftTalonMaster.selectProfileSlot(1, 0);
				rightTalonMaster.selectProfileSlot(1, 0);
				currentGear = DriveGear.HIGH;
				SmartDashboard.putBoolean("High Gear", true);
				break;
			case LOW:
				leftGearSolenoid.set(Value.kReverse);
				rightGearSolenoid.set(Value.kReverse);
				leftTalonMaster.selectProfileSlot(0, 0);
				rightTalonMaster.selectProfileSlot(0, 0);
				currentGear = DriveGear.LOW;
				SmartDashboard.putBoolean("High Gear", false);
				break;
			case UNSUPPORTED:
			default:
				break;
			}
		}
	}
	public DriveGear getCurrentGear() {
		if (RobotMap.robot == RobotType.ORIGINAL_ROBOT_2018) {
			return currentGear;
		} else {
			return DriveGear.UNSUPPORTED;
		}
	}
    
    
    /**
     * Uses the talon native distance close loop to drive a distance.
     * @param inches
     */
    public void driveDistance(double inches, boolean motionMagic) {
    	if (currentControlMode == DriveControlMode.STANDARD_DRIVE && Robot.oi.getDriveEnabled()) {
    		currentControlMode = DriveControlMode.DISTANCE_CLOSE_LOOP;
    		rightTalonMaster.selectProfileSlot(1, 0);
    		leftTalonMaster.selectProfileSlot(1, 0);
    		rightTalonMaster.configNominalOutputForward(0.08, configTimeout);
    		leftTalonMaster.configNominalOutputForward(0.08, configTimeout);
    		rightTalonMaster.configNominalOutputReverse(0.08, configTimeout);
    		leftTalonMaster.configNominalOutputReverse(0.08, configTimeout);
    		rightTalonMaster.configAllowableClosedloopError(1, kAllowableErrorDistance, configTimeout);
    		leftTalonMaster.configAllowableClosedloopError(1, kAllowableErrorDistance, configTimeout);
    		ControlMode mode;
    		if (motionMagic) {
    			mode = ControlMode.MotionMagic;
    		} else {
    			mode = ControlMode.Position;
    		}
    		rightTalonMaster.set(mode, inches/(Math.PI*wheelDiameter)*-1);
    		leftTalonMaster.set(mode, inches/(Math.PI*wheelDiameter));
    	}
    }

    public void stopDistanceDrive() {
    	if (currentControlMode == DriveControlMode.DISTANCE_CLOSE_LOOP) {
    		rightTalonMaster.neutralOutput();
    		leftTalonMaster.neutralOutput();
    		rightTalonMaster.configAllowableClosedloopError(1, 0, configTimeout); // motion profiling does not use this
    		leftTalonMaster.configAllowableClosedloopError(1, 0, configTimeout);
    		currentControlMode = DriveControlMode.STANDARD_DRIVE; // this needs to be changed before calling useClosed/OpenLoop so that they work
    		rightTalonMaster.selectProfileSlot(0, 0);
    		leftTalonMaster.selectProfileSlot(0, 0);
    		rightTalonMaster.configNominalOutputForward(nominalOutputVoltage/12, configTimeout);
    		leftTalonMaster.configNominalOutputForward(nominalOutputVoltage/12, configTimeout);
    	}
    }

//    
//    /**
//     * Loads the passed trajectory, and activates motion profiling mode
//     * @param trajectory
//     */
//    public void loadMotionProfile(Trajectory trajectory) {
//    	loadMotionProfile(trajectory, false);
//    }
//    /**
//     * Loads the passed trajectory, and activates motion profiling mode
//     * @param trajectory
//     */
//    public void loadMotionProfile(Trajectory trajectory, boolean flipLeftRight) {
//    	if (currentControlMode == DriveControlMode.STANDARD_DRIVE) {
//    		currentControlMode = DriveControlMode.MOTION_PROFILE;
//    		rightTalonMaster.changeControlMode(TalonControlMode.MotionProfile);
//    		leftTalonMaster.changeControlMode(TalonControlMode.MotionProfile);
//    		rightTalonMaster.set(TalonSRX.SetValueMotionProfile.Disable.value);
//    		leftTalonMaster.set(TalonSRX.SetValueMotionProfile.Disable.value);
//    		rightTalonMaster.configNominalOutputVoltage(+1.0f, -1.0f);
//    		leftTalonMaster.configNominalOutputVoltage(+1.0f, -1.0f);
//    		System.out.println(rightTalonMaster.GetNominalClosedLoopVoltage());
//
//    		TankModifier modifier = new TankModifier(trajectory);
//    		TrajectoryPoint[] leftTrajectory;
//    		TrajectoryPoint[] rightTrajectory;
//    		modifier.modify(wheelBaseWidth);
//    		if (flipLeftRight) {
//    			leftTrajectory = convertToTalonPoints(modifier.getLeftTrajectory(), false); // Pathfinder seems to return swapped left/right, so swap them back
//    			rightTrajectory = convertToTalonPoints(modifier.getRightTrajectory(), false);
//    		} else {
//    			leftTrajectory = convertToTalonPoints(modifier.getRightTrajectory(), false); // Pathfinder seems to return swapped left/right, so swap them back
//    			rightTrajectory = convertToTalonPoints(modifier.getLeftTrajectory(), false);
//    		}
//    		/*File leftFile = new File("/home/lvuser/lastMP/traj-left.csv");
//    		File rightFile = new File("/home/lvuser/lastMP/traj-right.csv");
//    		Pathfinder.writeToCSV(leftFile, modifier.getRightTrajectory()); // also swap here
//    		Pathfinder.writeToCSV(rightFile, modifier.getLeftTrajectory());
//    		File csvFile = new File("/home/lvuser/lastMP/trajectory.csv");
//    		Pathfinder.writeToCSV(csvFile, trajectory);*/
//    		motionProfileNotifierUpdateTime = trajectory.segments[0].dt/2;
//    		processMotionProfileNotifier.stop();
//    		processTalonMotionProfile.reset();
//    		rightTalonMaster.changeMotionControlFramePeriod((int) (motionProfileNotifierUpdateTime*1000));
//    		leftTalonMaster.changeMotionControlFramePeriod((int) (motionProfileNotifierUpdateTime*1000));
//    		rightTalonMaster.clearMotionProfileTrajectories();
//    		leftTalonMaster.clearMotionProfileTrajectories();
//    		for (int i = 0; i < trajectory.length(); i++) {
//    			rightTalonMaster.pushMotionProfileTrajectory(rightTrajectory[i]);
//    			leftTalonMaster.pushMotionProfileTrajectory(leftTrajectory[i]);
//    		}
//    	}
//    }
//    
//    public void startMotionProfile() {
//    	if (currentControlMode == DriveControlMode.MOTION_PROFILE && Robot.oi.getDriveEnabled()) {
//    		enable();
//    		processMotionProfileNotifier.startPeriodic(motionProfileNotifierUpdateTime);
//    	} else if (!Robot.oi.getDriveEnabled()) {
//    		disable();
//    	}
//    }
//
//    public void stopMotionProfile() {
//    	if (currentControlMode == DriveControlMode.MOTION_PROFILE) {
//    		processMotionProfileNotifier.stop();
//    		rightTalonMaster.set(TalonSRX.SetValueMotionProfile.Disable.value);
//    		leftTalonMaster.set(TalonSRX.SetValueMotionProfile.Disable.value);
//    		currentControlMode = DriveControlMode.STANDARD_DRIVE; // this needs to be changed before calling useClosed/OpenLoop so that they work
//    		if (Robot.oi.getOpenLoop()) {
//    			useOpenLoop();
//    		} else {
//    			useClosedLoop();
//    		}
//    	}
//    }
//    
//    /**
//     * Returns true if both sides have reached their last trajectory point
//     * @return
//     */
//    public boolean isMotionProfileComplete() {
//    	TalonSRX.MotionProfileStatus leftStatus = new TalonSRX.MotionProfileStatus();
//    	TalonSRX.MotionProfileStatus rightStatus = new TalonSRX.MotionProfileStatus();
//    	if (currentControlMode == DriveControlMode.MOTION_PROFILE) {
//    		rightTalonMaster.getMotionProfileStatus(rightStatus);
//    		leftTalonMaster.getMotionProfileStatus(leftStatus);
////    		System.out.println("isMotionProfileComplete " + (leftStatus.activePoint.isLastPoint && rightStatus.activePoint.isLastPoint
////    				&& processTalonMotionProfile.isProfileStarted()));
//    		return leftStatus.activePoint.isLastPoint && rightStatus.activePoint.isLastPoint
//    				&& processTalonMotionProfile.isProfileStarted();
//    	}
//    	return false; // if motion profiling not active
//    }
//    
//    private TrajectoryPoint[] convertToTalonPoints(Trajectory t, boolean invert) {
//    	TrajectoryPoint[] points = new TrajectoryPoint[t.length()];
//    	for (int i = 0; i < points.length; i++) {
//    		Segment s = t.get(i);
//    		TrajectoryPoint point = new TrajectoryPoint();
//    		point.position = s.position/(wheelDiameter*Math.PI);
//    		point.velocity = (s.velocity/(wheelDiameter*Math.PI)) * 60;
//    		point.timeDurMs = (int) (s.dt * 1000.0);
//    		point.profileSlotSelect = 1;
//    		point.velocityOnly = false;
//    		point.zeroPos = i == 0;
//    		point.isLastPoint = i == t.length() - 1;
//
//    		if (invert) {
//    			point.position = -point.position;
//    			point.velocity = -point.velocity;
//    		}
//
//    		points[i] = point;
//    	}
//    	return points;
//    }
//    
//    private class ProcessTalonMotionProfileBuffer implements Runnable {
//    	
//    	private boolean profileStarted = false;
//    	private boolean clearCompleted = false;
//    	private TalonSRX.MotionProfileStatus leftStatus = new TalonSRX.MotionProfileStatus();
//    	private TalonSRX.MotionProfileStatus rightStatus = new TalonSRX.MotionProfileStatus();
//    	
//    	public void reset() {
//    		System.out.println("Thread reset, previous clearCompleted: " + clearCompleted);
//    		profileStarted = false;
//    		clearCompleted = false;
//    	}
//    	
//    	private void startProfile() {
//    		rightTalonMaster.set(TalonSRX.SetValueMotionProfile.Enable.value);
//    		leftTalonMaster.set(TalonSRX.SetValueMotionProfile.Enable.value);
//    	}
//
//    	@Override
//    	public void run() {
//    		rightTalonMaster.getMotionProfileStatus(rightStatus);
//    		leftTalonMaster.getMotionProfileStatus(leftStatus);
//    		if (clearCompleted) {
//    			rightTalonMaster.processMotionProfileBuffer();
//    			leftTalonMaster.processMotionProfileBuffer();
//    			System.out.println("R- Bottom Buffer Points: " + rightStatus.btmBufferCnt + " Active Point pos: " + rightStatus.activePoint.position + " vel: " + rightStatus.activePoint.velocity);
//    			System.out.println("L- Bottom Buffer Points: " + leftStatus.btmBufferCnt + " Active Point pos: " + leftStatus.activePoint.position + " vel: " + leftStatus.activePoint.velocity);
//    			if (!profileStarted && rightStatus.btmBufferCnt>=requiredTrajPoints && leftStatus.btmBufferCnt>=requiredTrajPoints) {
//    				startProfile();
//    				profileStarted = true;
//    			}
//    		} else {
//    			System.out.println("Clear not completed " + leftStatus.btmBufferCnt);
//    			clearCompleted = leftStatus.btmBufferCnt == 0 && rightStatus.btmBufferCnt == 0;
//    			if (clearCompleted) {
//    				System.out.println("Before points pushed R - active point pos " + rightStatus.activePoint.position + " vel " 
//    						+ rightStatus.activePoint.velocity + " valid " + rightStatus.activePointValid + " bottom buffer " + rightStatus.btmBufferCnt);
//    				System.out.println("Before points pushed L - active point pos " + leftStatus.activePoint.position + " vel " 
//    						+ leftStatus.activePoint.velocity + " valid " + leftStatus.activePointValid + " bottom buffer " + leftStatus.btmBufferCnt);
//    			}
//    		}
//    	}
//
//		public boolean isProfileStarted() {
//			return profileStarted;
//		}
//    }
 
    
    private enum DriveControlMode {
    	STANDARD_DRIVE, MOTION_PROFILE, DISTANCE_CLOSE_LOOP
    }
    
    public enum DriveGear {
    		HIGH, LOW, UNSUPPORTED
    }
}

