package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyException;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyPacket;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive to a cube
 */
public class DriveToCube extends Command {
	
	private static final double cameraHorizFOV = 75;
	private static final double cameraVertFOV = 47;
	private static final double cameraHeight = 24;
	private static final double cameraVertAngle = 11; // How far down the camera is pointed

    public DriveToCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.pixy);
    		requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		try {
				PixyPacket cube = Robot.pixy.readPacket(1);
				if (cube != null) {
					double angleH = (cube.X * (cameraHorizFOV/320)) - (cameraHorizFOV/2);
					double angleV = ((cube.Y + (cube.Height/2)) * (cameraVertFOV/200)) - (cameraVertFOV/2) - cameraVertAngle;
					double distance = Math.tan(Math.toRadians(90-Math.abs(angleV))) * cameraHeight;
					double horizDistance = Math.tan(Math.toRadians(angleH)) * distance;
					System.out.println("H: " + horizDistance);
					System.out.println("HA: " + angleH);
					System.out.println("D: " + distance);
					System.out.println("X: " + cube.X);
					System.out.println("Y: " + cube.Y);
				}
			} catch (PixyException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
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
