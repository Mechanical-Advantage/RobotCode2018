package org.usfirst.frc.team6328.robot.commands;

import org.usfirst.frc.team6328.robot.Robot;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyException;
import org.usfirst.frc.team6328.robot.subsystems.PixyI2C.PixyPacket;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive to a cube
 */
public class DriveToCube extends Command {
	
	private static final double cameraHorizFOV = 79.84523585564033;
	private static final double cameraHorizTan = Math.tan(Math.toRadians(cameraHorizFOV/2));
	private static final double cameraVertFOV = 47;
	private static final double cameraVertTan = Math.tan(Math.toRadians(cameraVertFOV/2));
	private static final double cameraHeight = 24.5;
	private static final double cameraVertAngle = 14; // How far down the camera is pointed
	private static final int cameraWidthPixels = 320; // pixels
	private static final int halfCameraWidthPixels = cameraWidthPixels/2;
	private static final int cameraHeightPixels = 200; // 0 at top
	private static final int halfCameraHeightPixels = cameraHeightPixels/2;

    public DriveToCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    		requires(Robot.pixy);
//    		requires(Robot.driveSubsystem);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    		try {
				PixyPacket cube = Robot.pixy.readPacket(1);
				if (cube != null) {
					// See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
					int cubeBottom = cube.Y + (cube.Height/2);
					double angleH = Math.toDegrees(
							Math.atan(((cube.X-halfCameraWidthPixels)*cameraHorizTan
									/halfCameraWidthPixels)));
					double angleV = (Math.toDegrees(
							Math.atan(((cubeBottom-halfCameraHeightPixels)*-1*cameraVertTan
									/halfCameraHeightPixels)))) - cameraVertAngle;
					double distance = Math.tan(Math.toRadians(90-Math.abs(angleV))) * cameraHeight;
					double horizDistance = Math.tan(Math.toRadians(angleH)) * distance;
					System.out.print("H: " + horizDistance);
					System.out.print(" HA: " + angleH);
					System.out.print(" D: " + distance);
					System.out.print(" VA: " + angleV);
					System.out.print(" X: " + cube.X);
					System.out.println(" Y: " + cubeBottom);
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
