package frc.robot.Commands.GameObjectTrackingCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class SimpleGoToObjectCommand extends Command {

    private DriveSubsystem driveSubsystem;

    private double xVelocity;
    private double yVelocity;
    private double[] targetPoseAndHeading;
    private boolean foundTarget = false;

    private Pose2d finalPose;

    public SimpleGoToObjectCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        // Find target position
        this.targetPoseAndHeading = GameObjectTracker.getTargetDistanceAndHeading(); // Returns hypotenuse, heading, x distance and y distance
        // Check that the pose isn't just zeroes
        this.foundTarget = !(
            this.targetPoseAndHeading[0] == 0 &&
            this.targetPoseAndHeading[1] == 0 &&
            this.targetPoseAndHeading[2] == 0 &&
            this.targetPoseAndHeading[3] == 0 
        );
        // Create position command
        if (foundTarget) {
            finalPose = new Pose2d(new Translation2d(this.targetPoseAndHeading[2],
                this.targetPoseAndHeading[3]),
                Rotation2d.fromRadians(this.targetPoseAndHeading[1]));
            if (finalPose.getX() > finalPose.getY()) { // If X > Y
                xVelocity = 1;
                yVelocity = finalPose.getY() / finalPose.getX(); // Y = Y/X
            }
            else { // If Y >= X
                xVelocity = finalPose.getX() / finalPose.getY(); //X = X/Y
                yVelocity = 1;
            }
        }
        
    }

    public double getTargetRotationalVelocity() {
        Pose2d robotPose = driveSubsystem.getRobotPose();
        //TODO: Swich To a Profiled PID Controller
        // Slow down as the difference between the target and final rotations gets smaller
        // This does assume that we're within 180 degrees, but that should be reasonable
        return (robotPose.getRotation().getRadians() - finalPose.getRotation().getRadians()) / Math.PI;
    }

    @Override
    public void execute() {
        if (foundTarget) {
            this.driveSubsystem.drive(xVelocity, yVelocity, getTargetRotationalVelocity());
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (foundTarget) {
            // Stop the robot
            this.driveSubsystem.drive(0, 0, 0);
        }
    }

    private boolean isWithinTolerance() {
        Pose2d robotPose = driveSubsystem.getRobotPose();
        /* TODO: If we are assuming that the x and y parts of the object trcking data are
         than we cant use it to see if we are done. Instead the common should take have to constructors
         One that is the same as it is now(For use in deadline race common groups) and one that takes in
         a boolean supplier that when it is true ends the commond
        */ 
        return (
            Math.abs(robotPose.getX() - finalPose.getX()) <= VisionConstants.kGoToObjectPositionTolerance.getX() &&
            Math.abs(robotPose.getY() - finalPose.getY()) <= VisionConstants.kGoToObjectPositionTolerance.getY()
        );
    }

    @Override
    public boolean isFinished() {
        if (foundTarget) {
            return isWithinTolerance();
        }
        return true; // If no pose was made, exit
    }

}
