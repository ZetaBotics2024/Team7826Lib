package frc.robot.Commands.AutonCommands.ObjectCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Constants.AutonConstants.PIDPositioningAutonConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class SimpleGoToObjectCommand extends Command {

    private DriveSubsystem driveSubsystem;

    private ProfiledPIDController rotationPIDController;

    private NetworkTablesTunablePIDConstants rotationPIDTuner;

    private double xVelocity;
    private double yVelocity;
    private double[] targetPoseAndHeading;
    private boolean foundTarget = false;

    private Pose2d finalPose;

    private Supplier<Boolean> isFinishedSupplier;

    private Command finishedCommand = null;
    private double startTime;

    public SimpleGoToObjectCommand(DriveSubsystem driveSubsystem, Supplier<Boolean> isFinishedSupplier) {
        this.driveSubsystem = driveSubsystem;

        this.isFinishedSupplier = isFinishedSupplier;

        configurePIDs();
        configurePIDTuners();
        addRequirements(this.driveSubsystem);
    }

    public SimpleGoToObjectCommand(DriveSubsystem driveSubsystem, Command finishedCommand, double timeout) {
        this(driveSubsystem, null);

        this.finishedCommand = finishedCommand;

        this.startTime = Timer.getFPGATimestamp();
        
        this.isFinishedSupplier = new Supplier<Boolean>() {
            public Boolean get() {
                return finishedCommand.isFinished() || startTime + timeout > Timer.getFPGATimestamp();
            }
        };

        
    }

    public void configurePIDs() {
        this.rotationPIDController = new ProfiledPIDController(
            PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant,
            PIDPositioningAutonConstants.kTranslationPIDControllerConstraints); //TODO: Make other constants?

        this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

        this.rotationPIDController.setTolerance(PIDPositioningAutonConstants.kRotationToleranceRadians);
    }

    @Override
    public void initialize() {
        if (finishedCommand != null) {
            finishedCommand.schedule();
        }
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
            finalPose = new Pose2d(new Translation2d(this.targetPoseAndHeading[2], this.targetPoseAndHeading[3]), Rotation2d.fromRadians(this.targetPoseAndHeading[1]));
            if (finalPose.getX() > finalPose.getY()) { // If X > Y
                xVelocity = 1;
                yVelocity = finalPose.getY() / finalPose.getX(); // Y = Y/X
            }
            else { // If Y >= X
                xVelocity = finalPose.getX() / finalPose.getY(); //X = X/Y
                yVelocity = 1;
            }
        }

        
        this.rotationPIDController.reset(this.driveSubsystem.getRobotPose().getRotation().getDegrees());
        
    }

    public void configurePIDTuners() {
        

        this.rotationPIDTuner = new NetworkTablesTunablePIDConstants(
            "PIDGoToPose/Rotation",
            PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant,
            0);
    }

    /**
     * WARNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the PIDs bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {

        double[] currentRotationPIDValues = this.rotationPIDTuner.getUpdatedPIDConstants();
        if(this.rotationPIDTuner.hasAnyPIDValueChanged()) {
            this.rotationPIDController = new ProfiledPIDController(
                currentRotationPIDValues[0],
                currentRotationPIDValues[1],
                currentRotationPIDValues[2],
                PIDPositioningAutonConstants.kRotationPIDControllerConstraints);
            this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
            this.rotationPIDController.setTolerance(PIDPositioningAutonConstants.kRotationToleranceRadians);
        }
    }

    public double getTargetRotationalVelocity() {
        Pose2d robotPose = driveSubsystem.getRobotPose();
        // Slow down as the difference between the target and final rotations gets smaller
        // This does assume that we're within 180 degrees, but that should be reasonable
        return (robotPose.getRotation().getRadians() - finalPose.getRotation().getRadians()) / Math.PI;
    }

    @Override
    public void execute() {

        double rotationVelocity = this.rotationPIDController.calculate(this.driveSubsystem.getRobotPose().getRotation().getRadians(),
            finalPose.getRotation().getRadians());
        if (foundTarget) {
            this.driveSubsystem.drive(xVelocity, yVelocity, rotationVelocity);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (foundTarget) {
            // Stop the robot
            this.driveSubsystem.drive(0, 0, 0);
        }
    }

    /*private boolean isWithinTolerance() {
        Pose2d robotPose = driveSubsystem.getRobotPose();
        return (
            Math.abs(robotPose.getX() - finalPose.getX()) <= VisionConstants.kGoToObjectPositionTolerance.getX() &&
            Math.abs(robotPose.getY() - finalPose.getY()) <= VisionConstants.kGoToObjectPositionTolerance.getY()
        );
    }*/

    @Override
    public boolean isFinished() {
        return isFinishedSupplier.get();
    }

}
