package frc.robot.Commands.GameObjectTrackingCommands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.PIDPositioningAutonConstants;
import frc.robot.Constants.GameObjectTrackingConstants.RotateToFaceGameObjectConstants;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.GeneralUtils.Tolerance;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class RotateToFaceGameObject extends Command {
    private DriveSubsystem driveSubsystem;
    private NetworkTablesTunablePIDConstants rotationPIDTuner;
    private ProfiledPIDController rotationPIDController;
    private double goalRotation = 0;

    public RotateToFaceGameObject(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.rotationPIDController = new ProfiledPIDController(
            RotateToFaceGameObjectConstants.kPRotationConstant,
            RotateToFaceGameObjectConstants.kIRotationConstant,
            RotateToFaceGameObjectConstants.kDRotationConstant,
            RotateToFaceGameObjectConstants.kRotationPIDControllerConstraints
        );
        this.rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.rotationPIDController.setTolerance(RotateToFaceGameObjectConstants.kRotationTolorence);
        this.rotationPIDTuner = new NetworkTablesTunablePIDConstants("RotateToFaceGameObjectPID", PIDPositioningAutonConstants.kPRotationPIDConstant,
            PIDPositioningAutonConstants.kIRotationPIDConstant,
            PIDPositioningAutonConstants.kDRotationPIDConstant, 
            0.0);
        addRequirements(this.driveSubsystem);
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

    @Override
    public void initialize() {
        this.goalRotation = driveSubsystem.getRobotPose().getRotation().getRadians() + GameObjectTracker.getTargetDistanceAndHeading()[1];
    }

    @Override
    public void execute() {
        updatePIDValuesFromNetworkTables();
        double rotationSpeed = rotationPIDController.calculate(driveSubsystem.getRobotPose().getRotation().getRadians(), goalRotation);
        this.driveSubsystem.drive(0.0, 0.0, rotationSpeed, true);
    }

    @Override 
    public void end(boolean interrupt) {
        this.driveSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return Tolerance.inTolorance(this.driveSubsystem.getRobotPose().getRotation().getRadians(),
        goalRotation,
        RotateToFaceGameObjectConstants.kRotationTolorence);
    }

}
