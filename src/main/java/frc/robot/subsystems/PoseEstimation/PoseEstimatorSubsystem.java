package frc.robot.Subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Subsystems.PoseEstimation.OdometryIOInputsAutoLogged;

public class PoseEstimatorSubsystem extends SubsystemBase{
    private OdometryIO odometryIO;
    private OdometryIOInputsAutoLogged odometryInputs = new OdometryIOInputsAutoLogged();

    private PhotonCamera exampleCamera = new PhotonCamera("ExampleCamera");
    private PhotonPoseEstimator exampleEstimator;

    public PoseEstimatorSubsystem(DriveSubsystem driveSubsystem) { 
        configPoseEstimators();
        VisionConstants.kAprilTagLayout.setOrigin(VisionConstants.originPosition);
        this.odometryIO = new OdometryIOOdometryUpdaterThread(
            new OdometryUpdaterThread(driveSubsystem,
            this.exampleEstimator));
    }

    public Pose2d getRobotPose() {
        return this.odometryInputs.robotPosition;
    }

    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }

    public void setRobotPose(Pose2d newRobotPose) {
        this.odometryIO.setRobotPose(newRobotPose);
    }

    public void resetRobotPose() {
        this.odometryIO.resetRobotPose();
    }

    public void periodic() {
        this.odometryIO.updateInputs(this.odometryInputs);
        Logger.processInputs("Odometry/", odometryInputs);
    }

    private void configPoseEstimators() {
        this.exampleEstimator = new PhotonPoseEstimator(VisionConstants.kAprilTagLayout,
            PoseStrategy.LOWEST_AMBIGUITY, exampleCamera,
            VisionConstants.kExampleCameraToRobotCenter);
    }

    public void setAlliance(Alliance alliance) {
        this.odometryIO.setAlliance(alliance);    
    }
    
}
