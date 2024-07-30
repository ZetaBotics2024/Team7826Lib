package frc.robot.subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.utils.OdometryUtils.FudgedPoint;

public class OdometryUpdaterThread extends Thread{
    private DriveSubsystem driveSubsystem;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private PhotonPoseEstimator[] photonPoseEstimators;

    public OdometryUpdaterThread(DriveSubsystem driveSubsystem, PhotonPoseEstimator... photonPoseEstimators) {
        this.driveSubsystem = driveSubsystem;
        this.photonPoseEstimators = photonPoseEstimators;
        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics,
        this.driveSubsystem.getGyroAngleRotation2d(), this.driveSubsystem.getModulePositions(), new Pose2d(),
        VisionConstants.kSwerveDrivePoseEstimateTrust, VisionConstants.kVisionPoseEstimateTrust);
        Logger.recordOutput("Vision/Updates/VisionUpdatedSuccess", true);
    }

    @Override
    public void run() {
        while(true) {
            this.swerveDrivePoseEstimator.update(this.driveSubsystem.getGyroAngleRotation2d(),
                this.driveSubsystem.getModulePositions());
            if(VisionConstants.updateVision) {
                try {
                    for(PhotonPoseEstimator photonPoseEstimator : this.photonPoseEstimators) {
                        photonPoseEstimator.update().ifPresent(visionReading -> {
                            if(visionReading != null) {
                                Pose2d estimatedPosition = visionReading.estimatedPose.toPose2d();
                                Pose2d estimatedPositionWithGyroAngle = new Pose2d(estimatedPosition.getTranslation(),
                                    this.driveSubsystem.getGyroAngleRotation2d());
                                FudgedPoint fudgedEstimatedPosition = new FudgedPoint(estimatedPositionWithGyroAngle, VisionConstants.kFudgeFactor);
                                updatedPoseEstimationWithVisionData(fudgedEstimatedPosition, visionReading.timestampSeconds);
                            }
                        });
                    }   
                } catch (Exception e) {
                    Logger.recordOutput("Vision/Updates/VisionUpdatedSuccess", false);
                }
            }
        }
    }

    public Pose2d getRobotPose() {
        return this.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setRobotPose(AutonPoint newRobotPose) {
        setRobotPose(newRobotPose.getAutonPoint());
    }

    public void setRobotPose(Pose2d newRobotPose) {
        this.swerveDrivePoseEstimator.resetPosition(this.driveSubsystem.getGyroAngleRotation2d(),
        this.driveSubsystem.getModulePositions(), newRobotPose);
    }

    public void resetRobotPose() {
        this.swerveDrivePoseEstimator.resetPosition(this.driveSubsystem.getGyroAngleRotation2d(),
        this.driveSubsystem.getModulePositions(), new Pose2d());
    }

    private void updatedPoseEstimationWithVisionData(FudgedPoint estimatedVisionPose, double timestamp) {
        this.swerveDrivePoseEstimator.addVisionMeasurement(estimatedVisionPose.getFudgedPoint(), timestamp);
    }

    public void setAlliance(Alliance currentAlliance) {
        AprilTagFieldLayout fieldTags = photonPoseEstimators[0].getFieldTags();
    
        boolean allianceChanged = false;
        switch (currentAlliance) {
            case Blue:
                fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                allianceChanged = (VisionConstants.originPosition == OriginPosition.kRedAllianceWallRightSide);
                VisionConstants.originPosition = OriginPosition.kBlueAllianceWallRightSide;
                RobotModeConstants.isBlueAlliance = true;
                break;
            case Red:
                fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                allianceChanged = (VisionConstants.originPosition == OriginPosition.kBlueAllianceWallRightSide);
                VisionConstants.originPosition = OriginPosition.kRedAllianceWallRightSide;
                RobotModeConstants.isBlueAlliance = false;
                break;
            default:
            // No valid alliance data. Nothing we can do about it
        }
        if (allianceChanged) {
            SmartDashboard.putBoolean("set alliance", true);

          // The alliance changed, which changes the coordinate system.
          // Since a tag may have been seen and the tags are all relative to the
          // coordinate system, the estimated pose
          // needs to be transformed to the new coordinate system.
          Pose2d newPose = flipAlliance(getRobotPose());
          setRobotPose(newPose);
        }
    }
            
    private Pose2d flipAlliance(Pose2d poseToMirror) {
        Pose2d mirroredPose2d = new Pose2d(poseToMirror.getX(),
            (FieldConstants.kFieldWidthMeters - poseToMirror.getY()),
            Rotation2d.fromDegrees(poseToMirror.getRotation().getDegrees() * -1));
        return mirroredPose2d;
      }
    
}
