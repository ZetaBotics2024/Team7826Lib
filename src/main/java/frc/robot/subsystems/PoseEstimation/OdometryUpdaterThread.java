package frc.robot.Subsystems.PoseEstimation;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.OdometryUtils.FudgedPoint;

public class OdometryUpdaterThread extends Thread{
    private DriveSubsystem driveSubsystem;
    private SwerveDrivePoseEstimator swerveDrivePoseEstimator;
    private PhotonPoseEstimator[] photonPoseEstimators;
    private boolean hasAllianceChanged = false;

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
            setAlliance();
            this.swerveDrivePoseEstimator.update(this.driveSubsystem.getGyroAngleRotation2d(),
                this.driveSubsystem.getModulePositions());
            if(VisionConstants.updateVision) {
                try {
                    for(PhotonPoseEstimator photonPoseEstimator : this.photonPoseEstimators) {
                        photonPoseEstimator.update().ifPresent(visionReading -> {
                            boolean usedExcludedTag = false;
                            for(PhotonTrackedTarget target : visionReading.targetsUsed) {
                                for(int i = 0; i < VisionConstants.kExcludedTags.length; i++) {
                                    usedExcludedTag = target.getFiducialId() == VisionConstants.kExcludedTags[i];
                                }
                                if(usedExcludedTag) {
                                        break;
                                    }
                            }
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

    public void setAlliance() {
        AprilTagFieldLayout fieldTags = photonPoseEstimators[0].getFieldTags();
        if(RobotModeConstants.hasAllianceChanged) {
            if(RobotModeConstants.isBlueAlliance) {
                fieldTags.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
                VisionConstants.originPosition = OriginPosition.kBlueAllianceWallRightSide;
            } else {
                fieldTags.setOrigin(OriginPosition.kRedAllianceWallRightSide);
                VisionConstants.originPosition = OriginPosition.kRedAllianceWallRightSide;
            }
            setRobotPose(flipAlliance(getRobotPose()));
            RobotModeConstants.hasAllianceChanged = false;
        }    
    }
            
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(new Pose2d(
            new Translation2d(FieldConstants.kFieldLengthMeters, FieldConstants.kFieldWidthMeters),
            new Rotation2d(Math.PI)));
    }
    
}
