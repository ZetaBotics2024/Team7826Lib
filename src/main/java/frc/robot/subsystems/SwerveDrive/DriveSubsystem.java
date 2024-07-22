package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIOInputsAutoLogged;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModule;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;

public class DriveSubsystem extends SubsystemBase{

    private final SwerveModule frontLeftSwerveModule;
    private final SwerveModule frontRightSwerveModule;
    private final SwerveModule backLeftSwerveModule;
    private final SwerveModule backRightSwerveModule;
    
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged(); 

    private SwerveModuleState[] swerveModuleStates;

    private ChassisSpeeds desiredChassisSpeeds;

    private SwerveDrivePoseEstimator poseEstimator;

    public DriveSubsystem(SwerveModuleIO frontLeftSwerveModuleIO, SwerveModuleIO frontRightSwerveModuleIO,
        SwerveModuleIO backLeftSwerveModuleIO, SwerveModuleIO backRightSwerveModuleIO, GyroIO gyroIO) {
        
        this.frontLeftSwerveModule = new SwerveModule(frontLeftSwerveModuleIO, SwerveDriveConstants.kFrontLeftModuleName);
        this.frontRightSwerveModule = new SwerveModule(frontRightSwerveModuleIO, SwerveDriveConstants.kFrontRightModuleName);
        this.backLeftSwerveModule = new SwerveModule(backLeftSwerveModuleIO, SwerveDriveConstants.kBackLeftModuleName);
        this.backRightSwerveModule = new SwerveModule(backRightSwerveModuleIO, SwerveDriveConstants.kBackRightModuleName);
        
        this.gyroIO = gyroIO;

        this.poseEstimator = new SwerveDrivePoseEstimator(SwerveDriveConstants.kDriveKinematics, this.gyroInputs.yawAngle, getModulePositions(), new Pose2d());
    }

    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity) {
        this.desiredChassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity, this.gyroInputs.yawAngle);   
    }
  
    public void stop() {
        this.drive(0, 0, 0);
    }

    /**
     * Sets the swerves to 45 degrees to lock the robot in place.
     */
    public void lockSwerves(){
        SwerveModuleState frontLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        SwerveModuleState frontRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState rearLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState rearRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));

        this.frontLeftSwerveModule.setDesiredModuleState(frontLeftLockupState);
        this.frontRightSwerveModule.setDesiredModuleState(frontRightLockupState);
        this.backLeftSwerveModule.setDesiredModuleState(rearLeftLockupState);
        this.backRightSwerveModule.setDesiredModuleState(rearRightLockupState);
    }

    @Override
    public void periodic() {

        this.swerveModuleStates = getModuleStates();

        if (desiredChassisSpeeds != null) {  
            SwerveModuleState[] desiredStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);   
            // If we're not trying to move, we lock the angles of the wheels
            if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
                && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
                SwerveModuleState[] currentStates = swerveModuleStates;
                for(int i = 0; i < currentStates.length; i++) {
                    desiredStates[i].angle = currentStates[i].angle;
                }
            }
            // Positive angles should be counter clockwise.
            setModuleStates(desiredStates);
            Logger.recordOutput("SwerveDrive/ModuleStates/DesiredModuleStates", desiredStates);
        }
        
        logSwerveDrive();

        // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
        desiredChassisSpeeds = null;


        this.poseEstimator.update(this.gyroInputs.yawAngle, getModulePositions());
        Logger.recordOutput("Odometry/RobotPose", this.poseEstimator.getEstimatedPosition());
    }

    private void logSwerveDrive() {
        ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();

        this.frontLeftSwerveModule.periodic();
        this.frontRightSwerveModule.periodic();
        this.backLeftSwerveModule.periodic();
        this.backRightSwerveModule.periodic();
        this.gyroIO.updateInputs(gyroInputs, currentChassisSpeeds.omegaRadiansPerSecond);
        Logger.processInputs("SwerveDrive/Gyro", gyroInputs);

        Logger.recordOutput("SwerveDrive/ChassisSpeeds/CurrentXVelocityMPS", currentChassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/CurrentYVelocityMPS", currentChassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/CurrentRotationVelocityRadiansPerSecond", currentChassisSpeeds.omegaRadiansPerSecond);

        if(desiredChassisSpeeds != null) {
            Logger.recordOutput("SwerveDrive/DesiredChassisSpeeds/DesiredXVelocityMPS", this.desiredChassisSpeeds.vxMetersPerSecond);
            Logger.recordOutput("SwerveDrive/DesiredChassisSpeeds/DesiredYVelocityMPS", this.desiredChassisSpeeds.vyMetersPerSecond);
            Logger.recordOutput("SwerveDrive/DesiredChassisSpeeds/DesiredRotationVelocityRadiansPerSecond", this.desiredChassisSpeeds.omegaRadiansPerSecond);
        }
    
        Logger.recordOutput("SwerveDrive/ModuleStates/ActualModuleStates", getModuleStates());
    }

    /**
     * Gets the position of all four swerve modules
     * @return SwerveModulePosition[4]: Returns an array of the module position of the modules
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = {
            this.frontLeftSwerveModule.getModulePosition(),
            this.frontRightSwerveModule.getModulePosition(),
            this.backLeftSwerveModule.getModulePosition(),
            this.backRightSwerveModule.getModulePosition()
        };
        return positions;
    }

    /**
     * Gets the position of all four swerve modules
     * @return SwerveModulePosition[4]: Returns an array of the module position of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
            this.frontLeftSwerveModule.getModuleState(),
            this.frontRightSwerveModule.getModuleState(),
            this.backLeftSwerveModule.getModuleState(),
            this.backRightSwerveModule.getModuleState()
        };
        return states;
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
        this.frontLeftSwerveModule.setDesiredModuleState(desiredStates[0]);
        this.frontRightSwerveModule.setDesiredModuleState(desiredStates[1]);
        this.backLeftSwerveModule.setDesiredModuleState(desiredStates[2]);
        this.backRightSwerveModule.setDesiredModuleState(desiredStates[3]); 
    }

    private ChassisSpeeds getChassisSpeeds() {
        return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(this.swerveModuleStates);
    }

}
