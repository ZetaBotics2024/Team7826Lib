package frc.robot.subsystems.SwerveDrive;

import org.littletonrobotics.junction.Logger;

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

    public DriveSubsystem(SwerveModuleIO frontLeftSwerveModuleIO, SwerveModule frontRightSwerveModuleIO,
        SwerveModuleIO backLeftSwerveModulemIO, SwerveModuleIO backRightSwerveModuleIO, GyroIO gyroIO) {
        
        this.frontLeftSwerveModule = new SwerveModule(frontLeftSwerveModuleIO, SwerveDriveConstants.kFrontLeftModuleName);
        this.frontRightSwerveModule = new SwerveModule(frontLeftSwerveModuleIO, SwerveDriveConstants.kFrontLeftModuleName);
        this.backLeftSwerveModule = new SwerveModule(frontLeftSwerveModuleIO, SwerveDriveConstants.kFrontLeftModuleName);
        this.backRightSwerveModule = new SwerveModule(frontLeftSwerveModuleIO, SwerveDriveConstants.kFrontLeftModuleName);

        this.gyroIO = gyroIO;
    }

    public void drive(double desiredXVelocity, double desiredYVelocity, double desiredRotationalVelocity) {
        this.desiredChassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(desiredXVelocity, desiredYVelocity,
                desiredRotationalVelocity, this.gyroInputs.yawAngle);   
    }
  
    public void stop() {
        this.drive(0, 0, 0);
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
        }
        
        logSwerveDrive();

        // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
        desiredChassisSpeeds = null;
    }

    private void logSwerveDrive() {
        this.gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("SwerveDrive", gyroInputs);

        ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/CurrentXVelocityMPS", currentChassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/CurrentYVelocityMPS", currentChassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/CurrentRotationVelocityRadiansPerSecond", currentChassisSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("SwerveDrive/ChassisSpeeds/DesiredXVelocityMPS", this.desiredChassisSpeeds.vxMetersPerSecond);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/DesiredYVelocityMPS", this.desiredChassisSpeeds.vyMetersPerSecond);
        Logger.recordOutput("SwerveDrive/ChassisSpeeds/DesiredRotationVelocityRadiansPerSecond", this.desiredChassisSpeeds.omegaRadiansPerSecond);

        Logger.recordOutput("SwerveDrive/DesiredModuleStates", getModuleStates());
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
