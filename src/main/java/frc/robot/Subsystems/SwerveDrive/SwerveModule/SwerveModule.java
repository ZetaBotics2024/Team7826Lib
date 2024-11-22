package frc.robot.Subsystems.SwerveDrive.SwerveModule;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.Utils.CommandUtils.Wait;
import frc.robot.Utils.SwerveDriveUtils.DesiredMetersPerSecondToVoltage;
import frc.robot.Utils.SwerveDriveUtils.SwerveDriveVoltageVSMetersPerSecondTableCreater;
import frc.robot.Utils.SwerveDriveUtils.SwerveModuleAngleOptimizer;
import frc.robot.Subsystems.SwerveDrive.SwerveModule.SwerveModuleIOInputsAutoLogged;

public class SwerveModule {
    
    private final SwerveModuleIO swerveModuleIO;
    private final SwerveModuleIOInputsAutoLogged swerveModuleInputs = new SwerveModuleIOInputsAutoLogged();
    private String swerveModuleName = "NoModuleNameSet";
    private SwerveDriveVoltageVSMetersPerSecondTableCreater voltageTableCreator;

    /**
     * Creates a Swerve Module Object and compleates all initialization
     * @param swerveModuleIO SwerveModuleIO: The IO object for the Swerve Module. It can be either SwerveModule
     * @param swerveModuleName
     */
    public SwerveModule(SwerveModuleIO swerveModuleIO, String swerveModuleName) {
        this.swerveModuleIO = swerveModuleIO;
        this.swerveModuleName = swerveModuleName;
        this.voltageTableCreator = new SwerveDriveVoltageVSMetersPerSecondTableCreater();
    }

    /**
    * Update inputs without running the rest of the periodic logic. This is useful since these
    * updates need to be properly thread-locked.
    */
    public void updateInputs() {
        swerveModuleIO.updateInputs(swerveModuleInputs);
    }

    /**
     * Must be called in DriveSubsystem Periodic. Updates all logged inputes
     */
    public void periodic() {
        updateInputs();
        //if(DriverStation.isEnabled()) {
        //    voltageTableCreator.createVoltageVSMetersPerSecondTable(swerveModuleName,
        //    13, .1, 1, getDriveMotorSpeedInMetersPerSecond(),
        //    this.swerveModuleIO::setDesiredModuleDriveVoltage);
        //}
        
        Logger.processInputs(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName, swerveModuleInputs);
    }

    /**
     * Sets the desired state of the Swerve Module
     * @param desiredState SwerveModuleState: The desired state for the module
     */
    public void setDesiredModuleState(SwerveModuleState desiredState) {
        SwerveModuleState optimizedState = SwerveModuleAngleOptimizer.optimize(desiredState, getModuleState().angle); 
        Logger.recordOutput(SwerveModuleConstants.kSwerveModuleOutputLoggerBase + swerveModuleName + "DesiredMetersPerSecond", optimizedState.speedMetersPerSecond);
        
        /* Better system we made at the fair does not work well with sim. Need to try with actual swerve mods latter
         *  double driveVoltage = optimizedState.speedMetersPerSecond == 0 ? 0 : 
            MathUtil.clamp((((optimizedState.speedMetersPerSecond/SwerveDriveConstants.kMaxSpeedMetersPerSecond) * 13)
            + (SwerveDriveConstants.kDriveFeedForward * Math.signum(optimizedState.speedMetersPerSecond))), -13, 13);
         */
         
        //double desiredRPM = optimizedState.speedMetersPerSecond / SwerveModuleConstants.kDriveConversionVelocityFactor;
        //this.swerveModuleIO.setDesiredModuleVelocityRPM(desiredRPM);
        this.swerveModuleIO.setDesiredModuleDriveVoltage(metersPerSecondToVoltage(optimizedState.speedMetersPerSecond));
        //this.swerveModuleIO.setDesiredModuleDriveVoltage(DesiredMetersPerSecondToVoltage.metersPerSecondToVoltage(optimizedState.speedMetersPerSecond));//metersPerSecondToVoltage(optimizedState.speedMetersPerSecond));
        this.swerveModuleIO.setDesiredModuleAngle(optimizedState.angle);
    }

    public double metersPerSecondToVoltage(double desiredMetersPerSecond) {
        double percentOfMaxSpeed = desiredMetersPerSecond / SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        double appliedFeedforwardVoltage = (desiredMetersPerSecond != 0 ? SwerveDriveConstants.kVoltageFeedforward : 0) *
            Math.signum(desiredMetersPerSecond);
        double unclampedVoltage = percentOfMaxSpeed *
            SwerveDriveConstants.kVoltageForMaxSpeed +
            appliedFeedforwardVoltage;
        double clampedVoltage = MathUtil.clamp(unclampedVoltage, -SwerveDriveConstants.kVoltageForMaxSpeed, SwerveDriveConstants.kVoltageForMaxSpeed);
        return clampedVoltage;
    }

    public void stopModuleDrive() {
        this.swerveModuleIO.setDesiredModuleDriveVoltage(0);
    }
    
    /**
     * Gets the position of the module
     * @return SwerveModulePosition: The Swerve Module Position, usefull for odometry
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            swerveModuleInputs.driveMotorDistanceMeters, Rotation2d.fromRotations(swerveModuleInputs.wheelAngleRelitivePositionRotations));
    }

    /**
     * The current state of the module
     * @return SwerveModuleState: The current state of the module
     */
    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveMotorSpeedInMetersPerSecond(), getWheelRotationAsRotation2d());
    }

    /**
     * The Speed of the drive motor in meters per second
     * @return Double: The current speed of the drive motor in MPS(Meters per second)
     */
    private double getDriveMotorSpeedInMetersPerSecond() {
        return this.swerveModuleInputs.driveMotorRPM * SwerveModuleConstants.kDriveConversionVelocityFactor;
    }

    /**
     * The current angle of the module's wheel
     * @return Rotation2d: The module's wheel angle
     */
    private Rotation2d getWheelRotationAsRotation2d() {
        return Rotation2d.fromRotations(swerveModuleInputs.wheelAngleRelitivePositionRotations);
    }
    
}