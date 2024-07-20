package frc.robot.utils.SwerveDriveUtils;

import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIOPigeon2;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSparkMax;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOTalonFX;

public class DriveSubsystemCreationUtils {
    public static DriveSubsystem createSparkMaxSwerve() {
        return new DriveSubsystem(
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public static DriveSubsystem createTalonFXSwerve() {
        return new DriveSubsystem(
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public static DriveSubsystem createSimSwerve() {
        return new DriveSubsystem(
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackRightModuleName),
            new GyroIO() {});         
    }

    public static DriveSubsystem createReplaySwerve() {
        return new DriveSubsystem(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new GyroIO() {});         
    }
}
