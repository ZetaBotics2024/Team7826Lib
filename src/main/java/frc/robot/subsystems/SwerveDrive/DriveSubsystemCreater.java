package frc.robot.subsystems.SwerveDrive;

import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIOPigeon2;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSparkMax;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOTalonFX;

public class DriveSubsystemCreater {
    private DriveSubsystem driveSubsystem;

     public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystem;
    }

    public void createSparkMaxSwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSparkMax(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public void createTalonFXSwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOTalonFX(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOPigeon2());         
    }

    public void createSimSwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kFrontRightModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackLeftModuleName),
            new SwerveModuleIOSim(SwerveDriveConstants.kBackRightModuleName),
            new GyroIOSim());         
    }

    public void createReplaySwerve() {
        this.driveSubsystem = new DriveSubsystem(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new GyroIO() {});         
    }
}
