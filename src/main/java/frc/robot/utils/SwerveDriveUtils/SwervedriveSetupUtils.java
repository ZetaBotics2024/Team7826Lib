package frc.robot.utils.SwerveDriveUtils;

import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIO;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIOPigeon2;
import frc.robot.subsystems.SwerveDrive.Gyro.GyroIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIO;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSim;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOSparkMax;
import frc.robot.subsystems.SwerveDrive.SwerveModule.SwerveModuleIOTalonFX;
import frc.robot.utils.JoystickUtils.ControllerInterface;

public class SwervedriveSetupUtils {
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
            new GyroIOSim());         
    }

    public static DriveSubsystem createReplaySwerve() {
        return new DriveSubsystem(
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new SwerveModuleIO() {},
            new GyroIO() {});         
    }

    public static void createFieldOrientedDriveCommand(DriveSubsystem driveSubsystem,
        ControllerInterface driverController) {
        FieldOrientedDriveCommand fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        driveSubsystem, 
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX());
        
        driveSubsystem.setDefaultCommand(fieldOrientedDriveCommand);
    }
}
