package frc.robot;


import frc.robot.Constants.RobotModeConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.DriveSubsystemCreater;

public class RobotCreater {
    // Decloration of subsystem creatoin
    private final DriveSubsystemCreater driveSubsystemCreater;

    public RobotCreater() {
        this.driveSubsystemCreater = new DriveSubsystemCreater();

        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                this.driveSubsystemCreater.createSparkMaxSwerve();
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystemCreater.createSimSwerve();
                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystemCreater.createTalonFXSwerve();
                break;
        default:
            throw new RuntimeException("Invalid Robot Mode. Please set teh current mode value in RobotModeConstants");
        }
    }

    public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystemCreater.getDriveSubsystem();
    }
}
