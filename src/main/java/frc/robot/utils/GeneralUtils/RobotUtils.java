package frc.robot.utils.GeneralUtils;


import frc.robot.Constants.RobotModeConstants;
import frc.robot.utils.SwerveDriveUtils.SwervedriveSetupUtils;

public class RobotUtils {
    /**
     * Configures the robot based on whether or not it Real, SIM or Replay. 
     * @param driveSubsystem
     */
    public static void configureRobotBasedOnMode(SubsystemContainer subsystemContainer) {
        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                subsystemContainer.setDriveSubsystem(SwervedriveSetupUtils.createSparkMaxSwerve());
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                subsystemContainer.setDriveSubsystem(SwervedriveSetupUtils.createSimSwerve());
                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                subsystemContainer.setDriveSubsystem(SwervedriveSetupUtils.createReplaySwerve());
                break;
        default:
            throw new RuntimeException("Invalid Robot Mode. Please set teh current mode value in RobotModeConstants");
        }
    }
}
