package frc.robot.utils.GeneralUtils;

import frc.robot.subsystems.SwerveDrive.DriveSubsystem;

public class SubsystemContainer {
    private DriveSubsystem driveSubsystem;

    /**
     * Creates SubsystemContainer object. Configures all subsystems bassed on the robots mode. 
     */
    public SubsystemContainer() {
        RobotUtils.configureRobotBasedOnMode(this);
    }

    /**
     * Gets the only instants of the Drivesubsystem
     * @return
     */
    public DriveSubsystem getDriveSubsystem() {
        return this.driveSubsystem;
    }

    /** Sets the drive subsystems intence. Should only be called in the configure robot based on mode. DO NOT CALL!! 
     * @param driveSubsystem: The value to set the drive subsystem to.
     */
    public void setDriveSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }
}