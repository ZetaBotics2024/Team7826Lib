package frc.robot.subsystems.SwerveDrive;

import frc.robot.commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.commands.SwerveDriveCommands.LockSwerves;
import frc.robot.utils.JoystickUtils.ControllerInterface;

public class DriveCommandFactory {
    // All Subsystems needed for every command
    DriveSubsystem driveSubsystem;

    // All other dependencies for every command that uses primarly DriveSubsystem
    ControllerInterface driverController;

    public DriveCommandFactory(DriveSubsystem driveSubsystem, ControllerInterface driverController) {
        this.driveSubsystem = driveSubsystem;
        this.driverController = driverController;
    }

    public FieldOrientedDriveCommand createFieldOrientedDriveCommand() {
        return new FieldOrientedDriveCommand(
        this.driveSubsystem, 
        () -> -this.driverController.getLeftY(),
        () -> -this.driverController.getLeftX(),
        () -> -this.driverController.getRightX());
    }

    public LockSwerves createLockSwervesCommand() {
        return new LockSwerves(this.driveSubsystem);
    }

}
