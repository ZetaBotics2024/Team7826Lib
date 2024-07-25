// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ControlConstants;
import frc.robot.commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.commands.SwerveDriveCommands.LockSwerves;
import frc.robot.commands.SwerveDriveCommands.TestDrive;
import frc.robot.commands.SwerveDriveCommands.TestDrive2;
import frc.robot.subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.JoystickUtils.ControllerInterface;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually bPe handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
    private RobotCreater robotCreater;

    // Controller Decloration and Instantiation
    private ControllerInterface driverController = new ControllerInterface(ControlConstants.kDriverControllerPort);
    private ControllerInterface buttonBoard = new ControllerInterface(ControlConstants.kButtonBoardPort);
    private ControllerInterface buttonBoardAlt = new ControllerInterface(ControlConstants.kButtonBoardAltPort);
    private LoggedDashboardChooser<Command> loggablLoggedDashboardChooser = new LoggedDashboardChooser<>("AutonTestLog");
    private SendableChooser<Command> autonChooser = new SendableChooser<>();
    // Declare all Subsystems and Command Factories
    private DriveSubsystem driveSubsystem;
    private DriveCommandFactory driveCommandFactory;

    // Decloration of Commands
    FieldOrientedDriveCommand fieldOrientedDriveCommand;

    /** 
     * Initalized all Subsystem and Commands 
     */
    public RobotContainer() {
        // Creates all subsystems. Must be first call. 
        this.robotCreater = new RobotCreater();

        this.driveSubsystem = this.robotCreater.getDriveSubsystem();
        this.driveCommandFactory = new DriveCommandFactory(this.driveSubsystem, this.driverController);   
        this.fieldOrientedDriveCommand = this.driveCommandFactory.createFieldOrientedDriveCommand();
        this.driveSubsystem.setDefaultCommand(this.fieldOrientedDriveCommand);

        SmartDashboard.putData(this.autonChooser);
        configureBindings();
    }

    /**
     * Used to configure button binding
     */
    private void configureBindings() {

    }

    /**
     * Used to select our send our selected autonomus command to the Robot.java file.
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return this.loggablLoggedDashboardChooser.get();
    }
}
