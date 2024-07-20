// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.jni.ControlConfigJNI;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.RobotModeConstants;
import frc.robot.commands.SwerveDriveCommands.FieldOrientedDriveCommand;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.JoystickUtils.ControllerInterface;
import frc.robot.utils.SwerveDriveUtils.DriveSubsystemCreationUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually bPe handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {  
    // Controller Decloration and Instantiation
    private ControllerInterface driverController = new ControllerInterface(ControlConstants.kDriverControllerPort);
    private ControllerInterface buttonBoard = new ControllerInterface(ControlConstants.kButtonBoardPort);
    private ControllerInterface buttonBoardAlt = new ControllerInterface(ControlConstants.kButtonBoardAltPort);

    // Decloration of Subsystems
    private DriveSubsystem driveSubsystem;

    // Decloration of Commands
    FieldOrientedDriveCommand fieldOrientedDriveCommand;

    /** 
     * Initalized all Subsystem and Commands 
     */
    public RobotContainer() {
        configureRobotBasedOnMode();
        createFieldOrientedDriveCommand();
        


        configureBindings();
    }

    public void configureRobotBasedOnMode() {
        switch (RobotModeConstants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                this.driveSubsystem = DriveSubsystemCreationUtils.createSparkMaxSwerve();
                break;
            case SIM:
                // Sim robot, instantiate physics sim IO implementation
                this.driveSubsystem = DriveSubsystemCreationUtils.createSimSwerve();
                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                this.driveSubsystem = DriveSubsystemCreationUtils.createReplaySwerve();
                break;
        default:
            throw new RuntimeException("Invalid Robot Mode. Please set teh current mode value in RobotModeConstants");
        }
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
        return null;
    }

    public void createFieldOrientedDriveCommand() {
        System.out.println(this.driverController.getLeftX());
        this.fieldOrientedDriveCommand = new FieldOrientedDriveCommand(
        this.driveSubsystem, 
        () -> -this.driverController.getLeftY(),
        () -> -this.driverController.getLeftX(),
        () -> -this.driverController.getRightX());
        
        this.driveSubsystem.setDefaultCommand(this.fieldOrientedDriveCommand);
    }
}
