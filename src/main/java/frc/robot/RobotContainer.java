// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControlConstants;
import frc.robot.utils.GeneralUtils.SubsystemContainer;
import frc.robot.utils.GeneralUtils.AutonUtils.AutonPoint;
import frc.robot.utils.GeneralUtils.AutonUtils.FudgeFactor;
import frc.robot.utils.JoystickUtils.ControllerInterface;
import frc.robot.utils.SwerveDriveUtils.SwervedriveSetupUtils;

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

    // SubsystemContainer
    private SubsystemContainer subsystemContainer;

    // Decloration of Commands

    /** 
     * Initalized all Subsystem and Commands 
     */
    public RobotContainer() {
        this.subsystemContainer = new SubsystemContainer();
        
        SwervedriveSetupUtils.createFieldOrientedDriveCommand(
            this.subsystemContainer.getDriveSubsystem(), this.driverController);
        
        AutonPoint testPoint = new AutonPoint(3, 3, 60, new FudgeFactor(0, 0, 0, 0, 0, 0));
        Logger.recordOutput("TestAutonPointBlue", testPoint.getAutonPoint(false));
        Logger.recordOutput("TestAutonPointRed", testPoint.getAutonPoint(true));
        
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
        return null;
    }
}
