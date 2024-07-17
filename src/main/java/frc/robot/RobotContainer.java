// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.LEDUtils.LEDManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually bPe handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Decloration of Subsystems
  
  // Decloration of Commands
  
  /** 
   * Initalized all Subsystem and Commands 
  */
  public RobotContainer() {
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
