package frc.robot.commands.AutonCommands.PathplannerAutonCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerCreateAuton {

    public static Command createAutonCommand(String autonName) {
        return AutoBuilder.buildAuto(autonName);
    }
}
