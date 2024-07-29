package frc.robot.commands.AutonCommands.PathplannerAutonCommands;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;

public class PathPlannerAutonCreaterFromFile {

    public static Command createAutonCommand(String autonName, AutonPoint endPoint, double maxTime, DriveSubsystem driveSubsystem) {
        return new PathPlannerFollowPath(AutoBuilder.buildAuto(autonName), endPoint.getAutonPoint(), maxTime, driveSubsystem);
    }
}
