package frc.robot.Auton;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPoseAfterTime;
import frc.robot.commands.AutonCommands.PathplannerAutonCommands.PathPlannerCreateAuton;
import frc.robot.commands.AutonCommands.PathplannerAutonCommands.PathPlannerCreateAutonFromPoints;
import frc.robot.commands.AutonCommands.PathplannerAutonCommands.PathPlannerFollowPath;
import frc.robot.subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.GenerateAuto;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.utils.CommandUtils.SequentialGroupCommand;

public class ExampleAuton extends Command{

    public static Command getExampleAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        //autonCommands.add(PathPlannerCreateAutonFromPoints.createAutonCommand(autonPointManager.kExampleAutonPointArray, 10, driveSubsystem));
        autonCommands.add(new PIDGoToPoseAfterTime(autonPointManager.kExampleAutonPoint, 3, driveSubsystem));
        //autonCommands.add(PathPlannerCreateAuton.createAutonCommand(autonPointManager.kExampleAutonName));
 
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
  