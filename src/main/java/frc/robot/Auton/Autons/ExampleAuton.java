package frc.robot.Auton.Autons;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.commands.AutonCommands.ChoreoAutonCommands.ChoreoTrajectoryCommandCreater;
import frc.robot.commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPoseAfterTime;
import frc.robot.commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreaterFromFile;
import frc.robot.commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreaterFromPoints;
import frc.robot.commands.AutonCommands.PathplannerAutonCommands.PathPlannerFollowPath;
import frc.robot.commands.AutonCommands.WPILibTrajectoryCommands.WPILibTrajectoryCommandCreater;
import frc.robot.subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.GenerateAuto;
import frc.robot.utils.CommandUtils.SequentialGroupCommand;

public class ExampleAuton{

    public static Command getExampleAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        autonCommands.add(ChoreoTrajectoryCommandCreater.createChoreoTrajectoryCommand("ExampleAutonChoreo", driveSubsystem));
        autonCommands.add(PathPlannerAutonCreaterFromPoints.createAutonCommand(autonPointManager.kExampleAutonPointArray, 10, driveSubsystem));
        autonCommands.add(new PIDGoToPose(autonPointManager.kExampleAutonPoint, driveSubsystem));
        autonCommands.add(PathPlannerAutonCreaterFromFile.createAutonCommand(autonPointManager.kExampleAutonName,
            autonPointManager.kExampleAutonEndPoint, 10, driveSubsystem));
        autonCommands.add(new WPILibTrajectoryCommandCreater(autonPointManager.kExampleAutonPointArray, driveSubsystem));

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
  