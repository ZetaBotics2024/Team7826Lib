package frc.robot.Auton.Autons;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.ChoreoAutonCommands.ChoreoTrajectoryCommandCreater;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPoseAfterTime;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreaterFromFile;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreaterFromPoints;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerFollowPath;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibTrajectoryCommandCreater;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class ExampleAuton{

    public static Command getExampleAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        autonCommands.add(ChoreoTrajectoryCommandCreater.createChoreoTrajectoryCommand("ExampleAutonChoreo", driveSubsystem));
        //autonCommands.add(PathPlannerAutonCreaterFromPoints.createAutonCommand(autonPointManager.kExampleAutonPointArray, 10, driveSubsystem));
        //autonCommands.add(new PIDGoToPose(autonPointManager.kExampleAutonPoint, driveSubsystem));
        //autonCommands.add(PathPlannerAutonCreaterFromFile.createAutonCommand(autonPointManager.kExampleAutonName,
         //   autonPointManager.kExampleAutonEndPoint, 10, driveSubsystem));
        //autonCommands.add(new WPILibTrajectoryCommandCreater(autonPointManager.kExampleAutonPointArray, driveSubsystem));

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
  