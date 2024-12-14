package frc.robot.Auton.Autons;

import java.util.ArrayList;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.ChoreoAutonCommands.ChoreoTrajectoryCommandCreator;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPoseAfterTime;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreatorFromFile;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerAutonCreatorFromPoints;
import frc.robot.Commands.AutonCommands.PathplannerAutonCommands.PathPlannerFollowPath;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Commands.GameObjectTrackingCommands.PIDGoToObjectCommand;
import frc.robot.Commands.GameObjectTrackingCommands.SimpleGoToObjectCommand;
import frc.robot.Constants.AutonConstants.WPILibAutonConstants;
import frc.robot.Deprecated.WPILibTrajectoryCommandCreator;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class AutonNewWPILIbSystem{

    public static Command getAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        
        /*autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("ExampleWPILibPath", autonPointManager.kExampleAutonPointArray, 10,
        (new double[] {3.0, 0.0, 0.0}),
        (new double[] {3.0, 0.0, 0.0}),
        (new double[] {1.0, 0.0, 0.0}),
        4.3,
        2.3,
        9.1,
        3.5,
        new Pose2d(20, 20, new Rotation2d(.2)),
        driveSubsystem));*/
        
            
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("ExampleWPILibPath2",
        autonPointManager.kExampleAutonPointArray2, 5,
        driveSubsystem));
        
        //autonCommands.add(new PIDGoToPose(autonPointManager.kExampleAutonPointArray2[1], driveSubsystem));
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
  