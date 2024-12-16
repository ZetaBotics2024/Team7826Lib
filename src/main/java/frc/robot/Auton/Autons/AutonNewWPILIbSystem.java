package frc.robot.Auton.Autons;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.AutonPointManager;
import frc.robot.Commands.AutonCommands.WPILibTrajectoryCommands.WPILibFollowTrajectoryFromPointsCommand;
import frc.robot.Commands.GameObjectTrackingCommands.DriveForward;
import frc.robot.Commands.GameObjectTrackingCommands.RotateToFaceGameObject;
import frc.robot.Commands.GameObjectTrackingCommands.RotateToFaceGameObjectWhileDriving;
import frc.robot.Subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.GenerateAuto;
import frc.robot.Utils.CommandUtils.ParallelGroupCommand;
import frc.robot.Utils.CommandUtils.SequentialGroupCommand;

public class AutonNewWPILIbSystem{

    public static Command getAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.setRobotPose(autonPointManager.kExampleStartPoint);
        
        ArrayList<Command> autonCommands = new ArrayList<>();
        autonCommands.add(new RotateToFaceGameObjectWhileDriving(3, 2, 12, driveSubsystem));
        /* 
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("ExampleWPILibPath", autonPointManager.kExampleAutonPointArray2, 10,
        (new double[] {3.0, 0.0, 0.0}),
        (new double[] {3.0, 0.0, 0.0}),
        (new double[] {1.0, 0.0, 0.0}),
        4.3,
        2.3,
        9.1,
        3.5,
        new Pose2d(.02, .02, new Rotation2d(.05)),
        driveSubsystem));
        
            
        autonCommands.add(new WPILibFollowTrajectoryFromPointsCommand("ExampleWPILibPath2", autonPointManager.kExampleAutonPointArray2, 2,
        (new double[] {1.0, 0.0, 0.0}),
        (new double[] {1.0, 0.0, 0.0}),
        (new double[] {1.0, 0.0, 0.0}),
        1,
        2.5,
        2,
        3.7,
        new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(1)),
        driveSubsystem));
        */
        //autonCommands.add(new PIDGoToPose(autonPointManager.kExampleAutonPointArray2[1], driveSubsystem));
        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    } 
}
  