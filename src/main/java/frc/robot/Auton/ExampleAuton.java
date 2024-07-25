package frc.robot.Auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.CommandUtils.SequentialGroupCommand;
import frc.robot.utils.GeneralUtils.AutonUtils.GenerateAuto;
import frc.robot.utils.GeneralUtils.AutonUtils.AutonPointUtils.AutonPoint;

public class ExampleAuton extends Command{

    public static Command getExampleAuton(AutonPointManager autonPointManager, DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        driveSubsystem.resetRobotPose(autonPointManager.kExampleStartPoint);
        ArrayList<Command> autonCommands = new ArrayList<>();
        autonCommands.add(new PIDGoToPose(new AutonPoint(3, 3, 60), driveSubsystem));

        SequentialGroupCommand auton = GenerateAuto.generateAuto(autonCommands);
        return auton;
    }
}
