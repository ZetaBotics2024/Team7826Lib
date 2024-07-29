package frc.robot.commands.AutonCommands.PIDPositioningAutonCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.utils.CommandUtils.CustomWaitCommand;
import frc.robot.utils.CommandUtils.SequentialGroupCommand;
import frc.robot.utils.CommandUtils.Wait;

public class PIDGoToPoseAfterTime extends Command{
   

    public static Command createPIDGoToPoseAfterTime(AutonPoint endPoint, DriveSubsystem driveSubsystem, double waitTime) {
        CustomWaitCommand waitTimer = new CustomWaitCommand(waitTime);
        PIDGoToPose goToPose = new PIDGoToPose(endPoint, driveSubsystem);
        Command[] goToPoseAfterTimeSequence = {waitTimer, goToPose};
        return new SequentialGroupCommand(goToPoseAfterTimeSequence);
    }

    
}
