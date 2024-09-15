package frc.robot.Commands.GameObjectTrackingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;

public class TurnThenDriveToGameObject extends Command{
    private DriveSubsystem driveSubsystem;

    public TurnThenDriveToGameObject(DriveSubsystem driveSubsystem) {
      
        addRequirements(this.driveSubsystem);
    }
    
    @Override 
    public void initialize() {
    
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
  
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
