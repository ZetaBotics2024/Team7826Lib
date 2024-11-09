package frc.robot.Commands.AutonCommands.ObjectCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.AutonCommands.PIDPositioningAutonCommands.PIDGoToPose;
import frc.robot.Subsystems.GameObjectTracking.GameObjectTracker;
import frc.robot.Subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.Utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class PIDGoToObjectCommand extends Command {

    private DriveSubsystem driveSubsystem;

    private PIDGoToPose goToPoseCommand;
    private double[] targetPoseAndHeading;
    private boolean foundTarget = false;

    public PIDGoToObjectCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
    }

    @Override
    public void initialize() {
        // Find target position
        this.targetPoseAndHeading = GameObjectTracker.getTargetDistanceAndHeading(); // Returns hypotenuse, heading, x distance and y distance
        // Check that the pose isn't just zeroes
        this.foundTarget = !(
            this.targetPoseAndHeading[0] == 0 &&
            this.targetPoseAndHeading[1] == 0 &&
            this.targetPoseAndHeading[2] == 0 &&
            this.targetPoseAndHeading[3] == 0 
        );
        // Create position command
        if (foundTarget) {
            this.goToPoseCommand = new PIDGoToPose(
                new AutonPoint(new Pose2d(
                    new Translation2d(
                        this.targetPoseAndHeading[2],
                        this.targetPoseAndHeading[3]
                    ),
                    Rotation2d.fromRadians(this.targetPoseAndHeading[1])
                ), 
                new FudgeFactor(
                    0, 
                    0, 
                    0)
                ), // Perhaps fudge factor needs to be tuned 
                this.driveSubsystem);
                this.goToPoseCommand.initialize();
        }
        
    }

    @Override
    public void execute() {
        if (foundTarget) {
            this.goToPoseCommand.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (foundTarget) {
            this.goToPoseCommand.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        if (foundTarget) {
            return this.goToPoseCommand.isFinished();
        }
        return true; // If no pose was made, exit
    }

}
