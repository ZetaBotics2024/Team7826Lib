package frc.robot.Commands.AutonCommands.ObjectCommands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
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

    private Supplier<Boolean> isFinishedSupplier;

    private Command finishedCommand = null;
    private double startTime;

    public PIDGoToObjectCommand(DriveSubsystem driveSubsystem, Supplier<Boolean> isFinishedSupplier) {
        this.driveSubsystem = driveSubsystem;

        this.isFinishedSupplier = isFinishedSupplier;

        addRequirements(this.driveSubsystem);
    }

    public PIDGoToObjectCommand(DriveSubsystem driveSubsystem, Command finishedCommand, double timeout) {
        this(driveSubsystem, null);

        this.finishedCommand = finishedCommand;

        this.startTime = Timer.getFPGATimestamp();
        
        this.isFinishedSupplier = new Supplier<Boolean>() {
            public Boolean get() {
                return finishedCommand.isFinished() || startTime + timeout > Timer.getFPGATimestamp();
            }
        };
    }

    @Override
    public void initialize() {
        if (finishedCommand != null) {
            finishedCommand.schedule();
        }
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
        return isFinishedSupplier.get();
    }

}
