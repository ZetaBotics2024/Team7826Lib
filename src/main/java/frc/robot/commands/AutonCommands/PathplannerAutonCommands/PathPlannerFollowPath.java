package frc.robot.commands.AutonCommands.PathplannerAutonCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.PathPlannerAutonConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.utils.CommandUtils.Wait;

public class PathPlannerFollowPath extends Command{
    private Pose2d endPoint;
    private double maxTime;
    private PathPlannerFollowPath path;
    private Command followPathCommand;
    private Wait hardCutOffTimer;
    private DriveSubsystem driveSubsystem;

    public PathPlannerFollowPath(PathPlannerPath path, Pose2d endPoint, double maxTime, DriveSubsystem driveSubsystem) {
        this.endPoint = endPoint;
        this.maxTime = maxTime;
        this.followPathCommand = AutoBuilder.followPath(path);
        this.hardCutOffTimer = new Wait(maxTime);
        this.driveSubsystem = driveSubsystem;
    }
    
    @Override
    public void initialize() {
        this.followPathCommand.schedule();
        this.hardCutOffTimer.startTimer();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("PATH OVER");
        this.driveSubsystem.stop();
        this.followPathCommand.end(true);
    }

    @Override
    public boolean isFinished() {
        if(this.hardCutOffTimer.hasTimePassed()) {
            return true;
        }

        Pose2d robotPose = this.driveSubsystem.getRobotPose();
        boolean hasReachedXTolorence = Math.abs(this.endPoint.getTranslation().getX() - robotPose.getX()) <= 
            PathPlannerAutonConstants.kTranslationToleranceMeters;
        boolean hasReachedYTolorence = Math.abs(this.endPoint.getTranslation().getY() - robotPose.getY()) <= 
            PathPlannerAutonConstants.kTranslationToleranceMeters; 
        boolean hasReachedRotationTolorence = Math.abs(this.endPoint.getRotation().getDegrees() - robotPose.getRotation().getDegrees()) <= 
            PathPlannerAutonConstants.kTranslationToleranceMeters; 
    
        boolean hasReachedTolorence = hasReachedXTolorence && hasReachedYTolorence && hasReachedRotationTolorence;
        
        return hasReachedTolorence;
    }
}