package frc.robot.commands.AutonCommands.PathplannerAutonCommands;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants.PathPlannerAutonConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;

public class PathPlannerCreateAutonFromPoints {

    public static Command createAutonCommand(AutonPoint[] goalPoints, double maxTime, DriveSubsystem driveSubsystem) {
        AutonPoint startPoint = goalPoints[0];
        AutonPoint endPoint = goalPoints[goalPoints.length-1];
        Pose2d realStartPoint = new Pose2d(startPoint.getAutonPoint().getTranslation(), new Rotation2d());
        Pose2d realEndPoint = new Pose2d(endPoint.getAutonPoint().getTranslation(), new Rotation2d());
   
        ArrayList<Pose2d> pointsAsPose2d = new ArrayList<>();
        pointsAsPose2d.add(realStartPoint);
        for(int i = 1; i < goalPoints.length-1; i++) {
            //pointsAsPose2d.add(goalPoints[i].getAutonPoint());
        }
        pointsAsPose2d.add(realEndPoint);

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
                realStartPoint,
                realEndPoint
        );
        
        PathPlannerPath path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(PathPlannerAutonConstants.kMaxModuleSpeedMetersPerSecond,
                    PathPlannerAutonConstants.kMaxTranslationalAccelerationInMetersPerSecond,
                    PathPlannerAutonConstants.kMaxRotationalSpeedInDegrees,
                    PathPlannerAutonConstants.kMaxRotationalAccelerationInDegrees), 
                new GoalEndState(0.0, Rotation2d.fromDegrees(endPoint.getAutonPoint().getRotation().getDegrees()))
        );

        path.preventFlipping = true;

        return new PathPlannerFollowPath(path, endPoint.getAutonPoint(), maxTime, driveSubsystem);
    }
}
