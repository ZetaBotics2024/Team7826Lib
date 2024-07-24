package frc.robot.utils.GeneralUtils.AutonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotModeConstants;

public class AutonPoint {
    
    private Pose2d autonPoint;
    private FudgeFactor fudgeFactor;
    
    /**
     * Current Mirring assumes assumes mirred standered mirred field rather than a rotated or fliped.
     * @param xPointMeters
     * @param yPointMeters
     * @param rotationAngleDegrees
     * @param fudgeFactor
     */
    public AutonPoint(double xPointMeters, double yPointMeters, double rotationAngleDegrees, FudgeFactor fudgeFactor) {
        this.autonPoint = new Pose2d(xPointMeters, yPointMeters, Rotation2d.fromDegrees(rotationAngleDegrees));
        this.fudgeFactor = fudgeFactor;
    }

    public Pose2d getAutonPoint()
    {
        if(!RobotModeConstants.kIsBlueAlliance) {
            return new Pose2d(this.autonPoint.getX() + fudgeFactor.getRedFudgeFactors().getX(),
                (FieldConstants.kFieldWidthMeters - this.autonPoint.getY()) +
                fudgeFactor.getRedFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getRedFudgeFactors().getRotation().getDegrees()) * -1));
        }

        return new Pose2d(this.autonPoint.getX() + this.fudgeFactor.getBlueFudgeFactors().getX(),
            this.autonPoint.getY() + this.fudgeFactor.getBlueFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getBlueFudgeFactors().getRotation().getDegrees())));
        
    }  

    public Pose2d getAutonPoint(boolean shouldMirror) {
       if(shouldMirror) {
            return new Pose2d(this.autonPoint.getX() + fudgeFactor.getRedFudgeFactors().getX(),
                (FieldConstants.kFieldWidthMeters - this.autonPoint.getY()) +
                fudgeFactor.getRedFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getRedFudgeFactors().getRotation().getDegrees()) * -1));
        }

        return new Pose2d(this.autonPoint.getX() + this.fudgeFactor.getBlueFudgeFactors().getX(),
            this.autonPoint.getY() + this.fudgeFactor.getBlueFudgeFactors().getY(),
                Rotation2d.fromDegrees((this.autonPoint.getRotation().getDegrees() +
                this.fudgeFactor.getBlueFudgeFactors().getRotation().getDegrees())));
    }
}
