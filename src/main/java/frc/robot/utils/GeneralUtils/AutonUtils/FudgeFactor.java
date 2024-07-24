package frc.robot.utils.GeneralUtils.AutonUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FudgeFactor {
    
    private Pose2d blueFudgeFactors;
    private Pose2d redFudgeFactors;

    public FudgeFactor(double xFudgeFactorMetersBlue, double yFudgeFactorMetersBlue, double rotationAngleFudgeFactorDegreesBlue,
        double xFudgeFactorMetersRed, double yFudgeFactorMetersRed, double rotationAngleFudgeFactorDegreesRed) {

        this.blueFudgeFactors = new Pose2d(xFudgeFactorMetersBlue, yFudgeFactorMetersBlue, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegreesBlue));
        this.redFudgeFactors = new Pose2d(xFudgeFactorMetersRed, yFudgeFactorMetersRed, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegreesRed));
    }

    public FudgeFactor(double xFudgeFactorMeters, double yFudgeFactorMeters, double rotationAngleFudgeFactorDegrees) {

        this.blueFudgeFactors = new Pose2d(xFudgeFactorMeters, yFudgeFactorMeters, Rotation2d.fromDegrees(rotationAngleFudgeFactorDegrees));
        this.redFudgeFactors = this.blueFudgeFactors;
    }

    public Pose2d getBlueFudgeFactors() {
        return this.blueFudgeFactors;
    }

    public Pose2d getRedFudgeFactors() {
        return this.redFudgeFactors;
    }
}
