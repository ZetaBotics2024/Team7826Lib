package frc.robot.Constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Do not create an instants of a constant class

public final class AutonConstants {
    public final class PIDPositioningAutonConstants {
        public static double kPTranslationPIDConstant = RobotModeConstants.kIsNotSim ? .93 : 3;
        public static double kITranslationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static double kDTranslationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static final double kMaxTranslationalSpeedInMetersPerSecond = RobotModeConstants.kIsNotSim ? 20 : 4.22;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = RobotModeConstants.kIsNotSim ? 10 : 1.4184 * 5;

        public static final TrapezoidProfile.Constraints kTranslationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxTranslationalSpeedInMetersPerSecond, kMaxTranslationalAccelerationInMetersPerSecond);

        public static double kPRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0.0586 : .35;
        public static double kIRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static double kDRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static final double kMaxRotationalSpeedInRadsPerSecond = RobotModeConstants.kIsNotSim ? 9.89199998351 * 5 : 9.89199998351 * 5;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = RobotModeConstants.kIsNotSim ? 2.30100002339 * 5 : 2.30100002339 * 5;
        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationalSpeedInRadsPerSecond, kMaxRotationalAccelerationInRadsPerSecond);
        public static final double kTranslationToleranceMeters = RobotModeConstants.kIsNotSim ? .05 : .05; 
        public static final double kRotationToleranceRadians = RobotModeConstants.kIsNotSim ? 0.0349066 : 0.0349066; 
    }

    public final class PathPlannerAutonConstants {
        public static PIDConstants kTranslationPIDConstants = RobotModeConstants.kIsNotSim ? new PIDConstants(2, 0, 0) : new PIDConstants(2, 0, 0);
        public static PIDConstants kRotationPIDConstants = RobotModeConstants.kIsNotSim ? new PIDConstants(3.5, 0, 0) : new PIDConstants(3.5, 0, 0);
        public static final double kMaxModuleSpeedMetersPerSecond = RobotModeConstants.kIsNotSim ? 4.22 : 4.22;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = RobotModeConstants.kIsNotSim ? 1.4184 : 1.4184;
        public static final double kMaxRotationalSpeedInDegrees = RobotModeConstants.kIsNotSim ? 566.76985 : 566.76985;
        public static final double kMaxRotationalAccelerationInDegrees = RobotModeConstants.kIsNotSim ? 131.83759 : 131.83759;
        public static final double kTranslationToleranceMeters = RobotModeConstants.kIsNotSim ? 0.03 : 0.03; 
        public static final double kRotationToleranceDegrees = RobotModeConstants.kIsNotSim ? 3 : 3; 
    }

    public final class WPILibAutonConstants {
        public static double kPTranslationPIDConstant = RobotModeConstants.kIsNotSim ? 3 : 3;
        public static double kITranslationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static double kDTranslationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static final double kMaxTranslationalSpeedInMetersPerSecond = RobotModeConstants.kIsNotSim ? 4.22 : 4.22;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = RobotModeConstants.kIsNotSim ? 1.087 : 1.087;

        public static double kPRotationPIDConstant = RobotModeConstants.kIsNotSim ? .59 : .59;
        public static double kIRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static double kDRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static final double kMaxRotationalSpeedInRadsPerSecond = RobotModeConstants.kIsNotSim ? 9.89199998351 : 9.89199998351;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = RobotModeConstants.kIsNotSim ? 2.30100002339 : 2.30100002339;
        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationalSpeedInRadsPerSecond, kMaxRotationalAccelerationInRadsPerSecond);

        public static Pose2d kPositionTolorence = RobotModeConstants.kIsNotSim ?
            new Pose2d(.03, .03, Rotation2d.fromDegrees(3)) : new Pose2d(.03, .03, Rotation2d.fromDegrees(3));
    }

    public final class ChoreoAutonConstants {
        public static double kPTranslationPIDConstant = RobotModeConstants.kIsNotSim ? 5 : 5;
        public static double kITranslationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static double kDTranslationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;

        public static double kPRotationPIDConstant = RobotModeConstants.kIsNotSim ? 2.7 : 2.7;
        public static double kIRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
        public static double kDRotationPIDConstant = RobotModeConstants.kIsNotSim ? 0 : 0;
    }
}
