package frc.robot.Constants;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Do not create an instants of a constant class

public final class AutonConstants {
    public final class PIDPositioningAutonConstants {
        public static double kPTranslationPIDConstant = 3;
        public static double kITranslationPIDConstant = 0;
        public static double kDTranslationPIDConstant = 0;
        public static final double kMaxTranslationalSpeedInMetersPerSecond = 4.22;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = 1.4184 * 5;

        public static final TrapezoidProfile.Constraints kTranslationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxTranslationalSpeedInMetersPerSecond, kMaxTranslationalAccelerationInMetersPerSecond);

        public static double kPRotationPIDConstant = .4;
        public static double kIRotationPIDConstant = 0;
        public static double kDRotationPIDConstant = 0;
        public static final double kMaxRotationalSpeedInRadsPerSecond = 9.89199998351 * 5;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = 2.30100002339 * 5;
        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationalSpeedInRadsPerSecond, kMaxRotationalAccelerationInRadsPerSecond);
        public static final double kTranslationToleranceMeters = .05; 
        public static final double kRotationToleranceRadians = .06; 
    }

    public final class PathPlannerAutonConstants {
        public static PIDConstants kTranslationPIDConstants = new PIDConstants(2, 0, 0);
        public static PIDConstants kRotationPIDConstants = new PIDConstants(.03, 20, 0);
        public static final double kMaxModuleSpeedMetersPerSecond = 4.22;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = 1.4184;
        public static final double kMaxRotationalSpeedInDegrees = 566.76985;
        public static final double kMaxRotationalAccelerationInDegrees = 131.83759;
        public static final double kTranslationToleranceMeters = .03; 
        public static final double kRotationToleranceDegrees = 3; 
    }
}
