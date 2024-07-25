package frc.robot.Constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

// Do not create an instants of a constant class

public final class AutonConstants {
    public final class PIDPositioningAutonConstants {
        public static double kPTranslationPIDConstant = 3;
        public static double kITranslationPIDConstant = 0;
        public static double kDTranslationPIDConstant = 0;
        public static final double kMaxTranslationalSpeedInMetersPerSecond = 4.3;
        public static final double kMaxTranslationalAccelerationInMetersPerSecond = 5;

        public static final TrapezoidProfile.Constraints kTranslationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxTranslationalSpeedInMetersPerSecond, kMaxTranslationalAccelerationInMetersPerSecond);

        public static double kPRotationPIDConstant = .4;
        public static double kIRotationPIDConstant = 0;
        public static double kDRotationPIDConstant = 0;
        public static final double kMaxRotationalSpeedInMetersPerSecond = 11.5;
        public static final double kMaxRotationalAccelerationInRadsPerSecond = 20;

        public static final TrapezoidProfile.Constraints kRotationPIDControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxRotationalSpeedInMetersPerSecond, kMaxRotationalAccelerationInRadsPerSecond);
    }
}
