package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

// Do not create an instants of a constant class

public final class DrivetrainConstants {

    /**
     * Constants for the Swerve Drive
     */
    public static final class SwerveDriveConstants {
        // Driver Interaction constants
        public static final double kMaxSpeedMetersPerSecond = 4.3;
        public static final double kMaxRotationAnglePerSecond = 11.4;

        // Name of the CAN Bus the Swerve Drive is on.
        public static final String kCANLoopName = "rio"; // To swich to CANivor set the CANLoopName to the CANivors serial number or name

        // Swerve Module Configuration Constants
        public static final String kFrontLeftModuleName = "FrontLeftModule";
        public static final String kFrontRightModuleName = "FrontRightModule";
        public static final String kBackLeftModuleName = "BackLeftModule";
        public static final String kBackRightModuleName = "BackRightModule";

        // Drive Motor CANIDs
        public static final int kFrontLeftDriveMotorCANID = 2;
        public static final int kFrontRightDriveMotorCANID = 3;
        public static final int kBackLeftDriveMotorCANID = 4;
        public static final int kBackRightDriveMotorCANID = 5;

        public static final boolean kFrontLeftDriveMotorInverted =  false;
        public static final boolean kFrontRightDriveMotorInverted = false;
        public static final boolean kBackLeftDriveMotorInverted = false;
        public static final boolean kBackRightDriveMotorInverted = false;

        // Turn Motor CANIDs
        public static final int kFrontLeftTurnMotorCANID = 6;
        public static final int kFrontRightTurnMotorCANID = 7;
        public static final int kBackLeftTurnMotorCANID = 8;
        public static final int kBackRightTurnMotorCANID = 9;

        public static final boolean kFrontLeftTurnMotorInverted = true;
        public static final boolean kFrontRightTurnMotorInverted = true;
        public static final boolean kBackLeftTurnMotorInverted = true;
        public static final boolean kBackRightTurnMotorInverted = true;

        // Turning Absolute Encoder CANIDs
        public static final int kFrontLeftTurningAbsoluteEncoderCANID = 10;
        public static final int kFrontRightTurningAbsoluteEncoderCANID = 11;
        public static final int kBackLeftTurningAbsoluteEncoderCANID = 12;
        public static final int kBackRightTurningAbsoluteEncoderCANID = 13;

        // Turning Absolute Encoder Offsets in rotations
        public static final double kFrontLeftTurningAbsoluteEncoderOffsetRotations = 10;
        public static final double kFrontRightTurningAbsoluteEncoderOffsetRotations= 11;
        public static final double kBackLeftTurningAbsoluteEncoderOffsetRotations = 12;
        public static final double kBackRightTurningAbsoluteEncoderOffsetRotations = 13;
    
        // End of Swerve Module Configuration Constants

        public static final int kGyroCANID = 14; 

        // Kinematic Configuration

        public static final double kDistanceBetweenCentersOfRightAndLeftWheels = .60325;
        public static final double kDistanceBetweenCentersOfFrontAndBackWheels = .60325;
        public static final double kRadiusFromCenterToFarthestSwerveModule = Math
        .sqrt(((kDistanceBetweenCentersOfRightAndLeftWheels * kDistanceBetweenCentersOfRightAndLeftWheels)
            + (kDistanceBetweenCentersOfFrontAndBackWheels * kDistanceBetweenCentersOfFrontAndBackWheels)));

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            kDistanceBetweenCentersOfRightAndLeftWheels / 2),
        new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2,
            -kDistanceBetweenCentersOfRightAndLeftWheels / 2));
    }

    /**
     * Constants for the Swerve Modules
     */
    public static final class SwerveModuleConstants {
        public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
        public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

        // Motor Control Configuration Values
        public static final double kPModuleDrivePIDValue = 0.0001;
        public static final double kIModuleDrivePIDValue = .00000125;
        public static final double kDModuleDrivePIDValue = 0;
        public static final double kFFModuleDrivePIDValue = 0;
        public static final double kIZoneModuleDrivePIDValue = 0.0;

        public static final double kDriveMotorMinPercentOutput = -1;
        public static final double kDriveMotorMaxPercentOutput = 1;

        public static final double kPModuleTurnPIDValue = .1001;
        public static final double kIModuleTurnPIDValue = 0;
        public static final double kDModuleTurnPIDValue = 0;
        public static final double kFFModuleTurnPIDValue = 0;
        public static final double kIZoneModuleTurnPIDValue = 0.5 / 360; // 1/2 degrees converted to rotations

        public static final double kTurnMotorMinPercentOutput = -.2;
        public static final double kTurnMotorMaxPercentOutput = .2;

        // Swerve Module Configuration Values
        
        public static final double kAbsoluteTurningEncoderCPR = 4096.0;
        public static final double kNeoEncoderCPR = 4096.0;
        public static final double kMaxRPM = 5676.0;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3.733428923);// 3.83931974);//0.1016;
        public static final double kDriveGearRatio = (50.0 * 17.0 * 45.0) / (14.0 * 27.0 * 15.0);// 6.75/1.0;
        public static final double kTurningGearRatio = 150.0 / 7.0;

        public static final double kTurningConversionFactor = 360.0 / kTurningGearRatio;

        public static final double kAbsoluteTurningEncoderCPRToDegrees = (kAbsoluteTurningEncoderCPR
            / kAbsoluteTurningEncoderCPR) * 360.0;

        public static final double kAbsoluteTurningEncoderCPRToDegreesMult = 360.0 / 4096.0;

        public static final double kRelativeTurningEncoderDegreesToCPRMult = kNeoEncoderCPR / 360;
        // ((kNeoEncoderCPR / kNeoEncoderCPR) * 360) * kTurningGearRatio;

        public static final double kWheelDistancePerRotation = kWheelDiameterMeters * Math.PI;

        public static final double kDriveConversionPositionFactor = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
        public static final double kDriveConversionVelocityFactor = kDriveConversionPositionFactor / 60.0;

        public static final double kDriveEncoderDistancePerPulse = ((kWheelDiameterMeters * Math.PI) / kDriveGearRatio)
            / kNeoEncoderCPR;

        public static final double kTurningEncoderRadiansPerPulse =
            // Assumes the encoders are on a 1:1 reduction with the module shaft.
            (2 * Math.PI) / kAbsoluteTurningEncoderCPR;
        public static final int kDriveMotorMaxVoltage = 0;
        public static final int kTurnMotorMaxVoltage = 0;
    }
}
