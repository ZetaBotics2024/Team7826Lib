package frc.robot.Subsystems.SwerveDrive.SwerveModule;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.LoggerConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveDriveConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveModuleConstants;
import frc.robot.Utils.GeneralUtils.NetworkTableChangableValueUtils.NetworkTablesTunablePIDConstants;

public class SwerveModuleIOSparkMax implements SwerveModuleIO{

    private String swerveModuleName; 

    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;

    private SparkPIDController drivePIDController;
    private SparkPIDController turnPIDController;

    private RelativeEncoder driveRelativeEncoder;

    private CANcoder turnAbsoluteEncoder;
    private RelativeEncoder turnRelativeEncoder;

    private double turningAbsoluteEncoderOffset;

    private NetworkTablesTunablePIDConstants driveMotorPIDConstantTuner;
    private NetworkTablesTunablePIDConstants turnMotorPIDConstantTuner;

    /**
     * Creates a SwerveModuleIOSparkMax object and completes all configuration for the module
     * @param swerveModuleName String: The name of the module. List of valid module names can be found in SwerveDriveConstants
     */
    public SwerveModuleIOSparkMax(String swerveModuleName) {
        this.swerveModuleName = swerveModuleName;

        switch (this.swerveModuleName) {
            case SwerveDriveConstants.kFrontLeftModuleName:
                configDriveMotor(SwerveDriveConstants.kFrontLeftDriveMotorCANID, SwerveDriveConstants.kFrontLeftDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kFrontLeftTurnMotorCANID, SwerveDriveConstants.kFrontLeftTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kFrontLeftTurningAbsoluteEncoderCANID, SwerveDriveConstants.kFrontLeftTurningAbsoluteEncoderOffsetRotations);
                break;
            case SwerveDriveConstants.kFrontRightModuleName:
                configDriveMotor(SwerveDriveConstants.kFrontRightDriveMotorCANID, SwerveDriveConstants.kFrontRightDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kFrontRightTurnMotorCANID, SwerveDriveConstants.kFrontRightTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kFrontRightTurningAbsoluteEncoderCANID, SwerveDriveConstants.kFrontRightTurningAbsoluteEncoderOffsetRotations);
                break;
            case SwerveDriveConstants.kBackLeftModuleName:
                configDriveMotor(SwerveDriveConstants.kBackLeftDriveMotorCANID, SwerveDriveConstants.kBackLeftDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kBackLeftTurnMotorCANID, SwerveDriveConstants.kBackLeftTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kBackLeftTurningAbsoluteEncoderCANID, SwerveDriveConstants.kBackLeftTurningAbsoluteEncoderOffsetRotations);
                break;
            case SwerveDriveConstants.kBackRightModuleName:
                configDriveMotor(SwerveDriveConstants.kBackRightDriveMotorCANID, SwerveDriveConstants.kBackRightDriveMotorInverted);
                configTurnMotor(SwerveDriveConstants.kBackRightTurnMotorCANID, SwerveDriveConstants.kBackRightTurnMotorInverted);
                configTurningAbsoluteEncoder(SwerveDriveConstants.kBackRightTurningAbsoluteEncoderCANID, SwerveDriveConstants.kBackRightTurningAbsoluteEncoderOffsetRotations);
                break;
            default:
                throw new RuntimeException("Invalid Module Name: " + swerveModuleName + ", Please change to a valid name. List of valid names can be found in SwerveModuleConstants");
        }

        this.driveRelativeEncoder = this.driveMotor.getEncoder();
        this.turnRelativeEncoder = this.turnMotor.getEncoder();

        configDirvePID();
        configTurnPID();

        this.driveMotor.burnFlash();
        this.turnMotor.burnFlash();

        Timer.delay(1); // We should see if we can reduce this to dramaticly increase robot boot time. 
        resetTurningMotorToAbsolute();

        addInitLogs();
    }

    /**
     * Configures the drive motor.
     * @param driveMotorID Integer: The CANID for the drive motor
     * @param driveMotorInverted Boolean: Wether or not the drive motor is inverted
     */
    private void configDriveMotor(int driveMotorID, boolean driveMotorInverted) {
        this.driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        this.driveMotor.restoreFactoryDefaults();

        this.driveMotor.setInverted(driveMotorInverted);
        this.driveMotor.setIdleMode(IdleMode.kBrake);

        this.driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
        this.driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);
        this.driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

        this.driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kDriveMotorMaxAmpsSparkMax);
        this.driveMotor.enableVoltageCompensation(12);
    }

     /**
      * Configures the drive motor PID Controller. 
      */
    private void configDirvePID() {
        this.drivePIDController = this.driveMotor.getPIDController();
        
        this.drivePIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

        this.drivePIDController.setP(SwerveModuleConstants.kPModuleDrivePIDValue, 0);
        this.drivePIDController.setI(SwerveModuleConstants.kIModuleDrivePIDValue, 0);
        this.drivePIDController.setD(SwerveModuleConstants.kDModuleDrivePIDValue, 0);
        this.drivePIDController.setFF(SwerveModuleConstants.kFFModuleDrivePIDValue, 0);
        this.drivePIDController.setIZone(SwerveModuleConstants.kIZoneModuleDrivePIDValue, 0);
        this.drivePIDController.setOutputRange(SwerveModuleConstants.kDriveMotorMinPercentOutput, SwerveModuleConstants.kDriveMotorMaxPercentOutput);

         this.driveMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("SwerveModule/DrivePIDValues",
            SwerveModuleConstants.kPModuleDrivePIDValue,
            SwerveModuleConstants.kIModuleDrivePIDValue,
            SwerveModuleConstants.kDModuleDrivePIDValue, 0);
    }

    /**
     * Configures the turn motor.
     * @param turnMotorID Integer: The CANID for the turn motor
     * @param turnMotorInverted Boolean: Wether or not the turn motor is inverted
     */
    private void configTurnMotor(int turnMotorID, boolean turnMotorInverted) {
        this.turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        this.turnMotor.restoreFactoryDefaults();

        this.turnMotor.setInverted(turnMotorInverted);
        this.turnMotor.setIdleMode(IdleMode.kBrake);

        this.turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
        this.turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
        this.turnMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 500);

        this.driveMotor.setSmartCurrentLimit(SwerveModuleConstants.kTurnMotorMaxAmpsSparkMax);
    }
    
    /**
      * Configures the turn motor PID Controller. 
      */
    private void configTurnPID() {
        this.turnPIDController = this.turnMotor.getPIDController();

        this.turnPIDController.setP(SwerveModuleConstants.kPModuleTurnPIDValue, 0);
        this.turnPIDController.setI(SwerveModuleConstants.kIModuleTurnPIDValue, 0);
        this.turnPIDController.setD(SwerveModuleConstants.kDModuleTurnPIDValue, 0);
        this.turnPIDController.setFF(SwerveModuleConstants.kFFModuleTurnPIDValue, 0);
        this.turnPIDController.setIZone(SwerveModuleConstants.kIZoneModuleTurnPIDValue, 0);
        this.turnPIDController.setOutputRange(SwerveModuleConstants.kTurnMotorMinPercentOutput, SwerveModuleConstants.kTurnMotorMaxPercentOutput);

        this.turnMotorPIDConstantTuner = new NetworkTablesTunablePIDConstants("SwerveModule/TurnPIDValues",
            SwerveModuleConstants.kPModuleTurnPIDValue,
            SwerveModuleConstants.kIModuleTurnPIDValue,
            SwerveModuleConstants.kDModuleTurnPIDValue, 0);
    }

    /**
     * Configures the turnAbsoluteEncoder
     * @param turnAbsoluteEncoderID Integer: The CANID of the turnAbsoluteEncoder
     * @param turningAbsoluteEncoderOffset Double: The offset of the turning absolute encoder. i.e the difforence between the encoders value and whare the wheel is straight.
     * Should be calucluted by useing a straight edge to make all wheels perfectly straight forward(Make sure all wheels are in the same orientation). 
     * From there the value can be gotton from Pheonix Tuner. 
     */
    private void configTurningAbsoluteEncoder(int turnAbsoluteEncoderID, double turningAbsoluteEncoderOffset) {
        this.turnAbsoluteEncoder = new CANcoder(turnAbsoluteEncoderID, SwerveDriveConstants.kCANLoopName);
        CANcoderConfiguration turningAbsoluteEncoderConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magnetConfigs = new MagnetSensorConfigs();
        magnetConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        magnetConfigs.MagnetOffset = 0.0f;
        magnetConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        turningAbsoluteEncoderConfig.MagnetSensor = magnetConfigs;
        this.turnAbsoluteEncoder.getConfigurator().apply(turningAbsoluteEncoderConfig);

        this.turningAbsoluteEncoderOffset = turningAbsoluteEncoderOffset;
    }

    /**
     * Resets the relitive built in encoder of the turn motor to be equal to the offsetted absolute angle reading from the absolute encoder.
     */
    public void resetTurningMotorToAbsolute() {
        this.turnRelativeEncoder.setPosition((
            this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() 
            - this.turningAbsoluteEncoderOffset) * SwerveModuleConstants.kTurningGearRatio);
      }
    
    /**
     * Adds Swerve Module logs that never need to be updated to the dashboard.
     */
    private void addInitLogs() {
        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + "TurningAbsoluteEncoderOffset", this.turningAbsoluteEncoderOffset);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        inputs.driveMotorRotations = this.driveRelativeEncoder.getPosition();
        inputs.driveMotorRPM = this.driveRelativeEncoder.getVelocity();
        inputs.driveMotorSpeedMetersPerSecond = inputs.driveMotorRPM * SwerveModuleConstants.kDriveConversionVelocityFactor;
        inputs.driveMotorDistanceMeters = (inputs.driveMotorRotations / SwerveModuleConstants.kDriveGearRatio) * SwerveModuleConstants.kWheelDistancePerRotation;
        inputs.driveMotorAppliedVolts = this.driveMotor.getAppliedOutput() * this.driveMotor.getBusVoltage();
        inputs.driveMotorCurrentAmps = new double[] {this.driveMotor.getOutputCurrent()};

        inputs.turnMotorAbsolutePositionRotations = this.turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble();
        inputs.turnMotorRelitivePositionRotations = this.turnRelativeEncoder.getPosition();
        inputs.wheelAngleRelitivePositionRotations = inputs.turnMotorRelitivePositionRotations / SwerveModuleConstants.kTurningGearRatio;
        inputs.turnMotorRPM = this.turnRelativeEncoder.getVelocity();
        inputs.turnMotorAppliedVolts = turnMotor.getAppliedOutput() * turnMotor.getBusVoltage();
        inputs.turnMotorCurrentAmps = new double[] {this.turnMotor.getOutputCurrent()};

        updatePIDValuesFromNetworkTables();
    }

    /**
     * WORNING!!! There should only be one call of this method and that
     *  call should be commented out before going to a competition. 
     * Updates the PID values for the module bassed on network tables.
     * Must be called periodicly.
     */
    private void updatePIDValuesFromNetworkTables() {
        double[] currentDrivePIDValues = this.driveMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.driveMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.drivePIDController.setP(currentDrivePIDValues[0]);
            this.drivePIDController.setI(currentDrivePIDValues[1]);
            this.drivePIDController.setD(currentDrivePIDValues[2]);
            this.drivePIDController.setFF(currentDrivePIDValues[3]);
            this.driveMotor.burnFlash();
        }
        
        double[] currentTurnPIDValues = this.turnMotorPIDConstantTuner.getUpdatedPIDConstants();
        if(this.turnMotorPIDConstantTuner.hasAnyPIDValueChanged()) {
            this.turnPIDController.setP(currentTurnPIDValues[0]);
            this.turnPIDController.setI(currentTurnPIDValues[1]);
            this.turnPIDController.setD(currentTurnPIDValues[2]);
            this.turnPIDController.setFF(currentTurnPIDValues[3]);
            this.turnMotor.burnFlash();
        }
        
    }

    @Override
    public void setDesiredModuleVelocityRPM(double desiredRPM) {
        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + "DesiredRPM", desiredRPM);
        this.drivePIDController.setReference(desiredRPM, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {
        double desiredModuleRotations = desiredModuleAngle.getRotations(); 
        double desiredMotorRotation = desiredModuleRotations * SwerveModuleConstants.kTurningGearRatio;

        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + "DesiredModuleRotations" , desiredModuleRotations);
        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + "DesiredMotorRotations" , desiredMotorRotation);

        turnPIDController.setReference(desiredMotorRotation, CANSparkMax.ControlType.kPosition); 
    }
}
