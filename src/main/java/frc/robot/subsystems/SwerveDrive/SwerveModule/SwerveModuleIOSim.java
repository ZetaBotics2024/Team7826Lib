package frc.robot.subsystems.SwerveDrive.SwerveModule;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.LoggerConstants;
import frc.robot.Constants.DrivetrainConstants.SwerveModuleConstants;

public class SwerveModuleIOSim implements SwerveModuleIO{
    private static final double LOOP_PERIOD_SECS = 0.02;

    private DCMotorSim driveMotor = new DCMotorSim(DCMotor.getNEO(1), 6.75, 0.025);
    private DCMotorSim turnMotor = new DCMotorSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    private PIDController drivePIDController; 
    private PIDController turnPIDController; 
    
    private double desiredVelocityRPM = 0;
    private double desiredPositionRotations = 0;

    private String swerveModuleName;

    public SwerveModuleIOSim(String swerveModuleName) {
        this.swerveModuleName = swerveModuleName;
        configDirvePID();
        configTurnPID();
    }

    private void configDirvePID() {
        this.drivePIDController = new PIDController(SwerveModuleConstants.kPModuleDrivePIDValue,
            SwerveModuleConstants.kIModuleDrivePIDValue,
            SwerveModuleConstants.kDModuleDrivePIDValue);
    }
    
    /**
      * Configures the turn motor PID Controller. 
      */
    private void configTurnPID() {
        this.turnPIDController = new PIDController(SwerveModuleConstants.kPModuleTurnPIDValue,
            SwerveModuleConstants.kIModuleTurnPIDValue,
            SwerveModuleConstants.kDModuleTurnPIDValue);
    }

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveMotor.update(LOOP_PERIOD_SECS);
        turnMotor.update(LOOP_PERIOD_SECS);

        double driveVolts = MathUtil.clamp(this.drivePIDController.calculate(this.driveMotor.getAngularVelocityRPM(), this.desiredVelocityRPM), -12, 12);
        double turnVolts = MathUtil.clamp(this.turnPIDController.calculate(this.turnMotor.getAngularPositionRotations(), this.desiredPositionRotations), -12, 12);

        inputs.driveMotorRotations = this.driveMotor.getAngularPositionRotations();
        inputs.driveMotorRPM = this.driveMotor.getAngularVelocityRPM();
        inputs.driveMotorSpeedMetersPerSecond = inputs.driveMotorRPM * SwerveModuleConstants.kDriveConversionVelocityFactor;
        inputs.driveMotorDistanceMeters = (inputs.driveMotorRotations / SwerveModuleConstants.kDriveGearRatio) * SwerveModuleConstants.kWheelDistancePerRotation;
        inputs.driveMotorAppliedVolts = driveVolts;
        inputs.driveMotorCurrentAmps = new double[] {this.driveMotor.getCurrentDrawAmps()};

        inputs.turnMotorAbsolutePositionRotations = this.turnMotor.getAngularPositionRotations();
        inputs.turnMotorRelitivePositionRotations = this.turnMotor.getAngularPositionRotations();
        inputs.wheelAngleRelitivePositionRotations = inputs.turnMotorRelitivePositionRotations / SwerveModuleConstants.kTurningGearRatio;
        inputs.turnMotorRPM = this.turnMotor.getAngularVelocityRPM();
        inputs.turnMotorAppliedVolts = turnVolts;
        inputs.turnMotorCurrentAmps = new double[] {this.turnMotor.getCurrentDrawAmps()};
        
        this.driveMotor.setInputVoltage(driveVolts);
        this.turnMotor.setInputVoltage(turnVolts);
    }

    
    public void setDesiredModuleVelocityRPM(double desiredRPM) {
        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + " Desired RPM", desiredRPM);

        this.desiredVelocityRPM = desiredRPM;
    }

    public void setDesiredModuleAngle(Rotation2d desiredModuleAngle) {
        double desiredModuleRotations = desiredModuleAngle.getRotations(); 
        double desiredMotorRotation = desiredModuleRotations * SwerveModuleConstants.kTurningGearRatio;

        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + "DesiredModuleRotations" , desiredModuleRotations);
        Logger.recordOutput(LoggerConstants.kModuleOutputLoggingMenu + swerveModuleName + "DesiredMotorRotations" , desiredMotorRotation);

        this.desiredPositionRotations = desiredModuleRotations * SwerveModuleConstants.kTurningGearRatio;
    }
}
