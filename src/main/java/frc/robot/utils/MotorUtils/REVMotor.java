package frc.robot.utils.MotorUtils;

import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

public class REVMotor implements Motor{
    // Instants Variables

    // Motor Configuration
    private int CANID;
    private boolean motorInverted;
    private CANSparkMax motor;
    
    
    public void setMotorInverted(boolean motorInverted) {
        this.motorInverted = motorInverted;
    }
    
    public boolean isMotorInverted() {
        return this.motorInverted;
    }

    @Override
    public void init(int CANID) {

    }

    @Override
    public void setPID(int slot, double p, double i, double d, double f) {
    }

    @Override
    public void setIZone(double iZone) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setIZone'");
    }

    @Override
    public void setDFilter(double dFilter) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDFilter'");
    }

    @Override
    public void enableBreakMode(boolean breakModeEnabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'enableBreakMode'");
    }

    @Override
    public void setInverted(boolean motorInverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public boolean isInverted() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'isInverted'");
    }

    @Override
    public void setVelocityTempetureVoltageAndCurrentCANInterval(double milliseconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVelocityTempetureVoltageAndCurrentCANInterval'");
    }

    @Override
    public void setMotorPositionCANInterval(double milliseconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMotorPositionCANInterval'");
    }

    @Override
    public void setAnalogSensorVoltageAnalogSensorVelocityAndAnalogSensorPositionCANInterval(double milliseconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAnalogSensorVoltageAnalogSensorVelocityAndAnalogSensorPositionCANInterval'");
    }

    @Override
    public void setAlternateEncoderVelocityAndAlternateEncoderPositionCANInterval(double milliseconds) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setAlternateEncoderVelocityAndAlternateEncoderPositionCANInterval'");
    }

    @Override
    public void setCurrentLimit(double limit) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimit'");
    }

    /**
     * Sets the motor to follow the motor passed into this method, e.g have the motor mirror the other motors movement
     * @param motor CANSparkMax: The motor that the motor will follow
     * @param invert Boolean: Whether or not the motor should do the opisite of what the motor it is following does. e.g move in the opisite direction
     */
    public void follow(CANSparkMax motor, boolean invert) {
        TalonFX ctre = new TalonFX(CANID);
        this.motor.setClosedLoopRampRate();
    }

}
