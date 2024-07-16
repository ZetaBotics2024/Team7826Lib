package frc.robot.utils.MotorUtils;

import com.revrobotics.CANSparkFlex;

/**
 * Motor
 */
public interface Motor {
 
    /**
     * Must be called before anyother motor method
     * @param CANID The CAN ID of the motor(This can not be changed)
     */
    public void init(int CANID);

    /**
     * Enables or disables break mode
     * @param breakModeEnabled Boolean: Pass in true to enable and false to disable
     */
    public void enableBreakMode(boolean breakModeEnabled);

    /**
     * Sets whether motor is inverted
     * @param motorInverted Boolean: Sets wether the motor is inverted
     */
    public void setInverted(boolean motorInverted);

    /**
     * Gets whether motor is inverted
     * @return Boolean: Wether or not the motor is inverted
     */
    public boolean isInverted();

    /**
     * Sets the interval on which updates are sent for the motors Velocity, Tempeture, Voltage and Current.
     * @param milliseconds Double: The interval in milliseconds. The default is 20ms for SparkMax.
     */
    public void setVelocityTempetureVoltageAndCurrentCANInterval(double milliseconds);

    /**
     * Sets the interval on which updates are sent for the motors position
     * @param milliseconds Double: The interval in milliseconds. The default is 20ms for SparkMax.
     */
    public void setMotorPositionCANInterval(double milliseconds);

    /**
     * Sets the interval on which updates are sent for the motors Analog Sensor Voltage, Analog Sensor Velocity, and Analog Sensor Position(This is probably fine to increase to 500ms)
     * @param milliseconds Double: The interval in milliseconds. The default is 50ms for SparkMax.
     */
    public void setAnalogSensorVoltageAnalogSensorVelocityAndAnalogSensorPositionCANInterval(double milliseconds);

    /**
     * Sets the interval on which updates are sent for the motors Alternate Encoder Velocity and Alternate Encoder Position
     * @param milliseconds Double: The interval in milliseconds. The default is 20ms for SparkMax.
     */
    public void setAlternateEncoderVelocityAndAlternateEncoderPositionCANInterval(double milliseconds);

    /**
     * Sets the max amps(for REV, Volts for CTRE) the motor can pull while being unable to move(Provents motor from burning out)
     * @param limit Double: The motors power limit. 40amps is the max you should set for REV and 12 Volts is the max to input for CTRE.
     */
    public void setCurrentLimit(double limit);

    /**
     * Sets the max rotatinal movemeant of the motor in both directions. e.g the motor will not move passed the limit
     * enableRotationLimit() must be called after the limits are set for them to take effect.
     * @param positiveLimit
     * @param negitiveLimit
     */
    public void setRotationLimit(double positiveLimit, double negitiveLimit);

    /**
     * Enables rotatinal limits for the motor. Must be set via setRotationLimit() before this is called for it to have an effect.
     * @param enableLimits
     */
    public void enableRotationLimit(boolean enableLimits); 

    /**
     * Enables and sets the voltage compensation for the motor
     * This sets a new "max" voltage that the motor can use. This is used to compensate in the event that not the full 12 volts can ever reach the motor.
     * There is a chance this description is wrong.
     * @param Voltage The voltage that will be compensated to
     */
    public void enableVoltageCompensation(double voltage);

    /**
     * Enables and sets the voltage compensation for the motor
     * This sets a new "max" voltage that the motor can use. This is used to compensate in the event that not the full 12 volts can ever reach the motor.
     * There is a chance this description is wrong.
     * @param Voltage The new "max"
     */
    public void disableVoltageCompensation();

    /**
     * Gets the voltage that the motor controller is being supplied
     * @return Double: The voltage that the motor controller is being supplied
     */
    public double getSuppliedVoltage();

    /**
     * Gets the amperage that the motor controller is outputting
     * @return Double: The voltage that the motor controller is outputting
     */
    public double getOutputCurrent();

    /**
     * Gets the motor temperature
     * @return Double: The motor temperature
     */
    public double getMotorTemperature();


    /**
     * Sets PID Control Loop Constants
     * @param slot Integer: Which PID Loop to set the constants of. Only 0-3 are valid 
     * @param p Double:  Value for the P(Proportional) constant. 
     * @param i Double:  Value for the I(Integral) constant. 
     * @param d Double:  Value for the D(Derivative) constant. 
     * @param f Double:  Value for the F(Feedforward) constant. 
     */
    public void setPID(int slot, double p, double i, double d, double f);

    /**
     * Sets the range that the |error| must be to be applied 
     * @param iZone Double: The range that the |error| must be to be applied 
     */
    public void setIZone(double iZone);

    /**
     * A derivative filter smooths out the noise by applying a low-pass filter to the derivative term of the PID loop, 
     * which attenuates the high-frequency components and preserves the low-frequency ones
     * @param dFilter Double: The value of the D Filter 
    */
    public void setDFilter(double dFilter);
} 
