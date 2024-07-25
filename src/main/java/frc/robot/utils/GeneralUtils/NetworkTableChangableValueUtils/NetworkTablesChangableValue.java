package frc.robot.utils.GeneralUtils.NetworkTableChangableValueUtils;

import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.google.flatbuffers.FlexBuffers.Key;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NetworkTablesChangableValue {
    
    private LoggedDashboardNumber changableValue;
    private double lastChangableValue;
    private String networkTablesKey = "NoChangableValueKeyProvided";
    private boolean hasChangableValueChanged = false;


    /**
     * WORNING!!! THIS WILL NOT WORK DURING MATCHES AT COMPETION.
     * ALL VALUES TUNED HERE SHOULD BE ADDED TO THE RELEVENT CONSTANT FILE ONCE TUNING IS COMPLETE!!!
     * WORNING!!! VALUES MAY NOT STAY BETWEEN DASHBOARD OR ROBOT REBOOTS/CODE REDEPLOYS.
     * Please record the value somehow before proforming these actions to not loose progress
     * Adds a changable value to Smartdashboard. This allows for PID values to be 
     * tuned without redeploying code every change or for flywheel speed tuning, or other things of this nature.
     * @param networkTablesKey String: The key underwhich the value will be added to networktables. 
     * @param changableValue The inital value for the changable value. This can be a Integer, Double or Boolean
     */
    public NetworkTablesChangableValue(String networkTablesKey, double changableValue) {
        this.changableValue = new LoggedDashboardNumber(networkTablesKey, changableValue);
        this.networkTablesKey = networkTablesKey;
        this.lastChangableValue = this.changableValue.get();
    }   


    /**
     * Returns the current changableValue's value as a genaric object. Must be cased to desired type
     * @return Object: The current changableValue's value as a genaric object. Must be cased to desired type
     */
    public double getChangableValueOnNetworkTables() {
        if(this.changableValue.get() != this.lastChangableValue) {
            this.lastChangableValue = this.changableValue.get();
            this.hasChangableValueChanged = true;
        }

        return this.changableValue.get();
    }

    /**
     * Returns Wether or not the changabelValue has changed sence last check and sets it to false if it has.
     * @return Boolean: Wether or not the changabelValue has changed sence last check
     */
    public boolean hasChangableValueChanged() {
        boolean originalValue = this.hasChangableValueChanged;
        if(this.hasChangableValueChanged) {
            this.hasChangableValueChanged = false;
        }
        return originalValue;
    }
}
