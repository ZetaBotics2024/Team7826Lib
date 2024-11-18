package frc.robot.Utils.SwerveDriveUtils;

public class DesiredMetersPerSecondToVoltage {
    
    public static double metersPerSecondToVoltage(double desiredMetersPerSecond) {
        double desiredVoltage = desiredMetersPerSecond == 0 ? 0 :  
            (Math.abs(desiredMetersPerSecond) <= 0 ? applyLinearRegressionModel(desiredMetersPerSecond) :
            applyLinearRegressionModel(desiredMetersPerSecond));
        return desiredVoltage;
    }

    private static double applyLinearRegressionModel(double value) {
        double slope = 2.823;
        double yIntersept = 0.1443;
        double sine = Math.signum(value);
        return ((slope * Math.abs(value)) + yIntersept) * sine;
    }

    private static double applySecondRegressionModel(double value) {
        double slope = 2.823;
        double yIntersept = 0.1443;
        double sine = Math.signum(value);
        return ((slope * Math.abs(value)) + yIntersept) * sine;
    }
}
