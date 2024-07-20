package frc.robot.Constants;

public class RobotModeConstants {
    public static final Mode currentMode = Mode.SIM;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,
        /** Running a physics simulator. */
        SIM,
        /** Replaying from a log file. */
        REPLAY
    }
}
