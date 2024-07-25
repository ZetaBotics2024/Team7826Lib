package frc.robot.Auton;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class AutonPointManager {
    // Robot start positions
    public final AutonPoint kExampleStartPoint;

    // Decloration of all auton points. 
    public final AutonPoint kExampleAutonPoint;

    public AutonPointManager() {
        this.kExampleStartPoint = new AutonPoint(0, 0, 0);

        this.kExampleAutonPoint = new AutonPoint(2, 3, 50,
            new FudgeFactor(0.1, 0.3,
            7, -0.1, 0.2, 11));
    }
}
