package frc.robot.Auton;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive.DriveCommandFactory;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;
import frc.robot.utils.AutonUtils.AutonPointUtils.FudgeFactor;

public class AutonManager {
    // Decloration of auton names
    private final String exampleAutonName = "ExampleAuton";

    // Decloration of auton chooser
    private LoggedDashboardChooser<String> autonChooser;

    // Decloration of all auton dependencies
    private AutonPointManager autonPointManager;
    private DriveCommandFactory driveCommandFactory;
    private DriveSubsystem driveSubsystem;

    public AutonManager(DriveCommandFactory driveCommandFactory, DriveSubsystem driveSubsystem) {
        this.autonPointManager = new AutonPointManager();
        this.driveCommandFactory = driveCommandFactory;
        this.driveSubsystem = driveSubsystem;

        this.autonChooser = new LoggedDashboardChooser<>("AutonChooser");
        addAllAutons();

    }

    private void addAllAutons() {
        addAuton(exampleAutonName);
        this.autonChooser.addDefaultOption(exampleAutonName, exampleAutonName);
    }

    private void addAuton(String autonName) {
        this.autonChooser.addOption(autonName, autonName);
    }

    public Command getSelectedAuton() {
        // This system allows for auton to be run multable times in one robot init. \
        // If this results in a noticable wait before the start of motion this can be swaped out before comp
        Command selectedAuton;
        switch (autonChooser.get()) {
            case exampleAutonName:
                selectedAuton = ExampleAuton.getExampleAuton(this.autonPointManager, this.driveCommandFactory, this.driveSubsystem);
                break;
            default:
                selectedAuton = new Command() {};
                Logger.recordOutput("InvalidAutonName", "InvalidAutonName");
        }
        return selectedAuton;
    }
}
