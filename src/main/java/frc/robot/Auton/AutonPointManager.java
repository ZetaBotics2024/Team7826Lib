package frc.robot.Auton;

import frc.robot.utils.AutonUtils.AutonPointUtils.AutonPoint;

public class AutonPointManager {
    // Robot start positions
    public final AutonPoint kExampleStartPoint;

    // Decloration of all auton points. 
    public final AutonPoint kExampleAutonPoint;
    public final AutonPoint kExampleAutonPoint1;
    public final AutonPoint kExampleAutonPoint2;
    public final AutonPoint kExampleAutonPoint3;
    public final AutonPoint kExampleAutonPoint4;
    public final AutonPoint[] kExampleAutonPointArray = new AutonPoint[4];

    // Decloration of PathPlanner PathNames
    public final String kExampleAutonName;

    public AutonPointManager() {
        this.kExampleStartPoint = new AutonPoint(1.5134, 7, 0);

        this.kExampleAutonPoint1 = new AutonPoint(1.75, 7.1, 60);
        this.kExampleAutonPoint2 = new AutonPoint(3, 6, 180);
        this.kExampleAutonPoint3 = new AutonPoint(5, 3, 90);
        this.kExampleAutonPoint4 = new AutonPoint(4, 5, 90);
        this.kExampleAutonPoint = new AutonPoint(12, 6, -30);
        
        configExampleAutonPointArray();

        this.kExampleAutonName = "ExampleAuton";
    }

    public void configExampleAutonPointArray() {
        this.kExampleAutonPointArray[0] = this.kExampleAutonPoint;
        this.kExampleAutonPointArray[1] = this.kExampleAutonPoint2;
        this.kExampleAutonPointArray[2] = this.kExampleAutonPoint3;
        this.kExampleAutonPointArray[3] = this.kExampleAutonPoint4;
    }
}
