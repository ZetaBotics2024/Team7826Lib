package frc.robot.Auton;

import frc.robot.Utils.AutonUtils.AutonPointUtils.AutonPoint;

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
    public final AutonPoint kExampleAutonEndPoint = new AutonPoint(4.7, 4, 0);

    public AutonPointManager() {
        this.kExampleStartPoint = new AutonPoint(1.5134, 7, 0);

        this.kExampleAutonPoint1 = new AutonPoint(1.5134, 7, 0);
        this.kExampleAutonPoint2 = new AutonPoint(3, 6, 0);
        this.kExampleAutonPoint3 = new AutonPoint(5, 7, 0);
        this.kExampleAutonPoint4 = new AutonPoint(7, 8 , 90);
        this.kExampleAutonPoint = new AutonPoint(3, 3, 180);
        
        configExampleAutonPointArray();

        this.kExampleAutonName = "ExampleAuton";
    }

    public void configExampleAutonPointArray() {
        this.kExampleAutonPointArray[0] = this.kExampleAutonPoint1;
        this.kExampleAutonPointArray[1] = this.kExampleAutonPoint2;
        this.kExampleAutonPointArray[2] = this.kExampleAutonPoint3;
        this.kExampleAutonPointArray[3] = this.kExampleAutonPoint4;
    }
}
