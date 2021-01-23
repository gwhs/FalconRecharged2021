package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.utility.MathUtils;
import frc.robot.utility.TrajectoryMaker;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class TrajectoryHelper {

    public static double[][] slalom = { 
        { 30, 30 }, // Starting point
        { 80, 35 }, 
        { 125, 90 }, 
        { 180, 110 }, 
        { 250, 80 }, 
        { 285, 30 }, 
        { 340, 45 }, 
        { 320, 100 }, 
        { 275, 80 },
        { 250, 35 }, 
        { 180, 25 }, 
        { 120, 40 }, 
        { 80, 70 }, 
        { 30, 100 }
     };


    /**
     * translateAndScale takes an array of integer coordinates in 2-d space, and scales them to meters, and applies a scale in additinoos
     * Omits the first and last points
     * @param pointsArray  arrray of more than two X,Y coordinates
     * @param scale  resize the entire grid
     * @return
     */
    public static ArrayList<Translation2d> translateAndScale(double[][] pointsArray, double scale) {
        ArrayList<Translation2d> points = new ArrayList<>(pointsArray.length-1);
        // translate all the points to the initial coordinate
        double initialX = pointsArray[0][0];
        double initialY = pointsArray[0][1];

        for ( int i = 1; i < pointsArray.length; i++) {
            // also; convert from inches to meters
            // translate to 0,0
            double x = pointsArray[i][0];
            double y = pointsArray[i][1];
            x = x - initialX; // translate points to be relative to starting point
            y = y - initialY;
            x = MathUtils.inchesToMeters(x);  // convert to metric
            y = MathUtils.inchesToMeters(y);
            x = x * scale;  // apply extra scale
            y = y * scale;
            points.add(new Translation2d(x, y));
        }
        return points;
    }

    private static TrajectoryMaker createTrajectory(double [][] inputPoints, double scale)
    {
        ArrayList<Translation2d> points = translateAndScale(inputPoints, scale);  // make .2 for Hajel's garage.  Turns the 30 foot field to 6 feet
        Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(0));
        Translation2d lastPoint = points.remove(points.size()-1);  // remove last point in array
        Pose2d endPose = new Pose2d(lastPoint.getX(), lastPoint.getY(), new Rotation2d(0));

        return new TrajectoryMaker(initialPose, endPose, points);
    }

    public static TrajectoryMaker createSlolom()
    {
        return createTrajectory(slalom, 1);
    }     

    // Need better documentation here.  What are these doing?  Are the units in meters?
    public static TrajectoryMaker createfrontScorePath()
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(3, 0, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createTrenchToTargetDiagonal()
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.25, -0.25, new Rotation2d(0)), true);//8,-1.6
    }
    public static TrajectoryMaker createTargetToFrontOfTrench()
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(-0.25, 0.25, new Rotation2d(0)), true);// -4.2,1.6
    }
    public static TrajectoryMaker createTrenchForward() //Assuming facing forward
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.25, 0, new Rotation2d(0)), true);//2
    }

    public static TrajectoryMaker createForwardPath() //For Testing Purposes
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(.75, 0, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createZagPath() //For Testing Purposes
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 0.5, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createForwardPath2() //For Testing Purposes
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2, 0, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createToPortPath() //For Testing Purposes
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0.25, 0, new Rotation2d(0)), true);//3
    }

    public static TrajectoryMaker createPortToFrontofTrench()
    {
        ArrayList<Translation2d> points = new ArrayList<Translation2d>();
        points.add(new Translation2d(-1.5, 2.3));
        points.add(new Translation2d(-3, 2.3));
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(-5.3, 2.3, new Rotation2d(180)), points );
    }

    public static TrajectoryMaker createTestMultiPath()
    {
        ArrayList<Translation2d> points = new ArrayList<Translation2d>();
        points.add(new Translation2d(1,-0.5));
        points.add(new Translation2d(1,0.5));
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2, 0, new Rotation2d(Math.PI)), points );
    }
    public static TrajectoryMaker createMoveDownTrench()
    {
        return new TrajectoryMaker(new Pose2d(0,0,new Rotation2d(0)), new Pose2d(3, 0, new Rotation2d(0)), true);
    }

    public static TrajectoryMaker createMoveToPort()
    {
        ArrayList<Translation2d> points = new ArrayList<Translation2d>();
        points.add(new Translation2d(1.524, 2.286));
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(3.048, 4.572, new Rotation2d(0)), points );
    }


    public static TrajectoryMaker createAutonomousPath1() // Init Line (Start on Left) to Port Test
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2, 0, new Rotation2d(0)), true);
    }

    public static TrajectoryMaker createAutonomousPath2() //Test 2 Electric Bugaloo
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, -1, new Rotation2d(0)), true);
    }

    public static TrajectoryMaker createSidePath() //Test Path
    {
        //return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2.77, 0.2, new Rotation2d(0)), true);
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, -2, new Rotation2d(0)), true);
    }

    public static TrajectoryMaker createDiagonalPath() //Test Path
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(1, 1, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createBackwardPath() //Test Path
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2.77, 0, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createForwardPath3() //Test Path
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(2, 0, new Rotation2d(0)), true);
    }
    public static TrajectoryMaker createForwardPath4() //Test Path
    {
        return new TrajectoryMaker(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(-4.75, 0, new Rotation2d(0)), true);
    }
}
