package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.utility.MathUtils;
import frc.robot.utility.TrajectoryMaker;
//import jdk.nashorn.internal.objects.Global;

import java.util.ArrayList;

@SuppressWarnings("unused")
public class TrajectoryHelper {

    public static double[][] test4Meters= {
        {0,0},
        {157.48,0}, // roughly equal to 4 meters
    };

    public static double[][] test3Meters= {
        {0,0},
        {117.7721,0}, // roughly equal to 3 meters
    };

    public static double[][] test2Meters= {
        {0,0},
        {78.7402,0}, // roughly equal to 2 meters
    };

    public static double[][] test1Meter= {
        {0,0},
        {39.3701,0}, // roughly equal to 2 meters
    };

  //  public static double[][] testStep= {
  //      {0,0},
  //      // {60+6.68,0},
       // {90+7,0},
       //  {120+6.68 ,0},
       // {180+7,0}, should be +6
       // {240+7,0},
  //      {300+6.68, 0},
  //       {30+6.68, 0}
  //  };
 
//   public static double[][] testStep= {
//    { 30, 30 }, // Starting point
//    { 80+6.68, 35+6.68 }, 
//    { 125+6.68, 90+6.68 }, 
//    { 180+6.68, 110+6.68 }, 
//    { 250+6.68, 80+6.68 }, 
//    { 285+6.68, 30+6.68 }, 
//    { 340+6.68, 45+6.68 }, 
//    { 320+6.68, 100+6.68 }, 
//    { 275+6.68, 80+6.68 },
//    { 250+6.68, 35+6.68 }, 
//    { 180+6.68, 25+6.68 }, 
//    { 120+6.68, 40+6.68 }, 
//    { 80+6.68, 70+6.68 }, 
//    { 30+6.68, 100+6.68 }
//    };

 //   public static double[][] testStep= {
 //        //resulted in shift right
 //       {30,30},
 //       {200,30},
 //       {200,60},
 //       {30,60},
 //   };

//    public static double[][] testStep= {
//        {30,120},
//        {120,135},
//        {240,120},
//        {240,105},
//        {240,90},
//        {240,75},
//        {240,60},
//        {240,45},
//        {240,30},
//        {120,15},
//        {30,30},
//    };

//    public static double[][] testStep= {
// trying teleop curve
//        {60,90},
//        {100,20},
//        {180,90},
//        {200,90},
//        {240,90},
//        {270,30},
//        {300,60},
//        {330,120},
//        {300,150},
//        {240,150},
//        {180,150},
//        {120,90},
//        {180,150},
//        {120,90},
//        {90,90},
//        {60,150},
// 
//    };

 /*    public static double[][] testStep= {
   // trying slalom 2021.02.13
    {30,150},
    {60,165},
    {90,120}, //point D3
    {120,75},
    {150,75},
    {180,75},
    {210,75},
    {250,90},
    {270,120}, // D9
    {285,160},
    {300,160},
    {330,150},
    {350,120}, //D11
    //turn back
    {300,75}, 
    {280,100}, 
    {270,120}, // D9
    {265,165},
    {240,160},
    {210,160},
    {180,160},
    {150,160},
    {120,160},
    {105,150},
    {90,120}, //point D3
    {75,75},
    {60,75},
    //finish
    {30,90},     
    };
        
 */

public static double[][] testStep= {
    // Slalom 2021.02.15 001
    {30,150},
    {60,160},
    {90,120},
    {120,85},
    {150,80},
    {180,80},
    {225,80},
    {255,90},
    {270,120},
    {285,155},
    {310,160},
    {330,150},
    {340,120},
    {330,85},
    {300,75},
    {280,95},
    {270,120},
    {265,140},
    {240,155},
    {210,160},
    {180,160},
    {150,160},
    {120,155},
    {105,145},
    {90,120},
    {80,95},
    {60,80},
    {30,90},
    };

 /*
public static double[][] testStep= {
    // Barrel 2021.02.15 001
{30,100},
{60,90},
{90,85},
{120,90},
{150,100},
{180,120},
{180,140},
{150,160},
{120,140},
{120,120},
{140,100},
{160,90},
{180,95},
{200,100},
{210,100},
{225,100},
{255,95},
{270,90},
{285,80},
{300,60},
{280,40},
{250,30},
{200,40},
{200,60},
{210,85},
{230,120},
{250,140},
{300,160},
{330,150},
{350,120},
{330,100},
{300,90},
{280,85},
{270,85},
{265,85},
{240,90},
{210,90},
{180,85},
{150,80},
{120,85},
{105,85},
{90,90},
{80,90},
{60,90},
{30,90},
};
*/
    public static double[][] test2MetersAndBack= {
        {0,0},
        {78.7402,0}, // roughly equal to 2 meters
        {0,0}
    };

    public static double[][] test2MetersTriangle= {
        {0,0},
        {78.7402,0}, // roughly equal to 2 meters
        {78.7402,78.7402}, // roughly equal to 2 meters
        {0,0}
    };


    public static double[][] Start_to_B3= {
        {30,120},
        {90,120},
    };

    public static double[][] B3_to_C3=  {
        {90,120},
        {90,90},
    };
    
    public static double[][] B3_to_Finish = {
        {90,120},
        {150,60},
        {180,150},
        {345,150},
    };
        

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

     public static double[][] bounce = {

        { 30, 90},  // starting point
        { 75, 90},
        { 90, 150}, // bounce 1
        { 105, 85},
        { 135, 30},
        { 165, 30},  
        { 180, 45},
        { 178, 90},
        { 180, 150}, // bounce 2
        { 182, 90},
        { 195, 45},
        { 210, 30},
        { 255, 35},
        { 270, 110},
        { 270, 150}, // bounce 3
        { 280, 115},
        { 328, 90}   // end
     };

     public static double[][] barrel = {
        { 30, 96},  // starting point
        { 150, 96}, // circle around 150, 60
        { 168, 60},
        { 150, 36}, 
        { 132, 60}, // finish circle around 150, 60
        { 150, 96}, // circle arouund 240, 120
        { 240, 96}, 
        { 264, 120},
        { 240, 144},
        { 216, 120}, // complete circle around 240, 120
        { 276, 60}, // start circle around 300, 60
        { 312, 30},
        { 324, 84}, // finish circle
        { 30, 96}  // head back to start
    };

//    public static double GLOBAL_SCALE = 0.827;// divide size by 5 to fit into Hajel's garage
      public static double GLOBAL_SCALE = 0.806;// divide size by 5 to fit into Hajel's garage

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

    public static TrajectoryMaker createTest4Meters() // test path going only 4 meters forward
    {
        return createTrajectory(test4Meters, GLOBAL_SCALE);
    }   

    public static TrajectoryMaker createTest3Meters() // test path going 2 meters forward
    {
        return createTrajectory(test3Meters, GLOBAL_SCALE);
    }

    public static TrajectoryMaker createTest2Meters() // test path going 2 meters forward
    {
        return createTrajectory(test2Meters, GLOBAL_SCALE);
    }

    public static TrajectoryMaker createTestStep() // test path going forward
    {
        return createTrajectory(testStep, GLOBAL_SCALE);
    }

    public static TrajectoryMaker Start_to_B3()
    {
        return createTrajectory(Start_to_B3, GLOBAL_SCALE);
    }

    public static TrajectoryMaker B3_to_Finish()
    {
        return createTrajectory(B3_to_Finish, GLOBAL_SCALE);
    }

    public static TrajectoryMaker B3_to_C3()
    {
        return createTrajectory(B3_to_C3, GLOBAL_SCALE);
    }

    public static TrajectoryMaker createSlolom()
    {
        return createTrajectory(slalom, GLOBAL_SCALE);
    }     

    public static TrajectoryMaker createBounce()
    {
        return createTrajectory(bounce, GLOBAL_SCALE);
    } 

    public static TrajectoryMaker createBarrel()
    {
        return createTrajectory(barrel, GLOBAL_SCALE);
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
