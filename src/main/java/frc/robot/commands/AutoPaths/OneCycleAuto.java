package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;
import frc.robot.utility.TrajectoryMaker;
import frc.robot.utility.TrajectoryMaker;

public class OneCycleAuto extends SequentialCommandGroup {

    Shooter shooter;
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    public OneCycleAuto(SwerveDriveSubsystem swerveDriveSubsystem, Shooter shooter) {
        
        super();
        TrajectoryMaker createDriveTenFeet
        
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
    private Object createTrajectory(double[][] driveTenFeet2, double d, int i, int j, boolean b) {
		return null;
	}

	            x = MathUtils.inchesToMeters(x);  // convert to metric
                y = MathUtils.inchesToMeters(y);
                x = x * scale;  // apply extra scale
                y = y * scale;
                points.add(new Translation2d(x, y));
            }
            return points;
        }
    
        public static TrajectoryMaker createTrajectory(double [][] inputPoints, double scale, double startOrientation, double endOrientation, boolean isReversed) // for bounce
        {
            ArrayList<Translation2d> points = translateAndScale(inputPoints, scale);  // make .2 for Hajel's garage.  Turns the 30 foot field to 6 feet
            Pose2d initialPose = new Pose2d(0, 0, new Rotation2d(startOrientation));
            Translation2d lastPoint = points.remove(points.size()-1);  // remove last point in array
            Pose2d endPose = new Pose2d(lastPoint.getX(), lastPoint.getY(), new Rotation2d(endOrientation));
    
            return new TrajectoryMaker(initialPose, endPose, points, isReversed);
        }

        public static TrajectoryMaker createDriveTenFeet() {
            return createTrajectory(driveTenFeet, 0.827, 0, 0, false);
        }   
        TrajectoryMaker firstPath = TrajectoryHelper.createDriveTenFeet();
        Command autoCommandFirstPath = new Autonomous(swerveDriveSubsystem, driveTenFeet.getTrajectory(), driveTenFeet.getAngle(), true);

        addCommands(
            new Command(autoCommandFirstPath)


        );

    }
private Object createTrajectory(double[][] dri   null;     
    private static double[][] driveTenFeet = {
        {0, 60},
        {120, 60}
    };
   
    
}
