/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryHelper;
import frc.robot.utility.TrajectoryMaker;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

/**
 * Creates a trajectory that runs 
 * exit early if galactic search is finished 
 */
public class Finish_Auton extends SequentialCommandGroup {

  public GalacticSearch search;

  public Finish_Auton(SwerveDriveSubsystem swerveDriveSubsystem, double[][] inputPoints, GalacticSearch galacticSearch, boolean firstPath, double startOrientation, double endOrientation) { 
   
    super();
    TrajectoryMaker trajectory = TrajectoryHelper.createTrajectory(inputPoints, TrajectoryHelper.GLOBAL_SCALE, startOrientation, endOrientation, false);
    addCommands(
      new Autonomous(swerveDriveSubsystem, trajectory.getTrajectory(), trajectory.getAngle(), firstPath).withTimeout(60)
    );
    this.search = galacticSearch; 
  }

  //checks for galacticSearchDone 
  @Override
  public boolean isFinished() {
    return super.isFinished() || search.getDone();
  }
  
}