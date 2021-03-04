/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryHelper;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.commands.swervedrive.TurnToAngleProfiled;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class D6_Finish_A_Auton extends SequentialCommandGroup {
  /**
   * Creates a new AutoPath1.
   * 
   * 
   * This is just an Auto used for general testing. 
   */
  public D6_Finish_A_Auton(SwerveDriveSubsystem swerveDriveSubsystem) {  // test forward path
   
    super(
      //new Autonomous(swerveDriveSubsystem, TrajectoryHelper.createTestMultiPath().getTrajectory(), TrajectoryHelper.createTestMultiPath().getAngle())
      
      
      new Autonomous(swerveDriveSubsystem, TrajectoryHelper.D6_to_Finish_A().getTrajectory(), TrajectoryHelper.D6_to_Finish_A().getAngle()).withTimeout(1)
    );
  }
  
}