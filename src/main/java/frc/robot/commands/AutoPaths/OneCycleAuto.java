/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DaphneTwoConstants;
import frc.robot.TrajectoryHelper;
import frc.robot.commands.intake.IntakeSpeed;
import frc.robot.commands.shooter.AutoShoot;
import frc.robot.commands.swervedrive.AutoRotate;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.commands.swervedrive.GoToDistance;
import frc.robot.subsystems.ConveyorTalon;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utility.TrajectoryMaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class OneCycleAuto extends SequentialCommandGroup {
  /**
   * Creates a new OneCycleAuto.
   */
  public OneCycleAuto(SwerveDriveSubsystem swerveDriveSubsystem, ConveyorTalon conveyorTalon, Intake intake, Shooter shooter, int inputRPM) {
    // Start at Initiation Line
    // This is all theoretical code with no actual field measurements. 
    // MAKE SURE TO MEASURE AND SWAP VALUES BEFORE TESTING - Kyle
    /**
     * 
     * The Robot starts at the initiation line.
     * 
     * 
     * 
     */


    //TrajectoryMaker traj = TrajectoryHelper.createToPortPath();
    addCommands(
      new GoToDistance(86, swerveDriveSubsystem).withTimeout(5),
      new AutoShoot(conveyorTalon, shooter, true, inputRPM, DaphneTwoConstants.CONVEYOR_UNLOADS_SPEED)
    );
  }
}
