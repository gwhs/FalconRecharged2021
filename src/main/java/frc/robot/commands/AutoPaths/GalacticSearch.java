// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.TrajectoryHelper;
import frc.robot.commands.conveyor.SenseNewPowerCell;
import frc.robot.commands.intake.IntakeSpeed;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.subsystems.ConveyorTalon;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;
import frc.robot.utility.TrajectoryMaker;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
  /** Creates a new SensorTest. */
  public static final double intakeSpeed = -.5;
  public boolean galacticSearchDone = false;

  public void setDone()
  {
    galacticSearchDone = true;
  }

  public boolean getDone()
  {
    return galacticSearchDone;
  }
  /**
   * creates a conditional command that runs the hasSeenTrajectory if a powercell is in the conveyor, 
   * otherwise runs notSeenTrajectory
   * if run hasSeenTrajectory setDone following commands don't run
   * @param swerveDriveSubsystem 
   * @param intake
   * @param conveyor
   * @param hasSeenTrajectory
   * @param notSeenTrajectory
   * @return
   */
  public Command conditional(SwerveDriveSubsystem swerveDriveSubsystem, Intake intake, ConveyorTalon conveyor, double[][] hasSeenTrajectory, double[][] notSeenTrajectory)
  {
    return new ConditionalCommand(
      new Finish_Auton(swerveDriveSubsystem, hasSeenTrajectory, this)
        .raceWith(new IntakeSpeed(intake, intakeSpeed))
          .raceWith(new SenseNewPowerCell(conveyor)).andThen(()->setDone()), 
      new Finish_Auton(swerveDriveSubsystem, notSeenTrajectory, this)
        .raceWith(new IntakeSpeed(intake, intakeSpeed))
          .raceWith(new SenseNewPowerCell(conveyor)),
      conveyor::getHasSeen
    );
  }

  @Override
  public void initialize() {
    super.initialize();
    galacticSearchDone = false;
  }
  
  public GalacticSearch(SwerveDriveSubsystem swerveDriveSubsystem, Intake intake, ConveyorTalon conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Start_to_B3 --> if (found B3) B3_to_Finish 
    // else B3_to_C3 --> if (found C3) C3_to_Finish
    // else C3_to_D6 --> if (found D6) D6_to_Finish_A
    // else D6_to_Finish_B
    
    super();
    addCommands(
    new InstantCommand(intake::lowerIntake, intake),
    new Finish_Auton(swerveDriveSubsystem, Start_to_B3, this).raceWith(new IntakeSpeed(intake, intakeSpeed)).raceWith(new SenseNewPowerCell(conveyor)), 
    conditional(swerveDriveSubsystem, intake, conveyor, B3_to_Finish, B3_to_C3),
    conditional(swerveDriveSubsystem, intake, conveyor, C3_to_Finish, C3_to_D6),
    //conditional(swerveDriveSubsystem, intake, conveyor, D6_to_Finish_A, D6_to_Finish_B),
    new InstantCommand(swerveDriveSubsystem::stopDriveMotors, swerveDriveSubsystem),
    new IntakeSpeed(intake, 0)
    );
  } 
    
  /*
  public GalacticSearch(SwerveDriveSubsystem swerveDriveSubsystem, Intake intake, ConveyorTalon conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // Start_to_B3 --> if (found B3) B3_to_Finish 
    // else B3_to_C3 --> if (found C3) C3_to_Finish
    // else C3_to_D6 --> if (found D6) D6_to_Finish_A
    // else D6_to_Finish_B
    
    super();
    addCommands(
    new InstantCommand(intake::lowerIntake, intake),
    new Finish_Auton(swerveDriveSubsystem, driveForward, this).raceWith(new IntakeSpeed(intake, intakeSpeed)).raceWith(new SenseNewPowerCell(conveyor)), 
    conditional(swerveDriveSubsystem, intake, conveyor, driveRight, driveLeft),
    new InstantCommand(swerveDriveSubsystem::stopDriveMotors, swerveDriveSubsystem),
    new IntakeSpeed(intake, 0)
    );
  } */

    public static double[][] driveForward = {
      {30,120},
      {90,120},
    };

    public static double[][] driveRight = {
      {90,120},
      {90,150},
    };

    public static double[][] driveLeft = {
      {90,120},
      {90,90},
    };

    // measuring from top down (0 is the top, 180 bottom)
    public static double[] A6 = {180, 30};
    public static double[] B1 = {15, 60}; 
    public static double[] B3 = {90, 60}; 
    public static double[] B7 = {210, 60};
    public static double[] B8 = {240, 60};
    public static double[] C3 = {90, 90};
    public static double[] C9 = {270, 90};
    public static double[] D5 = {150, 120};
    public static double[] D6 = {180, 120};
    public static double[] D10 = {300, 120};
    public static double[] E6 = {180, 150};
    public static double[] B3_END = {300, 60};
    public static double[] C3_END = {300, 30};
    public static double[] D6_END_A = {300, 120};
    public static double[] D6_END_B = {300, 90};

    public static double[][] Start_to_B3= {
        B1,
        B3, 
    };

    public static double[][] B3_to_C3=  {
        B3,
        C3,
    };

    //345 finish?
    public static double[][] B3_to_Finish = {
      B3,
      D5,
      B7,
      B3_END,
    };

    public static double[][] C3_to_D6 = {
      C3,
      D6,
    };

    public static double[][] C3_to_Finish = {
      C3,
      D5,
      A6,
      C3_END,
    };

    public static double[][] D6_to_Finish_A = {
      D6,
      B8,
      D10,
      D6_END_A,
    };

    public static double[][] D6_to_Finish_B = {
      D6,
      E6,
      B7,
      C9,
      D6_END_B,
    };
}

