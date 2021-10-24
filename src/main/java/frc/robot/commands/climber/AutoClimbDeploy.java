/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DaphneTwoConstants;
import frc.robot.subsystems.Climber.ClimberTalonLower;
import frc.robot.subsystems.Climber.ClimberTalonUpper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoClimbDeploy extends SequentialCommandGroup {
  /**
   * Deploys Climber Arms
   */
  

  public AutoClimbDeploy(ClimberTalonUpper climberTalonUpper, ClimberTalonLower climberTalonLower, double startingTicksUpper, double startingTicksLower) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    super(new MoveUpperArmByInches(climberTalonUpper, 16, startingTicksUpper).withTimeout(2),
          new ParallelCommandGroup(new MoveLowerArmByInches(climberTalonLower, 18.75, startingTicksLower),
                                   new MoveUpperArmByInches(climberTalonUpper, 18, startingTicksUpper))
    );

  }
}
