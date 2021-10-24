package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberTalonLower;
import frc.robot.subsystems.Climber.ClimberTalonUpper;

public class ClimbSkewRight extends SequentialCommandGroup {
    public ClimbSkewRight(ClimberTalonUpper climberTalonUpper, ClimberTalonLower climberTalonLower, double startingTicksUpper, double startingTicksLower) {

        super(new SemiAutoPullUp(climberTalonUpper, climberTalonLower, startingTicksUpper, startingTicksLower),
              new MoveUpperArmByInches(climberTalonUpper, 18, startingTicksLower),
              new MoveUpperArmByInches(climberTalonUpper, 4.625, startingTicksLower),
              new ParallelCommandGroup(new MoveLowerArmByInches(climberTalonLower, 2, startingTicksLower),
                                       new MoveUpperArmByInches(climberTalonUpper, 2, startingTicksUpper)));

    }
}

