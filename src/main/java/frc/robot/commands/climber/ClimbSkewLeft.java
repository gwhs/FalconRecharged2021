package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberTalonLower;
import frc.robot.subsystems.Climber.ClimberTalonUpper;

public class ClimbSkewLeft extends SequentialCommandGroup {
    public ClimbSkewLeft(ClimberTalonUpper climberTalonUpper, ClimberTalonLower climberTalonLower, double startingTicksUpper, double startingTicksLower) {

        super(new MoveLowerArmByInches(climberTalonLower, 18.75, startingTicksLower),
              new MoveUpperArmByInches(climberTalonUpper, 18, startingTicksUpper),
              new MoveLowerArmByInches(climberTalonLower, 14.125, startingTicksLower),
              new ParallelCommandGroup(new MoveLowerArmByInches(climberTalonLower, 6.625, startingTicksLower),
                                       new MoveUpperArmByInches(climberTalonUpper, 2, startingTicksUpper)));

    }
}

