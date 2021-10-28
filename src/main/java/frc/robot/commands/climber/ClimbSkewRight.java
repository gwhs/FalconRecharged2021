package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DaphneTwoConstants;
import frc.robot.subsystems.Climber.ClimberTalonLower;
import frc.robot.subsystems.Climber.ClimberTalonUpper;

public class ClimbSkewRight extends SequentialCommandGroup {
    public ClimbSkewRight(ClimberTalonUpper climberTalonUpper, ClimberTalonLower climberTalonLower, double startingTicksUpper, double startingTicksLower) {
        //lower arm toward center
        super(new MoveLowerArmByInches(climberTalonLower, 4, startingTicksLower),
              new MoveUpperArmByInches(climberTalonUpper, 18, startingTicksLower),
              new MoveUpperArmByInches(climberTalonUpper, climberTalonLower.getLowerArm().getSelectedSensorPosition() * DaphneTwoConstants.CLIMBERTALONS_ONE_INCH_IN_TICKS + 4, startingTicksLower),
              new ParallelCommandGroup(new MoveLowerArmByInches(climberTalonLower, 2, startingTicksLower),
                                       new MoveUpperArmByInches(climberTalonUpper, 6, startingTicksUpper)));

    }
}

