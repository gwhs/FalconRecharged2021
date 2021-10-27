package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.DaphneTwoConstants;
import frc.robot.subsystems.Climber.ClimberTalonLower;
import frc.robot.subsystems.Climber.ClimberTalonUpper;

public class ClimbSkewLeft extends SequentialCommandGroup {
    public ClimbSkewLeft(ClimberTalonUpper climberTalonUpper, ClimberTalonLower climberTalonLower, double startingTicksUpper, double startingTicksLower) {
//upper arm closer to center
        super(new MoveUpperArmByInches(climberTalonUpper, 4, startingTicksUpper),
              new MoveLowerArmByInches(climberTalonLower, 18.75, startingTicksLower),
              new MoveLowerArmByInches(climberTalonLower, climberTalonUpper.getUpperArm().getSelectedSensorPosition() * DaphneTwoConstants.CLIMBERTALONS_ONE_INCH_IN_TICKS + 4, startingTicksLower),
              new ParallelCommandGroup(new MoveLowerArmByInches(climberTalonLower, 6, startingTicksLower),
                                       new MoveUpperArmByInches(climberTalonUpper, 2, startingTicksUpper)));

    }
}

