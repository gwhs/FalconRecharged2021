package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

public class OneCycleAuto extends SequentialCommandGroup {

    Shooter shooter;
    
    @Override
    public void end(boolean interrupted) {

    }

    /**
   * creates a conditional command that runs the hasSeenTrajectory if a powercell is in the conveyor,
   * otherwise runs notSeenTrajectory
   * if run hasSeenTrajectory setDone following commands don't run
   * @param swerveDriveSubsystem
   * @return
   */
    public Command conditional(SwerveDriveSubsystem swerveDriveSubsystem, Intake intake)
    {
        return new ConditionalCommand();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    public OneCycleAuto(SwerveDriveSubsystem swerveDriveSubsystem, Shooter shooter) {
        
        this.shooter = shooter;
        super();
        addCommands(
            new Finish_Auton(swerveDriveSubsystem, /*path*/, true, 0, 0)

        );
    }
   
    
}
