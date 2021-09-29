package frc.robot.commands.AutoPaths;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.swervedrive.Autonomous;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;

public class OneCycleAuto extends SequentialCommandGroup {

    Shooter shooter;
    
    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    public OneCycleAuto(SwerveDriveSubsystem swerveDriveSubsystem, Shooter shooter) {
        
        this.shooter = shooter;
        super();
        double[][] driveTenFeet = {
            {0, 60},
            {120, 60}
        };
        addCommands(
            new Finish_Auton(swerveDriveSubsystem, driveTenFeet, true, 0, 0),


        );

    }
   
    
}
