package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorTalon;

/**
 * Pauses for a configurable amount of time to wait for conveyor to see the power cell 
 */
public class WaitForConveyor extends CommandBase {

    ConveyorTalon conveyorTalon;
    private long startTime;
    private final double maxWaitTime = 100;

    public WaitForConveyor(ConveyorTalon conveyor) {
        super();
        conveyorTalon = conveyor;
        addRequirements(conveyorTalon);
    }

    //Set the startTime to remember when the command began
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    // checks if power cell is in conveyor to finish or .1 second has gone by
    public boolean isFinished() {
        long currentTime = System.currentTimeMillis();
        if (conveyorTalon.getHasSeen() || (currentTime - startTime) > maxWaitTime) {
            return true;
        }
        return false;
    }

}
