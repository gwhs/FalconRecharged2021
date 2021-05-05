/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.conveyor;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ConveyorTalon;

  /**
   * Creates a new SenseCell.
   * Sets the seen in conveyor to false
   * Once it sees powercell, set seen to true, runs the conveyor until the ball is not blocking the time of flights sensor
   */
public class SenseCell extends CommandBase {
  private boolean seen;

  private ConveyorTalon conveyorTalon;

  /**
   * 
   * @param conveyorTalon
   */
  public SenseCell(ConveyorTalon conveyorTalon) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(conveyorTalon);
    this.conveyorTalon = conveyorTalon;
    seen = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    seen = false;
    conveyorTalon.setHasSeen(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    seen = conveyorTalon.getStatus() && !conveyorTalon.isIgnored();
    if(seen)
    {
      conveyorTalon.setHasSeen(true);
      conveyorTalon.setConveyorSpeed(-.5);
    }
    else
    {
      conveyorTalon.setConveyorSpeed(0);
    }
  }

}
