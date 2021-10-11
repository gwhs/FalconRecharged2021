/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberTalon;
import frc.robot.subsystems.Climber.ClimberTalonLower;
import frc.robot.subsystems.Climber.ClimberTalonUpper;
import frc.robot.utility.MathUtils;

public class ClimberArmSpeed extends CommandBase {
  /**
   * Creates a new ClimberArmSpeed.
   */
  private ClimberTalonUpper mClimberTalonUpper;
  private ClimberTalonLower mClimberTalonLower;
  private XboxController operatorController;

  public ClimberArmSpeed(ClimberTalonUpper climberTalonUpper, ClimberTalonLower climberTalonLower, XboxController operatorController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climberTalonUpper, climberTalonLower);
    this.mClimberTalonUpper = climberTalonUpper;
    this.mClimberTalonLower = climberTalonLower;
    this.operatorController = operatorController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double speed1 = MathUtils.deadband(operatorController.getRawAxis(1), 0.2 );
    //double speed2 = MathUtils.deadband(operatorController.getRawAxis(5), 0.2 );
    // if(operatorController.get)
    mClimberTalonUpper.moveUpperArm(0.2);
    mClimberTalonLower.moveLowerArm(0.2);
    System.out.println("climber arm speed");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
