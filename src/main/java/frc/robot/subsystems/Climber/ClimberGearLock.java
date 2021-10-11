/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DaphneTwoConstants;

public class ClimberGearLock extends SubsystemBase {
  /**
   * Creates a new ClimberTalon. 48 to 1 Gear box ratio maybe
   */
  private DoubleSolenoid climberGearLock;

  public ClimberGearLock() {
    climberGearLock = new DoubleSolenoid(Constants.CLIMBERFORWARD_SOLENOID,Constants.CLIMBERREVERSE_SOLENOID);

    // upperArm.configForwardSoftLimitThreshold(243438);
    // upperArm.configReverseSoftLimitThreshold(10263);

    // upperArm.configForwardSoftLimitEnable(true);
    // upperArm.configReverseSoftLimitEnable(true);

  }

  public void toggleClimberGearLock() {
    if(climberGearLock.get() == Value.kReverse)
    {
      climberGearLock.set(Value.kForward);
    }
    else {
      climberGearLock.set(Value.kReverse);
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    }

 
}
