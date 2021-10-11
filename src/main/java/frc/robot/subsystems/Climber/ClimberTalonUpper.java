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

public class ClimberTalonUpper extends SubsystemBase {
  /**
   * Creates a new ClimberTalon. 48 to 1 Gear box ratio maybe
   */
  private TalonFX upperArm;
  private DoubleSolenoid climberGearLock;
  private DigitalInput leftLimit;

  public ClimberTalonUpper() {
    upperArm = new TalonFX(Constants.CLIMBER1_TALON); //left is upper
    climberGearLock = new DoubleSolenoid(Constants.CLIMBERFORWARD_SOLENOID,Constants.CLIMBERREVERSE_SOLENOID);
    leftLimit = new DigitalInput(1);
    upperArm.config_kP(0, .08, 0);
    upperArm.config_kI(0, 0.000, 0);
    upperArm.config_kD(0, 0, 0);
    upperArm.config_kF(0, 0, 0);

    // upperArm.configForwardSoftLimitThreshold(243438);
    // upperArm.configReverseSoftLimitThreshold(10263);

    // upperArm.configForwardSoftLimitEnable(true);
    // upperArm.configReverseSoftLimitEnable(true);

    upperArm.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Left Limit", leftLimit.get());
    SmartDashboard.putNumber("Left Encoder Pos", upperArm.getSelectedSensorPosition());
  }


  public void moveUpperArm(double speed){
    upperArm.set(ControlMode.PercentOutput, speed);
  }

  public TalonFX getUpperArm(){
    return upperArm;
  }

  public double inchesToTicks(double inches) {
    return inches * DaphneTwoConstants.CLIMBERTALONS_ONE_INCH_IN_TICKS;
  }

 
}
