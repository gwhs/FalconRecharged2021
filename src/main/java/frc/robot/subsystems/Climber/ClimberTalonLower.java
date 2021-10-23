/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DaphneTwoConstants;

public class ClimberTalonLower extends SubsystemBase {
  /**
   * Creates a new ClimberTalon. 48 to 1 Gear box ratio maybe
   */
  private TalonFX lowerArm;
  private DigitalInput rightLimit;

  public ClimberTalonLower() {
    lowerArm = new TalonFX(Constants.CLIMBER2_TALON); //right lower
    rightLimit = new DigitalInput(2);
    lowerArm.config_kP(0, .08, 0);
    lowerArm.config_kI(0, 0.00, 0);
    lowerArm.config_kD(0, 0, 0);
    lowerArm.config_kF(0, 0, 0);
    
    lowerArm.setInverted(true);

    // lowerArm.configForwardSoftLimitThreshold(-8469); //8469
    // lowerArm.configReverseSoftLimitThreshold(-245639); //245639

    //lowerArm.configForwardSoftLimitEnable(false);
    //lowerArm.configReverseSoftLimitEnable(false);

    lowerArm.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Right Limit", rightLimit.get());
    SmartDashboard.putNumber("Right Encoder Pos", lowerArm.getSelectedSensorPosition());
  }

  public void moveLowerArm(double speed){
    lowerArm.set(ControlMode.PercentOutput, speed);
  }

  public TalonFX getLowerArm(){
    return lowerArm;
  }

public BaseMotorController getUpperArm() {
	return null;
}

public double inchesToTicks(double inches) {
  return inches * DaphneTwoConstants.CLIMBERTALONS_ONE_INCH_IN_TICKS;
}
 
}
