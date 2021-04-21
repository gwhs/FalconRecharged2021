/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive.SwerveDriveSubsystem;
import frc.robot.utility.MathUtils;

public class HolonomicDriveCommand extends CommandBase {
 
  	private final SwerveDriveSubsystem mDrivetrain;
  	private final XboxController mXboxController;

	/**
      * Creates a new HolonomicDriveCommand.
      */
	public HolonomicDriveCommand(SwerveDriveSubsystem drivetrain, XboxController mXboxController) {
		mDrivetrain = drivetrain;
		addRequirements(drivetrain);
		this.mXboxController = mXboxController;
	}

	@Override
	public void execute() {
		
		//sets reobot oriented when auto
		if(mDrivetrain.getIsAuto()) {
			mDrivetrain.setFieldOriented(false);
		}
		
		//raw joystick values
		double forward = mXboxController.getY(Hand.kLeft); //real: pos
		double rotation = mXboxController.getTriggerAxis(Hand.kLeft) 
			- mXboxController.getTriggerAxis(Hand.kRight); //trigger values are between 0 and 1, left is -1 and right is +1
		double strafe = mXboxController.getX(Hand.kLeft); //real: pos
		
		//filters raw joystick values to avoid joystick drift
		double filteredForward = MathUtils.deadband(forward, 0.175, mDrivetrain.isFieldOriented());
		double filteredStrafe = MathUtils.deadband(strafe, 0.175, mDrivetrain.isFieldOriented());
		double filteredRotation = MathUtils.deadband(rotation, 0.1, mDrivetrain.isFieldOriented());

		mDrivetrain.swapPIDSlot(0); //switches to different set of PID values 
		mDrivetrain.holonomicDrive(filteredForward, -filteredStrafe, filteredRotation); //sets filtered joystick values to holomic drive
	}

	@Override 
	public void end(boolean interrupted) {
		mDrivetrain.stopDriveMotors();
	}
}
