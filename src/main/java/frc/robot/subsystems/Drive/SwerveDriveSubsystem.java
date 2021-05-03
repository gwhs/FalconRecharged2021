package frc.robot.subsystems.Drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

//ANGLE MOTOR TAKING LONG ROUTE ISSUE 
public class SwerveDriveSubsystem extends HolonomicDrivetrain { // + is clockwise - is counter clockwise test commit 2 electric bugaloo
	
	//used for calculating angle of wheels when rotating robot, describe kinematic parameters
	private static final double WHEELBASE = 22.5; //distance between front and back wheels
	private static final double TRACKWIDTH = 22.5; //distance between left and right wheels

	private boolean isAuto;
	
	/*
	 * 0 is Front Right
	 * 1 is Front Left
	 * 2 is Back Left
	 * 3 is Back Right
	 */
	private final SwerveDriveModule[] mSwerveModules = new SwerveDriveModule[4];

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200); //initialize NavX to read orientation/bearing of robot

	public SwerveDriveSubsystem(SwerveDriveModule m0, SwerveDriveModule m1, SwerveDriveModule m2, SwerveDriveModule m3) { 
		mSwerveModules[0] = m0;
		mSwerveModules[1] = m1;
		mSwerveModules[2] = m2;
		mSwerveModules[3] = m3;
		zeroGyro();

		// mSwerveModules[0].getDriveMotor().setInverted(InvertType.InvertMotorOutput); //real: false
		// mSwerveModules[2].getDriveMotor().setInverted(true); //
		// mSwerveModules[1].getDriveMotor().setInverted(false); //real: true
		// mSwerveModules[2].getDriveMotor().setInverted(false); //real: false
		//mSwerveModules[3].getDriveMotor().setInverted(TalonFXInvertType.CounterClockwise); //real: false
		
		//might have to do w/ wheel spinning the wrong way
		 mSwerveModules[0].getAngleMotor().setInverted(true); //real: true
		 mSwerveModules[1].getAngleMotor().setInverted(true); //real: true 
		 mSwerveModules[2].getAngleMotor().setInverted(true); //real: true
		 mSwerveModules[3].getAngleMotor().setInverted(true); //real: true

		//why only reseting for module 0 ??
		 mSwerveModules[0].resetEncoder();
		for(int i = 0; i < 4; i++) {
			mSwerveModules[i].getDriveMotor().setNeutralMode(NeutralMode.Brake);
		}

		isAuto = false;
	}

	public AHRS getNavX() { //AHRS: attitude heading reference system (orientation)
		return mNavX;
	}

	public double getGyroAngle() { //relative(to field) orientation in degrees  
		return (mNavX.getAngle() - getAdjustmentAngle());
	}

	public double getGyroAngle2() { //might get rid of, change where used
		return (-getGyroAngle());
	}

	public double getGyroRate() { //angular velocity of robot
		return mNavX.getRate();
	}

	public double getYaw() { //rotation around vertical(z) axis <--only one used
		return mNavX.getAngle();
	}

	public double getPitch() { //rotation around perpendicular(y) axis
		return mNavX.getPitch();
	}

	public double getRoll() { //rotation around forward(x) axis
		return  mNavX.getRoll();
	}

	public SwerveDriveModule getSwerveModule(int i) {
		return mSwerveModules[i];
	}


	@Override
	public void holonomicDrive(double forward, double strafe, double rotation) { //parameters: joystick values 0-1
		forward *= getSpeedMultiplier();
		strafe *= getSpeedMultiplier(); 
		
		//forward in fieldOriented is driver mode, forward will be relative to field
		//math that makes fieldOriented work
		if (isFieldOriented()) {
			
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = forward * Math.cos(angleRad) +
					strafe * Math.sin(angleRad);
			strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
			forward = temp;
		}
		
		//linear movement while rotating(speeds of different sides of robot)
		//potential for fancy tricks
		//diagram on git
		double frontSpeedDif = strafe - rotation * (WHEELBASE / TRACKWIDTH); //strafe while rotating
		double backSpeedDif = strafe + rotation * (WHEELBASE / TRACKWIDTH); //strafe while rotating
		double leftSpeedDif = forward - rotation * (TRACKWIDTH / WHEELBASE); //forward while rotating speeds
		double rightSpeedDif = forward + rotation * (TRACKWIDTH / WHEELBASE); //forward while rotating speeds

		double[] angles = new double[]{ //gives orientation of each wheel in degrees from speeds (vectors pt1)
				Math.atan2(backSpeedDif, leftSpeedDif) * 180 / Math.PI,
				Math.atan2(backSpeedDif, rightSpeedDif) * 180 / Math.PI,
				Math.atan2(frontSpeedDif, rightSpeedDif) * 180 / Math.PI,
				Math.atan2(frontSpeedDif, leftSpeedDif) * 180 / Math.PI
		};

		double[] speeds = new double[]{ //diagonal movement combined from linear motion (vectors pt2)
				//pythagorean thereom: Math.hypot(x, y) = sqrt(x^2 + y^2)
				Math.hypot(backSpeedDif, leftSpeedDif),
				Math.hypot(backSpeedDif, rightSpeedDif),
				Math.hypot(frontSpeedDif, rightSpeedDif),
				Math.hypot(frontSpeedDif, leftSpeedDif)
		};

		SmartDashboard.putNumber("Module 0 Ticks", mSwerveModules[0].getPosition());
		SmartDashboard.putNumber("Module 1 Ticks", mSwerveModules[1].getPosition());
		SmartDashboard.putNumber("Module 2 Ticks", mSwerveModules[2].getPosition());
		SmartDashboard.putNumber("Module 3 Ticks", mSwerveModules[3].getPosition());

		//giving modules their direction and speed (vector)
		for (int i = 0; i < 4; i++) {
			if (Math.abs(forward) > 0.05 ||
			    Math.abs(strafe) > 0.05 ||
			    Math.abs(rotation) > 0.05) {
				mSwerveModules[i].setTargetAngle(angles[i] + 180, isAuto); //"+180" this is why we can't have nice things --simplify, fix bug
			} else {
				mSwerveModules[i].setTargetAngle(mSwerveModules[i].getTargetAngle(), isAuto);
			}
			mSwerveModules[i].setTargetSpeed(speeds[i]);
		}
	}

	@Override
	public void stopDriveMotors() {
		for (SwerveDriveModule module : mSwerveModules) {
			module.setTargetSpeed(0);
		}
	}

	public void resetAllEncoders() {
		for(int i = 0; i < 4; i++) {
			mSwerveModules[i].resetEncoder();
		}
	}

	//distance traveled(odometer)
	public double getInches() {
		return mSwerveModules[0].getInches();
	}

	public void driveForwardDistance(double targetPos, double angle) { // inches & degrees
		double angleError = ((angle - mNavX.getYaw()) / 180)*10;

		angleError = Math.min(angleError, 1);
		angleError = Math.max(angleError, -1);
		targetPos = (targetPos * Constants.GEAR_RATIO)/(Constants.WHEEL_SIZE*Math.PI); //inches to ticks  -- can remove this magic number?  
		for (int i = 0; i < 4; i++) {
				mSwerveModules[i].setTargetAngle(angle, isAuto); //mSwerveModules[i].getTargetAngle());
				mSwerveModules[i].setTargetDistance(targetPos+mSwerveModules[i].getDriveMotor().getSelectedSensorPosition());
			}
			
	} // 2/12/19 3:37 PM i want boba and a burrito so bad right now !!!!!!!!!

	public void swapPIDSlot(int slot)
	{
		for(int i = 0; i < 4; i++)
		{
			mSwerveModules[i].setPIDSlot(slot);
		}
	}

	public void swapDrivePIDSlot(int slot)
	{
		for(int i = 0; i < 4; i++)
		{
			mSwerveModules[i].setDrivePIDSlot(slot);
		}
	}

	public void driveSidewaysDistance(double targetPos, double angle, double speed) {
		double angleError = ((angle - mNavX.getYaw()) / 180)*10;
		angleError = Math.min(angleError, 1);
		angleError = Math.max(angleError, -1);
		//holonomicDrive(0, speed, angleError);
	}

	public double calculateErrPos(double d1) {
		return d1 - mSwerveModules[0].getInches(); //return d1 - mSwerveModules[0].getDriveDistance();
	}

	public boolean getIsAuto()
	{
		return isAuto;
	}

	public void setIsAuto(boolean is)
	{
		isAuto = is;
	}

	@Override 
	public void periodic() { //is this needed here?
		SmartDashboard.putBoolean("Mod 0 Motor Inversion", mSwerveModules[0].getDriveMotor().getInverted());
		SmartDashboard.putBoolean("Mod 1 Motor Inversion", mSwerveModules[1].getDriveMotor().getInverted());
		SmartDashboard.putBoolean("Mod 2 Motor Inversion", mSwerveModules[2].getDriveMotor().getInverted());
		SmartDashboard.putBoolean("Mod 3 Motor Inversion", mSwerveModules[3].getDriveMotor().getInverted());
		SmartDashboard.putNumber("Mod 0 Angle", mSwerveModules[0].getCurrentAngle());
		SmartDashboard.putNumber("Mod 1 Angle", mSwerveModules[1].getCurrentAngle());
		SmartDashboard.putNumber("Mod 2 Angle", mSwerveModules[2].getCurrentAngle());
		SmartDashboard.putNumber("Mod 3 Angle", mSwerveModules[3].getCurrentAngle());
	}
}

