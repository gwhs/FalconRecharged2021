package frc.robot.subsystems.Drive;

public abstract class HolonomicDrivetrain extends Drivetrain {

	private double mAdjustmentAngle = 0; //value you subtract from the actual orientation to get the relative orientation in degrees
	private boolean mFieldOriented = true; //fieldOrientation sets direction relative to the field

	public double getAdjustmentAngle() {
		return mAdjustmentAngle;
	}

	public abstract double getGyroAngle(); //current adjusted orienation of robot

	public abstract void holonomicDrive(double forward, double strafe, double rotation); //setting values for holonomic drive

	public abstract void swapPIDSlot(int slot); //changing which set of PID values you use

	public boolean isFieldOriented() {
		return mFieldOriented;
	}

	/**
	 * value you subtract from the actual orientation to get the relative orientation in degrees
	 * @param adjustmentAngle
	 */
	public void setAdjustmentAngle(double adjustmentAngle) {
		mAdjustmentAngle = adjustmentAngle;
	}

	/**
	 * fieldOriented keeps direction consistent to the field
	 * not fieldOriented keeps direction relative to the robot
	 * @param fieldOriented
	 */
	public void setFieldOriented(boolean fieldOriented) {
		mFieldOriented = fieldOriented;
	}

	public abstract void stopDriveMotors(); //stops drive motors

	/**
	 * sets wherever robot is facing to orientation of 0 degrees
	 */
	public void zeroGyro() {
		setAdjustmentAngle(getGyroAngle() + getAdjustmentAngle());
	}
}
