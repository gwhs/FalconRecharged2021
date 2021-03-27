/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * DaphneTwoConstants provides a convenient place for to hold robot-specific numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class DaphneTwoConstants {
    public static final int ANGLE1_TALON = 1;
    public static final int ANGLE2_TALON = 3;
    public static final int ANGLE3_TALON = 7;
    public static final int ANGLE4_TALON = 5;
    public static final int DRIVE1_TALON = 0;
    public static final int DRIVE2_TALON = 2;
    public static final int DRIVE3_TALON = 6;
    public static final int DRIVE4_TALON = 4;

    public static final double LL_HEIGHT = 23; //inches
    public static final double LL_MOUNT_ANGLE = 50; //degrees
    public static final double VISION_TARGET_HEIGHT = 90;  //inches

    public static final int GREEN_RPM = 6000; //speed to shoot from right in front
    public static final int YELLOW_RPM = 4000; //speed to shoot from yellow zone
    public static final double CONVEYOR_UNLOADS_SPEED = -0.6;

}
