// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final int LEFT_FRONT_WHEEL_PORT = 1;
    public static final int RIGHT_FRONT_WHEEL_PORT = 2;
    public static final int LEFT_BACK_WHEEL_PORT = 3;
    public static final int RIGHT_BACK_WHEEL_PORT = 4;
    public static final int CONTROLLER_PORT = 0;
    public static final double HIGH_GEAR = 1.0;
    public static final double LOW_GEAR = 0.5;
    public static final int SPEED_ADJUSTOR_TRIGGER = 5;
    public static final int INTAKE_PORT = 6;
    public static double turnTolerance = 3;
    public static double turnRateTolerance = 1;
    public static final int WHEEL_RADIUS = 4;//inches
    public static final double WHEEL_GEAR_RATIO = 5/12;
    public static final double KS = 0;
    public static final double KV = 0;
    public static final double KA = 0;
    public static final double MAX_ACCELERATION = 5;//feet per second squared
    public static final double MAX_VELOCITY = 2;//feet per second
    public static final double MAX_ROTATION = Math.PI;
    public static final double MAX_ROTATION_CHANGE = Math.PI/4;
    

}
