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


    //Speeds
    public static final double TOWER_SPEED = 0.6;
    public static final double INTAKE_MOTOR_SPEED = 0.4;
    public static final double HIGH_GEAR = 1;
    public static final double LOW_GEAR = .5;
    public static final double CLIMBER_SPEED = 0.25;
    public static final double SHOOTER_MOTOR_SPEED = 1.0;
    /**the speed relative to SHOOTER_MOTOR_SPEED that shooter will count as full speed for shooterReady() */
    public static final double SHOOTER_MOTOR_SPEED_FULL = .95 * SHOOTER_MOTOR_SPEED;
    //PWM Ports
    public static final int LEFT_BOTTOM_CLIMBER_PORT = 4;
    public static final int LEFT_TOP_CLIMBER_PORT = 9;
    public static final int RIGHT_BOTTOM_CLIMBER_PORT = 13;
    public static final int RIGHT_TOP_CLIMBER_PORT = 14;
    public static final int LOWER_TOWER_MOTOR = 5;
    public static final int UPPER_TOWER_MOTOR = 6;
    public static final int INTAKE_PORT = 10;
    //CAN Ports
    public static final int LEFT_FRONT_WHEEL_PORT = 1;
    public static final int RIGHT_FRONT_WHEEL_PORT = 8;
    public static final int LEFT_BACK_WHEEL_PORT = 7;
    public static final int RIGHT_BACK_WHEEL_PORT =11;
    public static final int INTAKE_ROTATION_PORT = 12;
    public static final int SHOOTER_PORT = 4;
    

    //DIO Ports
    public static final int LOWER_SENSOR_PORT_INPUT = 1;
    public static final int UPPER_SENSOR_PORT_INPUT = 0;
    

    
    public static final double INTAKE_ROTATION_MOTOR_SPEED = 0.25;
    public static final double INTAKE_ROTATION_MOTOR_DISTANCE = 0.25;
    
    //Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int SHOOTER_CONTROLLER_PORT = 1;
    //Driver Button Bindings
    public static final int CLIMBER_TOP_FOWARD_BUTTON = 4; // yellow button
    public static final int CLIMBER_TOP_BACKWARD_BUTTON = 2; // red button
    public static final int CLIMBER_BOTTOM_FORWARD_BUTTON = 3;  // blue button
    public static final int CLIMBER_BOTTOM_BACKWARD_BUTTON = 1; //green button
    public static final int SPEED_ADJUSTOR_TRIGGER = 5; // left bumper
    //shooter button bindings
    public static final int SHOOT_BOLL_BUTTON = 6;//right bumper
    public static final int COLLECT_BALLS_BUTTON = 4; // yellow button
    public static final int STOP_COLLECT_BALLS_BUTTON = 3; //blue button
    public static final int RUN_TOWER_BUTTON = 2; //red button
    
    public static final int REVERSE_TOWER_BUTTON = 1; //green button
    //Intake data
    public static final double GEAR_RATIO_INTAKE_LIFT = (15/54)*1/1.925;
    public static final int ANGLE_INTAKE_DEPLOY = 80;

}
