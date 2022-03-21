// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.deser.std.ContainerDeserializerBase;

import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //PWM Ports
    public static final int INTAKE_PORT = 5;
    
    //CAN Ports
    public static final int LEFT_FRONT_WHEEL_PORT = 8;
    public static final int RIGHT_FRONT_WHEEL_PORT = 9;
    public static final int LEFT_BACK_WHEEL_PORT = 7;
    public static final int RIGHT_BACK_WHEEL_PORT =6;
    public static final int INTAKE_ROTATION_PORT = 10;
    public static final int SHOOTER_PORT = 1;
    public static final int BOTTOM_CLIMBER_PORT = 4;
    public static final int LEFT_TOP_CLIMBER_PORT = 3;
    public static final int RIGHT_TOP_CLIMBER_PORT = 2;
    public static final int LOWER_TOWER_MOTOR = 11;
    public static final int UPPER_TOWER_MOTOR = 5;

    //DIO Ports
    public static final int LOWER_SENSOR_PORT_INPUT = 0;
    public static final int UPPER_SENSOR_PORT_INPUT = 9;
    public static final int INTAKE_LIMIT_UPPER_PORT = 5;
    public static final int INTAKE_LIMIT_LOWER_PORT = 4;
    /**need to change */
    public static final int SHOOTER_SENSOR_PORT_INPUT = 7;

    //Controller Ports
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int SHOOTER_CONTROLLER_PORT = 1;
    //Driver Button Bindings
    public static final int CLIMBER_TOP_FOWARD_BUTTON = XboxController.Button.kY.value; // yellow button
    public static final int CLIMBER_TOP_BACKWARD_BUTTON = XboxController.Button.kX.value; // red button
    public static final int CLIMBER_BOTTOM_FORWARD_BUTTON = XboxController.Button.kB.value;  // blue button
    public static final int CLIMBER_BOTTOM_BACKWARD_BUTTON = XboxController.Button.kA.value; //green button
    public static final int SPEED_ADJUSTOR_TRIGGER = XboxController.Button.kLeftBumper.value; // left bumper
    //public static final int SHORT_ARM_UP_BUTTON = 55;
    //public static final int SHORT_ARM_DOWN_BUTTON = 56;
    //public static final int LONG_ARM_UP_BUTTON = 57;
    
    //shooter button bindings
    public static final int SHOOT_BOLL_BUTTON = XboxController.Button.kRightBumper.value;//right bumper
    public static final int COLLECT_BALLS_BUTTON = XboxController.Button.kB.value; // green button
    public static final int STOP_COLLECT_BALLS_BUTTON = XboxController.Button.kA.value; //red button   
    public static final int REVERSE_INTAKE_BUTTON = XboxController.Button.kX.value;
    public static final int ACTIVATE_INTAKE_BUTTON = XboxController.Button.kY.value;
    public static final int RESET_INTAKE_BUTTON = XboxController.Button.kLeftBumper.value;




    //other controler data
    public static final double TRIGGER_THRESHOLD = 0.5;
    
    //Tower
    public static final double LOWER_TOWER_SPEED = 0.25;
    public static final double UPPER_TOWER_SPEED = .4;
    public static final double NERF_LOWER_TOWER = -3;

    //Drivetrain Data
    public static final double DRIVETRAIN_KP = 0;
    public static final double DRIVETRAIN_KI = 0;
    public static final double DRIVETRAIN_KD = 0;
    public static final int WHEEL_RADIUS = 4;//inches
    public static final double WHEEL_GEAR_RATIO = 1/12.75;
    public static final double DRIVETRAIN_ROTATIONAL_KP = 0;
    public static final double DRIVETRAIN_ROTATIONAL_KI = 0;
    public static final double DRIVETRAIN_ROTAIONAL_KD = 0;
    public static final double LOW_GEAR = .25;
    public static final double HIGH_GEAR = .75;
    public static final double SUPER_HIGH_GEAR = .55;
    public static final double DRIVETRAIN_ENCODER_DISTANCE_PER_ROTATION = 2*Math.PI*WHEEL_RADIUS*WHEEL_GEAR_RATIO;
    public static final double DISTANCE_FROM_FENDER_TO_TAXI = 110;
    public static final double AUTO_SPEED = .4;

    //Intake Data
    public static final double INTAKE_LOWER_ENCODER_VALUE = -13;
    public static final double INTAKE_UPPER_ENCODER_VALUE = 0;
    public static final double INTAKE_ROTATION_MOTOR_DISTANCE = 0.25;
    public static final double INTAKE_ROTATION_MOTOR_SPEED_UP = 0.125;   
    public static final double INTAKE_ROTATION_MOTOR_SPEED_DOWN = 0.25;   
    public static final double INTAKE_MOTOR_SPEED = 0.7;
    public static final int ANGLE_INTAKE_DEPLOY = 80;
    public static final int INTAKE_GRAVITY_LOWER_TIME = 500;

    //Shooter data
    public static final double SHOOTER_SETPOINT = 2100;
    /**the speed relative to SHOOTER_MOTOR_SPEED that shooter will count as the minunum speed for shooterReady() */
    public static final double SHOOTER_MOTOR_TARGET_MIN = .98 * SHOOTER_SETPOINT;
    /**the speed relative to SHOOTER_MOTOR_SPEED that shooter will count as the max speed for shooterReady() */
    public static final double SHOOTER_MOTOR_TARGET_MAX = 1.05 * SHOOTER_SETPOINT;
    
    public static final double SHOOTER_kP = .0004; 
    public static final double SHOOTER_kI = 0;
    public static final double SHOOTER_kD = 0; 
    public static final double SHOOTER_kIz = 0; 
    public static final double SHOOTER_kFF = 0.00018; 
    public static final double SHOOTER_kMaxOutput = 1; 
    public static final double SHOOTER_kMinOutput = -1;

    //Climber
    public static final double RIGHT_UPPER_CLIMBER_SPEED = 0.85;
    public static final double LEFT_UPPER_CLIMBER_SPEED = .85;
    public static final double LOWER_CLIMBER_SPEED = 1;
}
