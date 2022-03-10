// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_WHEEL_PORT,MotorType.kBrushless);
  private final RelativeEncoder leftFrontEncoder = leftFrontMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();
  private final MecanumDrive drivetrain;
  private final AHRS gyro = new AHRS(Port.kMXP);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();
    //Original Inversions
    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);
    leftFrontMotor.setIdleMode(IdleMode.kCoast);
    rightFrontMotor.setIdleMode(IdleMode.kCoast);
    leftBackMotor.setIdleMode(IdleMode.kCoast);
    rightBackMotor.setIdleMode(IdleMode.kCoast);
    drivetrain = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    drivetrain.setMaxOutput(Constants.LOW_GEAR);
    gyro.calibrate();
  }
  public void drive(double ySpeed, double xSpeed, double zRotation){
    drivetrain.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Left Front Velocity", leftFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Right Front Velocity", rightFrontEncoder.getVelocity());
    SmartDashboard.putNumber("Left Back Velocity", leftBackEncoder.getVelocity());
    SmartDashboard.putNumber("Right Back Velocity", rightBackEncoder.getVelocity());

  }
  public void setMaxSpeed(double speed) {
    drivetrain.setMaxOutput(speed);
  }
  public double getGyro(){
    return gyro.getAngle();
  }
  public void resetGyro(){
    gyro.reset();
  }
  public void setSpeeds(double leftFrontSpeed, double rightFrontSpeed, double leftBackSpeed, double rightBackSpeed){
    leftFrontMotor.set(leftFrontSpeed);
    rightFrontMotor.set(rightFrontSpeed);
    leftBackMotor.set(leftBackSpeed);
    rightBackMotor.set(rightBackSpeed);
  }
  public double getAverageEncoderPosition(){
    return (leftFrontEncoder.getPosition()+rightFrontEncoder.getPosition()+leftBackEncoder.getPosition()+rightBackEncoder.getPosition())/4;
  }
}
