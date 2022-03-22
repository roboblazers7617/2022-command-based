// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;
import frc.robot.commands.StopShooter;

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor = new CANSparkMax(Constants.SHOOTER_PORT, MotorType.kBrushless);
  public final DigitalInput shooterSensor = new DigitalInput(Constants.SHOOTER_SENSOR_PORT_INPUT);
  private final RelativeEncoder encoder;
  private final SparkMaxPIDController pidController;
  public final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private double setPoint;



  private final NetworkTableEntry shooterMotorEntry, shooterStateEntry, shooterSensorEntry;
 
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor.restoreFactoryDefaults();
    shooterMotor.setInverted(true);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    shooterMotorEntry = ShuffleboardInfo.getInstance().getShooterMotorEntry();
    shooterStateEntry = ShuffleboardInfo.getInstance().getShooterStateEntry();
    shooterSensorEntry = ShuffleboardInfo.getInstance().getShooterSensorEntry();
    encoder = shooterMotor.getEncoder();
    pidController = shooterMotor.getPIDController();


    kP = Constants.SHOOTER_kP; 
    kI = Constants.SHOOTER_kI;
    kD = Constants.SHOOTER_kD; 
    kIz = Constants.SHOOTER_kIz; 
    kFF = Constants.SHOOTER_kFF; 
    kMaxOutput = Constants.SHOOTER_kMaxOutput; 
    kMinOutput = Constants.SHOOTER_kMinOutput;


    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);
    
    /*toggleShooter.setDefaultOption("yes shoot", -1.0);
    toggleShooter.setDefaultOption("no shoot", 0.0);
    tab.add(toggleShooter);*/
  }
  private void setVelocity(double speed){

    if(speed == 0.0)
      shooterMotor.set(0.0);
    else{
      
       setPoint = speed;
      
    }
  }

  public void setSetPoint(double setPoint) {
      this.setPoint = setPoint;
  }

  public void startShooter(){
    setVelocity(setPoint);
  }

  public void startShooterFast(){
    pidController.setReference(1850, CANSparkMax.ControlType.kVelocity);
  }

  public void stopShooter(){
    setVelocity(0.0);
  }

  public double getSpeed(){
    return encoder.getVelocity();
  }

  /**returns whether shooter is at full speed */
  public boolean shooterReady(){
    if(encoder.getVelocity() > Constants.SHOOTER_MOTOR_TARGET_MIN*setPoint && encoder.getVelocity() < Constants.SHOOTER_MOTOR_TARGET_MAX*setPoint){
      return true;
    }
    return false;

  }

  public boolean getShooterSensor(){
    return !shooterSensor.get();
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterMotorEntry.setDouble(getSpeed());
    shooterStateEntry.setBoolean(shooterReady());
    shooterSensorEntry.setBoolean(shooterSensor.get());
    pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

  }
}
