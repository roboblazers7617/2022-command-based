// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
  private final CANSparkMax lowerMotor = new CANSparkMax(Constants.LOWER_TOWER_MOTOR,MotorType.kBrushless);
  private final CANSparkMax upperMotor = new CANSparkMax(Constants.UPPER_TOWER_MOTOR,MotorType.kBrushless);
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  private NetworkTableEntry upperSpeed = tab.add("Upper Tower Motor Speed: ", 0).getEntry();
  private NetworkTableEntry lowerSpeed = tab.add("Lower Tower Motor Speed: ", 0).getEntry();
  private AnalogInput ultrasonicSensorLower = new  AnalogInput(Constants.LOWER_SENSOR_PORT);
  private AnalogInput ultrasonicSensorUpper = new  AnalogInput(Constants.UPPER_SENSOR_PORT);
  private NetworkTableEntry upperSensorDisplay = tab.add("Upper Sensor: ", getUpperSensor()).getEntry();
  private NetworkTableEntry lowerSensorDisplay = tab.add("Lower Sensor: ", getLowerSensor()).getEntry();
  private final SendableChooser<Boolean> sensorChooser = new SendableChooser<Boolean>();
  /** Creates a new Tower. 69 haha funny number*/
  public Tower() {
    sensorChooser.setDefaultOption("no ball", true);
    sensorChooser.setDefaultOption("yes ball", false);
    Shuffleboard.getTab("Debug").add(sensorChooser);
  }
  public void setSpeedUpper(double speed){
    upperMotor.set(speed);
  }

  public double getSpeedUpper(){
    return upperMotor.get();
  }

  public void setSpeedLower(double speed){
    lowerMotor.set(speed);
  }

  public double getSpeedLower(){
    return lowerMotor.get();
  }

  public boolean getUpperSensor(){
    if(ultrasonicSensorUpper.getValue() > 4000){
      return true;
    }
    return false;
  }

  public boolean getLowerSensor(){
    if(ultrasonicSensorLower.getValue() > 4000){
      return true;
    }
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upperSpeed.setDouble(upperMotor.get());
    lowerSpeed.setDouble(lowerMotor.get());
    upperSensorDisplay.setBoolean(getUpperSensor());
    lowerSensorDisplay.setBoolean(getLowerSensor());

  }
}
