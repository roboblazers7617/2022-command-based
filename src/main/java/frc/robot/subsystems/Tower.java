// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
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
  private boolean upperSensor = true;
  private boolean lowerSensor = true;
  private NetworkTableEntry upperSensorDisplay = tab.add("Upper Sensor: ", upperSensor).getEntry();
  private NetworkTableEntry lowerSensorDisplay = tab.add("Lower Sensor: ", lowerSensor).getEntry();
  private final SendableChooser<Boolean> sensorChooser = new SendableChooser<Boolean>();
  /** Creates a new Tower. */
  public Tower() {
    upperSensor = true;
    lowerSensor = true;
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
    return upperSensor;
  }

  public boolean getLowerSensor(){
    return lowerSensor;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    upperSpeed.setDouble(upperMotor.get());
    lowerSpeed.setDouble(lowerMotor.get());
    upperSensorDisplay.setBoolean(getUpperSensor());
    lowerSensorDisplay.setBoolean(getLowerSensor());
    lowerSensor = sensorChooser.getSelected();
  }
}
