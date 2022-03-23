// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;

public class Tower extends SubsystemBase {
  private final NetworkTableEntry towerUpperMotorEntry, towerLowerMotorEntry, towerLowerSensorEntry,
      towerUpperSensorEntry;

  private final CANSparkMax lowerMotor = new CANSparkMax(Constants.LOWER_TOWER_MOTOR,MotorType.kBrushless);
  private final CANSparkMax upperMotor = new CANSparkMax(Constants.UPPER_TOWER_MOTOR,MotorType.kBrushless);

  public final DigitalInput lowerSensor = new DigitalInput(Constants.LOWER_SENSOR_PORT_INPUT);
  public final DigitalInput upperSensor = new DigitalInput(Constants.UPPER_SENSOR_PORT_INPUT);

  public Tower() {
    lowerMotor.restoreFactoryDefaults();
    upperMotor.restoreFactoryDefaults();
    lowerMotor.setInverted(true);
    upperMotor.setInverted(true);
    lowerMotor.setIdleMode(IdleMode.kBrake);
    upperMotor.setIdleMode(IdleMode.kBrake);
    towerUpperMotorEntry = ShuffleboardInfo.getInstance().getTowerUpperMotorEntry();
    towerLowerMotorEntry = ShuffleboardInfo.getInstance().getTowerLowerMotorEntry();
    towerUpperSensorEntry = ShuffleboardInfo.getInstance().getTowerUpperSensorEntry();
    towerLowerSensorEntry = ShuffleboardInfo.getInstance().getTowerLowerSensorEntry();

  }

  /** Sets the upper motor speed for the tower. */
  public void setSpeedUpper(double speed) {
    upperMotor.set(speed);
  }

  /** Returns the set speed of the upper motor. */
  public double getSpeedUpper() {

    return upperMotor.get();
  }

  /** Sets the speed of the lower tower motor. */
  public void setSpeedLower(double speed) {
    lowerMotor.set(speed);
  }

  /** Returns the set speed of the lower motor */
  public double getSpeedLower() {
    return lowerMotor.get();
  }

  public void stop() {
    lowerMotor.set(0);
    upperMotor.set(0);
  }

  public boolean isBallHereLower(){
    boolean sensorVal = lowerSensor.get();
    return !sensorVal;
  }

  public boolean isBallHereUpper(){
    boolean sensorVal = upperSensor.get();
    return !sensorVal;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    towerUpperMotorEntry.setDouble(getSpeedUpper());
    towerLowerMotorEntry.setDouble(getSpeedLower());
    //towerUpperSensorEntry.setBoolean(upperSensor.get());
    //towerLowerSensorEntry.setBoolean(lowerSensor.get());
    towerUpperSensorEntry.setBoolean(isBallHereUpper());
    towerLowerSensorEntry.setBoolean(isBallHereLower());

  }
}
