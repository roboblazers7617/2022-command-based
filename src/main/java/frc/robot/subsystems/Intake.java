// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_PORT,MotorType.kBrushless);
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  private NetworkTableEntry speed = tab.add("Intake Motor Speed: ", 0).getEntry();
  /** Creates a new Intake. */
  public Intake() {
    
  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }

  public double getSpeed(){
    return intakeMotor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speed.setDouble(intakeMotor.get());
  }
}
