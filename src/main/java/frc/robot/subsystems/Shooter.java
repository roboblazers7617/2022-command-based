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

public class Shooter extends SubsystemBase {
  private final CANSparkMax shooterMotor = new CANSparkMax(Constants.SHOOTER_PORT,MotorType.kBrushless);
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  private NetworkTableEntry shooterSpeedDisplay = tab.add("Shooter Motor Speed: ", 0).getEntry();
  private final SendableChooser<Double> toggleShooter = new SendableChooser<Double>();
  /** Creates a new Shooter. */
  public Shooter() {
    toggleShooter.setDefaultOption("yes shoot", 0.25);
    toggleShooter.setDefaultOption("no shoot", 0.0);
    tab.add(toggleShooter);
  }
  public void setSpeed(double speed){
    shooterMotor.set(speed);
  }

  public double getSpeed(){
    return shooterMotor.get();
  }

  

  @Override
  public void periodic() {
    shooterSpeedDisplay.setDouble(getSpeed());
    setSpeed(toggleShooter.getSelected());
    // This method will be called once per scheduler run
  }
}
