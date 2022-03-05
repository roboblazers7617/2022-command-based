// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final PWMVictorSPX intakeMotor = new PWMVictorSPX(Constants.INTAKE_PORT);
  //intake rotation motor is the motor that raises the intake off ground
  private final PWMSparkMax intakeRotationMotor = new PWMSparkMax(Constants.INTAKE_ROTATION_PORT);
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  private NetworkTableEntry speedDisplay = tab.add("Intake Motor Speed: ", 0).getEntry();
  private NetworkTableEntry intakeRotationDisplay = tab.add("Intake Rotation Motor Position: ", 0).getEntry();
  private NetworkTableEntry intakeRotationSpeedDisplay = tab.add("Intake Rotation Motor Speed: ", 0).getEntry();
  //private RelativeEncoder encoder = intakeRotationMotor.getEncoder();
  /** Creates a new Intake. */
  public Intake() {
    
  }

  public void setSpeed(double speed){//for intake motor
    intakeMotor.set(speed);
  }

  public double getSpeed(){//for intake motor
    return intakeMotor.get();
  }

  public double getEncoderValue(){//for intake rotation motor
    return 0;//encoder.getPosition();
  }

  public void setRotationMotorSpeed(double speed){
    intakeRotationMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speedDisplay.setDouble(getSpeed());//update the intake motor speed display
    intakeRotationDisplay.setDouble(getEncoderValue());//updates the display showing the rotation
    intakeRotationSpeedDisplay.setDouble(intakeRotationMotor.get());//updates the display for the intake rotation motor speed
  }
}
