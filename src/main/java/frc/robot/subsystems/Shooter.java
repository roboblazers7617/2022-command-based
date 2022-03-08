// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;
import frc.robot.commands.StopShooter;

public class Shooter extends SubsystemBase {
  private final PWMSparkMax shooterMotor = new PWMSparkMax(Constants.SHOOTER_PORT);
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  private NetworkTableEntry shooterSpeedDisplay = tab.add("Shooter Motor Speed: ", 0).getEntry();
  private final SendableChooser<Double> toggleShooter = new SendableChooser<Double>();
 // private final RelativeEncoder encoder = shooterMotor.getEncoder();
  private final NetworkTableEntry shooterMotorEntry, shooterStateEntry;
 
  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor.setInverted(false);
    shooterMotorEntry = ShuffleboardInfo.getInstance().getShooterMotorEntry();
    shooterStateEntry = ShuffleboardInfo.getInstance().getShooterStateEntry();

    /*toggleShooter.setDefaultOption("yes shoot", -1.0);
    toggleShooter.setDefaultOption("no shoot", 0.0);
    tab.add(toggleShooter);*/
  }
  public void setSpeed(double speed){
    shooterMotor.set(speed);
  }

  public void startShooter(){
    setSpeed(Constants.SHOOTER_MOTOR_SPEED);
  }

  public void stopShooter(){
    setSpeed(0.0);
  }

  public double getSpeed(){
    return shooterMotor.get();
  }

  /**returns whether shooter is at full speed */
  public boolean shooterReady(){
    /*if(encoder.getVelocity() > Constants.SHOOTER_MOTOR_SPEED_FULL){
      return true;
    }
    return false;*/
    return true;
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    shooterMotorEntry.setDouble(getSpeed());
    shooterStateEntry.setBoolean(shooterReady());

  }
}
