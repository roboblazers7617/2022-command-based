// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;

public class Climber extends SubsystemBase {
 // private final PWMSparkMax leftClimber = new PWMSparkMax(Constants.LEFT_CLIMBER_PORT);
  
  private final CANSparkMax rightTopClimber = new CANSparkMax(Constants.RIGHT_TOP_CLIMBER_PORT, MotorType.kBrushless);
  private final CANSparkMax bottomClimber = new CANSparkMax(Constants.LEFT_BOTTOM_CLIMBER_PORT, MotorType.kBrushless);
  private final CANSparkMax leftTopClimber = new CANSparkMax(Constants.LEFT_TOP_CLIMBER_PORT, MotorType.kBrushless);
  private final NetworkTableEntry climberTopSpeedEntry;
  private final NetworkTableEntry climberBottomSpeedEntry;
  // private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  // private NetworkTableEntry leftClimberDisplay = tab.add("Left Climber Display: ", 0.0).getEntry();
  // private NetworkTableEntry rightClimberDisplay = tab.add("Right Climber Display: ", 0.0).getEntry();
  // private final SendableChooser<Double> climberToggle = new SendableChooser<Double>();
  /** Creates a new Climber. */
  public Climber() {
    // climberToggle.setDefaultOption("go climb ", 0.1);
    // climberToggle.setDefaultOption("un climb ", -0.1);
    // climberToggle.setDefaultOption("no climb no", 0.0);
    // tab.add(climberToggle);

    rightTopClimber.restoreFactoryDefaults();
    bottomClimber.restoreFactoryDefaults();
    leftTopClimber.restoreFactoryDefaults();

    climberBottomSpeedEntry = ShuffleboardInfo.getInstance().getBottomClimbEntry();
    climberTopSpeedEntry = ShuffleboardInfo.getInstance().getTopClimbEntry();
  }

  public void setSpeedTop(double speed){
    SmartDashboard.putNumber("Climber speed bghhf", speed);
   // leftClimber.set(speed);
    rightTopClimber.set(speed);
    leftTopClimber.set(speed);
  //  SmartDashboard.putNumber("LeftClimber Speed Read", leftClimber.get());
  }

  public void setSpeedBottom(double speed){

    bottomClimber.set(speed);
  }

  

  public double getSpeedTop(){
  //  return leftClimber.get();
  return rightTopClimber.get();
  }
  
  public double getSpeedBottom(){
    return bottomClimber.get();
  }

  @Override
  public void periodic() {
    climberBottomSpeedEntry.setDouble(getSpeedBottom());
    climberTopSpeedEntry.setDouble(getSpeedTop());
    //leftClimberDisplay.setDouble(getSpeedLeft());
    //rightClimberDisplay.setDouble(getSpeedRight());
    //setSpeed(climberToggle.getSelected());
    // This method will be called once per scheduler run

    
  }
}
