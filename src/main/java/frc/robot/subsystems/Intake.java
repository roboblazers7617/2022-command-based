// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.commands.ResetIntake;
import frc.robot.commands.ToggleIntakeRotation;
import edu.wpi.first.networktables.NetworkTableEntry;

import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final PWMVictorSPX intakeMotor = new PWMVictorSPX(Constants.INTAKE_PORT);
  // private SparkMaxLimitSwitch fowardLimit;
  // private SparkMaxLimitSwitch backwordsLimit;
  //intake rotation motor is the motor that raises the intake off ground
  private final CANSparkMax intakeRotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_PORT,MotorType.kBrushless);
  private boolean intakeRotationMotorRaised;
  //private boolean movingIntakeRotationMotor;
  private boolean upperLimitSwitch;
  private boolean lowerLimitSwitch;
  private ShuffleboardTab tab = Shuffleboard.getTab("Debug");
  private NetworkTableEntry speedDisplay = tab.add("Intake Motor Speed: ", 0).getEntry();
  
  private NetworkTableEntry intakeRotationSpeedDisplay = tab.add("Intake Rotation Motor Speed: ", 0).getEntry();
  private RelativeEncoder encoder = intakeRotationMotor.getEncoder();
  /** Creates a new Intake. */
  public Intake() {
    intakeMotor.setInverted(true);
    intakeRotationMotor.setInverted(true);
    intakeRotationMotorRaised = true;

    tab.add("toggle intake rotation: ", new ToggleIntakeRotation(this));
    
  }
  /**sets the speed for the intake motor not the intake rotation motor */
  public void setSpeedIntake(double speed){//for intake motor
    intakeMotor.set(speed);
  }
  /**gets the speed for the intake motor not the intake rotation motor */
  public double getSpeedIntake(){//for intake motor
    return intakeMotor.get();
  }

  public boolean getIntakeRotationMotorRaised(){
    return intakeRotationMotorRaised;
  }

  // public boolean isIntakeRotationMotorMoving(){
  //   return movingIntakeRotationMotor;
  // }

  public boolean isIntakeRasing(){
    if(upperLimitSwitch || encoder.getPosition() < 0.05){
      return false;
    }
    return true;
  }

  public boolean isIntakeLowering(){
    if(lowerLimitSwitch || encoder.getPosition() > 0.20){
      return false;
    }
    return true;
  }

  public void stopIntakeRotation(){
    intakeRotationMotor.set(0.0);
  }

  

  

  /** will raise the intake up within the robot */
  public void raiseIntake(){
    //SparkMaxLimitSwitch limit = intakeRotationMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED);

    //used this for help https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Soft%20Limits/src/main/java/frc/robot/Robot.java 
    intakeRotationMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    intakeRotationMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) Constants.INTAKE_ROTATION_MOTOR_DISTANCE);
    
    
    


  }

  /** will lower the intake down to the ground */
  public void lowerIntake(){
    //SparkMaxLimitSwitch limit = intakeRotationMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED);

    //used this for help https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Soft%20Limits/src/main/java/frc/robot/Robot.java 
    intakeRotationMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    intakeRotationMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) 0.0);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    speedDisplay.setDouble(getSpeedIntake());//update the intake motor speed display

    intakeRotationSpeedDisplay.setDouble(intakeRotationMotor.get());//updates the display for the intake rotation motor speed
  }
}
