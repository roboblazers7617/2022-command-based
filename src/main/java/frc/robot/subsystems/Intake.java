// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.ShuffleboardInfo;
import frc.robot.commands.ResetIntake;
import frc.robot.commands.ToggleIntakeRotation;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final PWMVictorSPX intakeMotor = new PWMVictorSPX(Constants.INTAKE_PORT);
  // private SparkMaxLimitSwitch fowardLimit;
  // private SparkMaxLimitSwitch backwordsLimit;
  //intake rotation motor is the motor that raises the intake off ground
  private final CANSparkMax intakeRotationMotor = new CANSparkMax(Constants.INTAKE_ROTATION_PORT,MotorType.kBrushless);

  //private boolean movingIntakeRotationMotor;
  //CLOSED IS CLOSED
  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;


  private boolean gravityLoweringIntake;
  private long timeUntilLowered;

  //private NetworkTableEntry speedDisplay = tab.add("Intake Motor Speed: ", 0).getEntry();
  private final NetworkTableEntry intakeRotationMotorPositionEntry, intakeMotorSpeedEntry, intakeRotationMotorSpeedEntry, intakeUpperLimitSwitchEntry, intakeLowerLimitSwitchEntry;
  
  //private NetworkTableEntry intakeRotationSpeedDisplay = tab.add("Intake Rotation Motor Speed: ", 0).getEntry();
  private RelativeEncoder encoder = intakeRotationMotor.getEncoder();

  /** Creates a new Intake. */
  public Intake() {
    //intakeMotor.restoreFactoryDefaults();
    intakeRotationMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(true);
    intakeRotationMotor.setInverted(true);

    intakeRotationMotor.setIdleMode(IdleMode.kCoast);
    //intakeMotor.setIdleMode(IdleMode.kCoast);


    gravityLoweringIntake = false;
    timeUntilLowered = 0;
    intakeRotationMotorPositionEntry = ShuffleboardInfo.getInstance().getIntakeRotationMotorPosition();
    intakeMotorSpeedEntry = ShuffleboardInfo.getInstance().getIntakeMotorSpeed();
    intakeRotationMotorSpeedEntry = ShuffleboardInfo.getInstance().getIntakeRotationMotorSpeed();
    intakeUpperLimitSwitchEntry = ShuffleboardInfo.getInstance().getIntakeUpperLimitSwitch();
    intakeLowerLimitSwitchEntry = ShuffleboardInfo.getInstance().getIntakeLowerLimitSwich();

    upperLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_UPPER_PORT);
    lowerLimitSwitch = new DigitalInput(Constants.INTAKE_LIMIT_LOWER_PORT);
    encoder.setPosition(0);

    
  }
  /**sets the speed for the intake motor not the intake rotation motor */
  private void setSpeedIntake(double speed){//for intake motor
    if(!isIntakeRotationMotorRaised() && isIntakeRasing()){
      intakeMotor.set(speed);
    }
  }
  public void startIntake(){
    setSpeedIntake(Constants.INTAKE_MOTOR_SPEED);
  }
  public void startIntakeReverse(){
    setSpeedIntake(-Constants.INTAKE_MOTOR_SPEED);
  }
  public void stopIntake(){
    setSpeedIntake(0.0);
  }
  /**gets the speed for the intake motor not the intake rotation motor */
  public double getSpeedIntake(){//for intake motor
    return intakeMotor.get();
  }

  public boolean isIntakeRotationMotorRaised(){
    return !upperLimitSwitch.get() || (getEncoderPosition() >= Constants.INTAKE_UPPER_ENCODER_VALUE);
  }

  public boolean isIntakeRotationMotorLowered(){
    return !lowerLimitSwitch.get() || (getEncoderPosition() <= Constants.INTAKE_LOWER_ENCODER_VALUE);
  }



  // public boolean isIntakeRotationMotorMoving(){
  //   return movingIntakeRotationMotor;
  // }

  public boolean isIntakeRasing(){
    return getSpeedIntake() == Constants.INTAKE_ROTATION_MOTOR_SPEED_UP;
  }

  public boolean isIntakeLowering(){
    return getSpeedIntake() == -Constants.INTAKE_ROTATION_MOTOR_SPEED_DOWN;
  }

  public boolean isIntakeGravityLowering(){
    return gravityLoweringIntake;
  }

  public void stopIntakeRotation(){
    intakeRotationMotor.set(0.0);
  }


  /** will raise the intake up within the robot */
  public void raiseIntake(){
    if(!isIntakeRotationMotorRaised() || isIntakeLowering()){
      intakeRotationMotor.setIdleMode(IdleMode.kBrake);

      stopIntake();

      //Raising the instake is a positive direction
      intakeRotationMotor.set(Constants.INTAKE_ROTATION_MOTOR_SPEED_UP);
    }
    

  }

  /** will lower the intake down to the ground, peramtor determines weather it will be a gravity deploy */
  public void lowerIntake(boolean gravity){
    if(isIntakeRotationMotorRaised() || isIntakeRasing()){
      intakeRotationMotor.setIdleMode(IdleMode.kCoast);
      //Lowering the intake is a negative direction
      intakeRotationMotor.set(-Constants.INTAKE_ROTATION_MOTOR_SPEED_DOWN);
      if(gravity){
        gravityLoweringIntake = true;
        timeUntilLowered = System.currentTimeMillis() + Constants.INTAKE_GRAVITY_LOWER_TIME;
      }
    }

  }


  /**returns the encoder position for the intake rotation motor except for when testing */
  private double getEncoderPosition(){
    //return fakeEncoderPosition;
    return encoder.getPosition();
  }
  /**intake is raised */
  /*private boolean isUpperLimitSwitchTriped(){
    //return false;
    return upperLimitSwitch.get();
  }

  private boolean isLowerLimitSwitchTriped(){
    //return false;
    return lowerLimitSwitch.get();
  }
*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeRotationMotorPositionEntry.setDouble(encoder.getPosition());
    intakeMotorSpeedEntry.setDouble(getSpeedIntake());
    intakeRotationMotorSpeedEntry.setDouble(encoder.getVelocity());
    intakeLowerLimitSwitchEntry.setBoolean(lowerLimitSwitch.get());
    intakeUpperLimitSwitchEntry.setBoolean(upperLimitSwitch.get());

    //intakeRotationSpeedDisplay.setDouble(intakeRotationMotor.get());//updates the display for the intake rotation motor speed
    //manage raise and lower intake
    //raise intake
    if(isIntakeRasing()){
      
      if(isIntakeRotationMotorRaised()){

        stopIntakeRotation();
      }
      
    }
    else if(isIntakeLowering()){
      
      if(isIntakeRotationMotorLowered() && !gravityLoweringIntake){


        intakeRotationMotor.set(0.0);

        
      }
    }
    else if(gravityLoweringIntake){
      if(timeUntilLowered <= System.currentTimeMillis()){
        stopIntakeRotation();
        timeUntilLowered = 0;
      }
      if(isIntakeRotationMotorLowered()){
        gravityLoweringIntake = false;


      }
    }

    if(isIntakeRotationMotorLowered() && encoder.getPosition() > Constants.INTAKE_LOWER_ENCODER_VALUE){
      intakeRotationMotor.setIdleMode(IdleMode.kCoast);
      encoder.setPosition(Constants.INTAKE_LOWER_ENCODER_VALUE);
    }
    //used for testing purposes
    // if(raisingIntake){
    //   fakeEncoderPosition -= 0.05;
    // }
    // if(loweringIntake){
    //   fakeEncoderPosition += 0.05;
    // }
  }
}