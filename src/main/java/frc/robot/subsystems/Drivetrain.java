// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.*;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
/*





































CHECK ROBOT GEAR RATIO WITH HOGANS
*/

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax leftBackMotor = new CANSparkMax(Constants.LEFT_BACK_WHEEL_PORT,MotorType.kBrushless);
  private final CANSparkMax rightBackMotor = new CANSparkMax(Constants.RIGHT_BACK_WHEEL_PORT,MotorType.kBrushless);
  private double speedModulator = 1.0;
  private final MecanumDrive drivetrain;
  private final AHRS gyro;
  private final RelativeEncoder leftBackEncoder = leftBackMotor.getEncoder();
  private final RelativeEncoder leftFrontEncoder= leftFrontMotor.getEncoder();
  private final RelativeEncoder rightBackEncoder = rightBackMotor.getEncoder();
  private final RelativeEncoder rightFrontEncoder = rightFrontMotor.getEncoder();
  private final ShuffleboardTab debugDrivetrain = Shuffleboard.getTab("debugDrivetrain");
  private final MecanumDriveKinematics kinematics = new MecanumDriveKinematics(new Translation2d(Units.inchesToMeters(10.375),Units.inchesToMeters(10.375)), new Translation2d(Units.inchesToMeters(10.375),Units.inchesToMeters(-10.375)), new Translation2d(Units.inchesToMeters(-10.375),Units.inchesToMeters(10.375)), new Translation2d(Units.inchesToMeters(-10.375),Units.inchesToMeters(-10.375)));
  private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, getAngle2d());
  private Pose2d pose;
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.KS, Constants.KV, Constants.KA);
  public final TrapezoidProfile.Constraints anglePIDProfile = new TrapezoidProfile.Constraints(Constants.MAX_ROTATION,Constants.MAX_ROTATION_CHANGE );
  private final PIDController leftFrontPID = new PIDController(0, 0, 0);
  private final PIDController rightFrontPID = new PIDController(0, 0, 0);
  private final PIDController leftBackPID = new PIDController(0, 0, 0);
  private final PIDController rightBackPID = new PIDController(0, 0, 0);
  private final PIDController xPID = new PIDController(0, 0, 0);
  private final PIDController yPID = new PIDController(0, 0, 0);
  private final ProfiledPIDController thetaPID = new ProfiledPIDController(0, 0, 0, anglePIDProfile);
  //private final NetworkTableEntry gyroAngle = 

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    leftFrontMotor.setInverted(true);
    leftBackMotor.setInverted(true);
    gyro = new AHRS(Port.kMXP);
    drivetrain = new MecanumDrive(leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor);
    debugDrivetrain.add(drivetrain);
    debugDrivetrain.add(gyro);
    thetaPID.reset(0, 0);
  }
  public void drive(double xSpeed, double ySpeed, double zRotation){
    drivetrain.driveCartesian(ySpeed, xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Speed Modulator", speedModulator);
    pose = odometry.update(getAngle2d(), getSpeed());

  }
  public Pose2d getPose(){
    return pose;
  }

  public PIDController getLeftFrontPID (){
    return leftFrontPID;
  }

  public PIDController getRightFrontPID (){
    return rightFrontPID;
  }

  public PIDController getLeftBackPID (){
    return leftBackPID;
  }

  public PIDController getRightBackPID (){
    return rightBackPID;
  }

  public MecanumDriveWheelSpeeds getSpeed(){
    return new MecanumDriveWheelSpeeds(
    Constants.WHEEL_GEAR_RATIO*leftFrontEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60, 
    Constants.WHEEL_GEAR_RATIO*rightFrontEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60, 
    Constants.WHEEL_GEAR_RATIO*rightBackEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60, 
    Constants.WHEEL_GEAR_RATIO*leftBackEncoder.getVelocity()*(Math.PI*2*Units.inchesToMeters(Constants.WHEEL_RADIUS))/60
    );
  }

  public void setMotorVoltages(MecanumDriveMotorVoltages volts){
    leftFrontMotor.setVoltage(volts.frontLeftVoltage);
    rightFrontMotor.setVoltage(volts.frontRightVoltage);
    leftBackMotor.setVoltage(volts.rearLeftVoltage);
    rightBackMotor.setVoltage(volts.rearRightVoltage);
  }

  public void resetOdometry(Pose2d pose){
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public double getSpeedModulator() {
    return speedModulator;
  }
  public void setSpeedModulator(double speedModulator) {
      this.speedModulator = speedModulator;
  }
  public void resetGyro(){
    gyro.reset();
  }
  public double getAngle(){
    return gyro.getAngle();
  }
  
  private Rotation2d getAngle2d(){
    return Rotation2d.fromDegrees(-getAngle());
  }

  public void resetEncoders(){
    leftFrontEncoder.setPosition(0);
    leftBackEncoder.setPosition(0);
    rightFrontEncoder.setPosition(0);
    rightBackEncoder.setPosition(0);
  }
  
  public double getLeftFrontEncoder(){
    return leftFrontEncoder.getPosition();
  }
  public double getLeftBackEncoder(){
    return leftBackEncoder.getPosition();
  }

  public double getRightBackEncoder(){
    return rightBackEncoder.getPosition();
  }
  public double getRightFrontEncoder(){
    return rightFrontEncoder.getPosition();
  }

  public SimpleMotorFeedforward getFeedforward(){
    return feedforward;
  }

  public MecanumDriveKinematics getKinematics(){
    return kinematics;
  }
  public PIDController getXPID() {
      return xPID;
  }
  public PIDController getYPID() {
      return yPID;
  }
  public ProfiledPIDController getThetaPID() {
      return thetaPID;
  }
  

}
