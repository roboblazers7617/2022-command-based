// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController; 
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.Automations.AdjustTower;
import frc.robot.commands.Automations.LoadBalls;
import frc.robot.commands.Automations.ShootBolls;
import frc.robot.commands.Automations.ShootOneBoll;
import frc.robot.commands.Autonomous.AutoCommand;
import frc.robot.commands.Autonomous.AutoEasy;
import frc.robot.commands.Autonomous.AutoEasyClose;
import frc.robot.commands.Autonomous.DriveWithEncoders;
import frc.robot.commands.Autonomous.PathPlannerFollowTrajectory;
import frc.robot.commands.Autonomous.RobotGoDiagy;
import frc.robot.commands.Autonomous.TestEncoders;
import frc.robot.commands.Autonomous.ThreeBallAutoLeft;
import frc.robot.commands.Autonomous.TwoBallAutoLeft;
import frc.robot.commands.Autonomous.TwoBallAutoRight;
import frc.robot.commands.Autonomous.TwoBallOdoAuto;
import frc.robot.commands.Climber.LowerTopClimber;
import frc.robot.commands.Climber.RaiseTopClimber;
import frc.robot.commands.Intake.ActivateIntake;
import frc.robot.commands.Intake.DeployIntake;
import frc.robot.commands.Intake.ResetIntake;
import frc.robot.commands.Intake.ReverseIntake;
import frc.robot.commands.Intake.RunIntake;
import frc.robot.commands.Intake.ToggleIntakeRotation;
import frc.robot.commands.Intake.ZeroOutIntake;
import frc.robot.commands.Intake.ZeroOutIntakeCancel;
import frc.robot.commands.Shooter.SpinShooter;
import frc.robot.commands.Shooter.StopShooter;
import frc.robot.commands.Tower.LoadOneTower;
import frc.robot.commands.Tower.LoadTower;
import frc.robot.commands.Tower.StopTower;
import frc.robot.commands.Tower.moveTowerIndividual;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private static final XboxController shooterController = new XboxController(Constants.SHOOTER_CONTROLLER_PORT);
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final Intake intake = new Intake();
  private static final Tower tower = new Tower();
  private static final Shooter shooter = new Shooter();
  private static final Climber climber = new Climber();
  private final ShuffleboardLayout commandLayout;
  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
   drivetrain.setDefaultCommand(getTeleOpDrive());
   commandLayout = ShuffleboardInfo.getInstance().getCommandLayout();
  // commandLayout.add(new RaiseBottomClimber(climber));
   commandLayout.add(new RaiseTopClimber(climber));
   //commandLayout.add(new LowerBottomClimber(climber));
   commandLayout.add(new LowerTopClimber(climber));
   commandLayout.add(new LoadTower(tower));
   commandLayout.add(new SpinShooter(shooter));
   commandLayout.add(new ShootBolls(shooter, tower));
   commandLayout.add(new ToggleIntakeRotation(intake));
   commandLayout.add(new StopShooter(shooter));
   commandLayout.add(new DeployIntake(intake));
   commandLayout.add(new ResetIntake(intake));
   commandLayout.add(new RunIntake(intake));
 //  tower.setDefaultCommand( new RunCommand(tower::stop, tower));  
   //intake.setDefaultCommand(new ResetIntakeForever(intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    autoChooser.setDefaultOption("Close Auto", new AutoEasyClose(drivetrain, shooter, tower));
    autoChooser.addOption("Farther Auto", new AutoEasy(drivetrain, shooter, tower));
    autoChooser.addOption("Do Nothing", new AutoCommand());
    autoChooser.addOption("Odo Left", new TwoBallOdoAuto(intake, tower, shooter, drivetrain));
    autoChooser.addOption("Two Ball Left", new TwoBallAutoLeft(tower, drivetrain, shooter, intake));
    autoChooser.addOption("Three Ball Left", new ThreeBallAutoLeft(tower, drivetrain, shooter, intake));
    autoChooser.addOption("Two Ball Right", new TwoBallAutoRight(tower, drivetrain, shooter, intake));
    autoChooser.addOption("Simple Auto Test", new PathPlannerFollowTrajectory(drivetrain, "TestAuto", Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION));
    autoChooser.addOption("Diagonal Test", new RobotGoDiagy(drivetrain, 30, Constants.LEFT));
    SmartDashboard.putData(autoChooser);

     JoystickButton speedButton = new JoystickButton(driverController, Constants.SPEED_ADJUSTOR_TRIGGER);
     speedButton.whenPressed(new InstantCommand(()-> drivetrain.setMaxSpeed(Constants.LOW_GEAR)));
     speedButton.whenReleased(new InstantCommand(()-> drivetrain.setMaxSpeed(Constants.SUPER_HIGH_GEAR)));


    JoystickButton highSpeedButton = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    highSpeedButton.whenPressed(new InstantCommand(()-> drivetrain.setMaxSpeed(Constants.SUPER_HIGH_GEAR)));
    highSpeedButton.whenReleased(new InstantCommand(()-> drivetrain.setMaxSpeed(Constants.HIGH_GEAR)));

     JoystickButton climberTopFowardButton = new JoystickButton(driverController, Constants.CLIMBER_TOP_FOWARD_BUTTON);
     JoystickButton climberTopBackwordButton = new JoystickButton(driverController, Constants.CLIMBER_TOP_BACKWARD_BUTTON);
  //   JoystickButton climberBottomFowardButton = new JoystickButton(driverController, Constants.CLIMBER_BOTTOM_FORWARD_BUTTON);
  //    JoystickButton climberBottomBackwardButton = new JoystickButton(driverController, Constants.CLIMBER_BOTTOM_BACKWARD_BUTTON);

     climberTopFowardButton.whenHeld(new RaiseTopClimber(climber));
     climberTopBackwordButton.whenHeld(new LowerTopClimber(climber));
//     climberBottomFowardButton.whenHeld(new RaiseBottomClimber(climber));
 //  climberBottomBackwardButton.whenHeld(new LowerBottomClimber(climber));

    JoystickButton intakeZeroOutButton = new JoystickButton(driverController, Constants.INTAKE_ZERO_OUT_BUTTON);
    JoystickButton intakeZeroOutCancelButton = new JoystickButton(driverController, Constants.INTAKE_ZERO_OUT_CANCEL_BUTTON);

    intakeZeroOutButton.whenPressed(new ZeroOutIntake(intake));
    intakeZeroOutCancelButton.whenPressed(new ZeroOutIntakeCancel(intake));






     JoystickButton collectBallsButton = new JoystickButton(shooterController, Constants.COLLECT_BALLS_BUTTON);
     JoystickButton stopCollectBallsButton = new JoystickButton(shooterController, Constants.STOP_COLLECT_BALLS_BUTTON);
     JoystickButton shootBallButton = new JoystickButton(shooterController, Constants.SHOOT_BOLL_BUTTON);
     JoystickButton reverseIntakeButton = new JoystickButton(shooterController, Constants.REVERSE_INTAKE_BUTTON);
     JoystickButton activateIntakeButton = new JoystickButton(shooterController, Constants.ACTIVATE_INTAKE_BUTTON);
     JoystickButton resetIntakeButton = new JoystickButton(shooterController, Constants.RESET_INTAKE_BUTTON);
     JoystickButton moveUpBallsButton = new JoystickButton(shooterController, XboxController.Button.kRightStick.value);
     JoystickButton moveUpBallButton = new JoystickButton(shooterController, XboxController.Button.kLeftStick.value);

    
     collectBallsButton.whenPressed(new LoadBalls(intake, tower));
     stopCollectBallsButton.whenPressed(new InstantCommand(tower::stop,tower).andThen(new ResetIntake(intake)));
     shootBallButton.whenPressed(new ShootBolls(shooter, tower));
     //shootBallButton.whenReleased(new InstantCommand (() -> tower.stop()).andThen(new StopShooter(shooter)));
     reverseIntakeButton.whenHeld(new ReverseIntake(intake));
     activateIntakeButton.whenHeld(new ActivateIntake(intake));
     resetIntakeButton.whenPressed(new ResetIntake(intake));

     moveUpBallsButton.whenPressed(new AdjustTower(tower,shooter));
     moveUpBallButton.whenPressed(new LoadOneTower(tower));

     Trigger leftTriggerButton = new Trigger(() -> shooterController.getLeftTriggerAxis() >= 0.5);
     leftTriggerButton.whenActive(new DeployIntake(intake));

     Trigger rightTriggerButton = new Trigger(() -> shooterController.getRightTriggerAxis() >= 0.5);
     //rightTriggerButton.whenActive(new GravityIntakeDeploy(intake));
    rightTriggerButton.whenActive(new ShootOneBoll(shooter, tower));

     Trigger moveTowerIndividualJoysticks = new Trigger(() -> Math.abs(shooterController.getLeftY()) >.2 || Math.abs(shooterController.getRightY()) >.2);
     moveTowerIndividualJoysticks.whenActive(new moveTowerIndividual(tower, shooterController::getRightY, shooterController::getLeftY));

     Trigger dPadLeft = new Trigger(() ->shooterController.getPOV() == 90);
     dPadLeft.whenActive(new InstantCommand(() -> shooter.setSetPoint(Constants.SHOOTER_SPEED)));

     Trigger dPadRight = new Trigger(() ->shooterController.getPOV() == 270);
     dPadRight.whenActive((new StopShooter(shooter)).andThen(new StopTower(tower)));

     Trigger dPadUp = new Trigger(() ->shooterController.getPOV() == 0);
     dPadUp.whenActive(new InstantCommand(() -> shooter.setSetPoint(Constants.FAST_SHOOTER_SPEED)));

     Trigger dPadDown = new Trigger(() ->shooterController.getPOV() == 180);
     dPadDown.whenActive(new InstantCommand(() -> shooter.setSetPoint(Constants.SLOW_SHOOTER_SPEED)));
  }

public Command getTeleOpDrive(){
    return new RunCommand(() -> drivetrain.drive(driverController.getLeftY(),-driverController.getLeftX(),-driverController.getRightX()), drivetrain);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  } 



  public static Command followTrajectory(String trajectoryName) {
    // An ExampleCommand will run in autonomous
    TrajectoryConfig tConfig = new TrajectoryConfig(Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION);
    tConfig.setKinematics(drivetrain.getKinematics());
    PathPlannerTrajectory  trajectory = PathPlanner.loadPath(trajectoryName, Constants.MAX_VELOCITY, Constants.MAX_ACCELERATION);
   /* MecanumControllerCommand trajectoryFollow = new MecanumControllerCommand(trajectory, 
    drivetrain :: getPose, 
    drivetrain.getFeedforward(),
    drivetrain.getKinematics(), 
    drivetrain.getXPID(), 
    drivetrain.getYPID(), 
    drivetrain.getThetaPID(), 
    Units.feetToMeters(Constants.MAX_VELOCITY),
    drivetrain.getLeftFrontPID(),
    drivetrain.getLeftBackPID(),
    drivetrain.getRightFrontPID(),
    drivetrain.getRightBackPID(), 
    drivetrain :: getSpeed,
    drivetrain::setMotorVoltages, 
    drivetrain);
    drivetrain.resetOdometry(trajectory.getInitialPose());
    return trajectoryFollow.andThen(() -> drivetrain.drive(0, 0, 0));*/
    PPMecanumControllerCommand trajectoryFollow = new PPMecanumControllerCommand(
      trajectory, 
      drivetrain::getPose,
      drivetrain.getKinematics(),
      drivetrain.getXPID(),
      drivetrain.getYPID(),
      drivetrain.getThetaPID(), 
      Constants.MAX_VELOCITY, 
      drivetrain::setSpeeds, 
      drivetrain);
      drivetrain.resetOdometry(trajectory.getInitialPose());
      return trajectoryFollow.andThen(() -> drivetrain.drive(0,0,0));
  }

  public void startTeleop(){
    drivetrain.setBrakeMode("coast");
  }
}
