// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
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
  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);
  private final XboxController shooterController = new XboxController(Constants.SHOOTER_CONTROLLER_PORT);
  private final Drivetrain drivetrain = new Drivetrain();
  private final AutoCommand autoCommand = new AutoCommand();
  private final Intake intake = new Intake();
  private final Tower tower = new Tower();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final ShuffleboardLayout commandLayout;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
   drivetrain.setDefaultCommand(getTeleOpDrive());
   commandLayout = ShuffleboardInfo.getInstance().getCommandLayout();
   commandLayout.add(new RaiseBottomClimber(climber));
   commandLayout.add(new RaiseTopClimber(climber));
   commandLayout.add(new ReverseTower(tower));
   commandLayout.add(new RunTower(tower));
   commandLayout.add(new LoadTower(tower));
   commandLayout.add(new SpinShooter(shooter));
   commandLayout.add(new ShootBolls(shooter, tower));
   commandLayout.add(new ToggleIntakeRotation(intake));
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

     JoystickButton speedButton = new JoystickButton(driverController, Constants.SPEED_ADJUSTOR_TRIGGER);
     speedButton.whenHeld(new SpeedAdjustor(drivetrain));

     JoystickButton climberTopRaiseButton = new JoystickButton(driverController, Constants.CLIMBER_TOP_RAISE_BUTTON);
     JoystickButton climberTopLowerButton = new JoystickButton(driverController, Constants.CLIMBER_TOP_LOWER_BUTTON);
     JoystickButton climberBottomRaiseButton = new JoystickButton(driverController, Constants.CLIMBER_BOTTOM_RAISE_BUTTON);
      JoystickButton climberBottomLowerButton = new JoystickButton(driverController, Constants.CLIMBER_BOTTOM_LOWER_BUTTON);

     climberTopRaiseButton.whenHeld(new RaiseTopClimber(climber));
     climberTopLowerButton.whenHeld(new LowerTopClimber(climber));
     climberBottomRaiseButton.whenHeld(new RaiseBottomClimber(climber));
     climberBottomLowerButton.whenHeld(new LowerBottomClimber(climber));

     JoystickButton collectBallsButton = new JoystickButton(shooterController, Constants.COLLECT_BALLS_BUTTON);
     JoystickButton stopCollectBallsButton = new JoystickButton(shooterController, Constants.STOP_COLLECT_BALLS_BUTTON);
     JoystickButton runTowerManualButton = new JoystickButton(shooterController, Constants.RUN_TOWER_BUTTON);
     JoystickButton reverseTowerButton = new JoystickButton(shooterController, Constants.REVERSE_TOWER_BUTTON);
     JoystickButton shootBallButton = new JoystickButton(shooterController, Constants.SHOOT_BOLL_BUTTON);

     collectBallsButton.whenPressed(new LoadBalls(intake, tower));
     stopCollectBallsButton.whenPressed(new InstantCommand(tower::stop,tower).andThen(new InstantCommand(() -> intake.setSpeedIntake(0),intake)));
     runTowerManualButton.whenHeld(new RunTower(tower));
     reverseTowerButton.whenHeld(new ReverseTower(tower));
     shootBallButton.whenHeld(new ShootBolls(shooter, tower));
     shootBallButton.whenReleased(new InstantCommand (() -> tower.setSpeedUpper(0),tower));    
  }

public Command getTeleOpDrive(){
    return new RunCommand(() -> drivetrain.drive(driverController.getLeftY(),driverController.getLeftX(),driverController.getRightX()), drivetrain);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoCommand;
  }
}
