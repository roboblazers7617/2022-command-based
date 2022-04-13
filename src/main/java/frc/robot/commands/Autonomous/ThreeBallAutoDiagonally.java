// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Automations.LoadBallsAuto;
import frc.robot.commands.Automations.ShootBolls;
import frc.robot.commands.Automations.ShootOneBoll;
import frc.robot.commands.Intake.ResetIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAutoDiagonally extends SequentialCommandGroup {

  // Multiplier for the forward/backward motion
  private double speedMultiplier = 0.7;  

  // Multiplier for the strafe
  private double strafeSpeedMultiplier = 1.1;

  public ThreeBallAutoDiagonally(Tower tower, Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.

    addCommands(

    // Initialize the gyro, encoders, and set the brake mode to Brake (rather than coast) so the robot does not slide around the field
      new InstantCommand(() -> drivetrain.resetGyro()),
      new InstantCommand(() -> drivetrain.resetEncoders()),
      new InstantCommand(() -> drivetrain.setBrakeMode("Brake")),

      // Shoot the preloaded ball
      new ShootOneBoll(shooter, tower),

      // Start driving toward the first ball on the left
      new DriveWithEncoders(drivetrain, 40, speedMultiplier),

      // Create a parallel deadline group. This will run LoadBallsAuto, and once the second ball is loaded will stop the motion
      // of the drivetrain toward the second ball and start moving toward the goal.
      new ParallelDeadlineGroup(
          new LoadBallsAuto(intake, tower),

          new SequentialCommandGroup( 
                new RobotGoDiagy(drivetrain, .85, Constants.LEFT, true), 
                new DriveWithEncoders(drivetrain, 10, speedMultiplier),

                // First ball has been grabbed. Start movement toward the second ball that is on the right
                new DriveWithEncoders(drivetrain, -10, speedMultiplier),
                new RobotGoDiagy(drivetrain, 2.1, Constants.RIGHT, false),
                new RobotGoDiagy(drivetrain, 1, Constants.RIGHT, true),
                // This DriveWithEncoders movement number can be large as the robot will stop moving forward once LoadBallsAuto completes
                new DriveWithEncoders(drivetrain, 28) 
          )  
      ),

      //Move toward the goal
      new ParallelCommandGroup (new RobotGoDiagy(drivetrain, 1.2, Constants.LEFT, false), new ResetIntake(intake)), 
      new StrafeWithTime(drivetrain, Constants.AUTO_SPEED*strafeSpeedMultiplier, .9),
      new DriveWithEncoders(drivetrain, -20),

      // Shoot the two balls and then set the brake mode to coast to be ready for teleop period
      new ShootBolls(shooter, tower),

      // Move the robot forward to ensure is taxied out of the tarmac
      new DriveWithEncoders(drivetrain, 30, .5), 
      new InstantCommand(() -> drivetrain.setBrakeMode("Coast"))
    );
  }}
