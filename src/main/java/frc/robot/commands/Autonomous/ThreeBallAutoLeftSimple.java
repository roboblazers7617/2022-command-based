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
import frc.robot.commands.Automations.ShootBollsFast;
import frc.robot.commands.Automations.ShootOneBoll;
import frc.robot.commands.Intake.ResetIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallAutoLeftSimple extends SequentialCommandGroup {
  /** Creates a new ThreeBallAutoLeftSimple. */
  private double speedMultiplier = 0.7;
  private double strafeSpeedMultiplier = 1.1;

  public ThreeBallAutoLeftSimple(Tower tower, Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new InstantCommand(() -> drivetrain.resetGyro()),
      new InstantCommand(() -> drivetrain.resetEncoders()),
      new InstantCommand(() -> drivetrain.setBrakeMode("Brake")),
      new ShootOneBoll(shooter, tower),
      new DriveWithEncoders(drivetrain, 32, speedMultiplier),
      new ParallelDeadlineGroup(
          new LoadBallsAuto(intake, tower),
          new SequentialCommandGroup( 
              new StrafeWithTime(drivetrain, Constants.AUTO_SPEED*strafeSpeedMultiplier, 0.56), 
               new DriveWithEncoders(drivetrain,  43, speedMultiplier), new DriveWithEncoders(drivetrain, -48,speedMultiplier), 
              new TurnToAngle(drivetrain, -8),
              new StrafeWithTime(drivetrain, -Constants.AUTO_SPEED*strafeSpeedMultiplier, 2.3), new DriveWithEncoders(drivetrain, 50, speedMultiplier))
              ) ,
      new ParallelCommandGroup(new DriveWithEncoders(drivetrain, -14, speedMultiplier), new ResetIntake(intake)),
      new StrafeWithTime(drivetrain, Constants.AUTO_SPEED*strafeSpeedMultiplier, 1.82),
      new DriveWithEncoders(drivetrain, -43, speedMultiplier),
      new ShootBollsFast(shooter, tower),
      new InstantCommand(() -> drivetrain.setBrakeMode("Coast"))
    );
  }
}
