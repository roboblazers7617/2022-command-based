// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Automations.LoadOneBall;
import frc.robot.commands.Automations.LoadOneBallAuto;
import frc.robot.commands.Automations.ShootOneBoll;
import frc.robot.commands.Intake.ResetIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAutoRight extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoLeft. */
  public TwoBallAutoRight(Tower tower, Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.resetGyro()),
      new InstantCommand(() -> drivetrain.resetEncoders()),
      new InstantCommand(() -> drivetrain.setBrakeMode("Brake")),
      new ShootOneBoll(shooter, tower),
      new DriveWithEncoders(drivetrain, 32, .5),
      new StrafeWithTime(drivetrain, -Constants.AUTO_SPEED, .65),
      new ParallelDeadlineGroup(new LoadOneBallAuto(intake, tower),new DriveWithEncoders(drivetrain, 46, .3)),
      new ParallelCommandGroup(new DriveWithEncoders(drivetrain, -38, .5), new ResetIntake(intake)),
      new TurnToAngle(drivetrain, 0),
      new StrafeWithTime(drivetrain, Constants.AUTO_SPEED, .62),
      new DriveWithEncoders(drivetrain, -40, .5),
      new ShootOneBoll(shooter, tower),
      new InstantCommand(() -> drivetrain.setBrakeMode("Coast")),
      new DriveWithEncoders(drivetrain, 40, .5)
    );
  }
}
