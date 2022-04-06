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

//*************************** */
/* DOESN'T WORK PROPERLY!!!!!!!!!!!!!!!!!!!!! ******/
/**************************** */
public class ThreeBallAutoLeft extends SequentialCommandGroup {
  /** Creates a new TwoBallAutoLeft. */
  public ThreeBallAutoLeft(Tower tower, Drivetrain drivetrain, Shooter shooter, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> drivetrain.resetGyro()),
      new InstantCommand(() -> drivetrain.resetEncoders()),
      new InstantCommand(() -> drivetrain.setBrakeMode("Brake")),
      new ShootOneBoll(shooter, tower),
      //new DriveWithEncoders(drivetrain, 32),
      new GyroGoStraightSimple(drivetrain, 32,.5),
      new ParallelDeadlineGroup(
          new LoadBallsAuto(intake, tower),
 //         new SequentialCommandGroup(new DriveWithEncoders(drivetrain, 38), new DriveWithEncoders(drivetrain, -25), 
          new SequentialCommandGroup( 
              new StrafeWithTime(drivetrain, Constants.AUTO_SPEED, 0.65), 

               new GyroGoStraightSimple(drivetrain,  42, .5), new GyroGoStraightSimple(drivetrain, -48,.5), 
//   OLDEST         new StrafeWithTime(drivetrain, -Constants.AUTO_SPEED, 3.9), new DriveWithEncoders(drivetrain, 10))) ,
              new TurnToAngle(drivetrain, -8),
              new StrafeWithTime(drivetrain, -Constants.AUTO_SPEED, 2.32), new GyroGoStraightSimple(drivetrain, 35, .5))
              ) ,
        //      new ParallelCommandGroup(new DriveWithEncoders(drivetrain, -14)/*, new ResetIntake(intake)*/),
      new ParallelCommandGroup(new GyroGoStraightSimple(drivetrain, -14, .5), new ResetIntake(intake)),
      new StrafeWithTime(drivetrain, Constants.AUTO_SPEED, 1.9),
//      new DriveWithEncoders(drivetrain, -40),
      new GyroGoStraightSimple(drivetrain, -46, .5),
      new ShootBolls(shooter, tower),
      new InstantCommand(() -> drivetrain.setBrakeMode("Coast"))
    );
  }
}
