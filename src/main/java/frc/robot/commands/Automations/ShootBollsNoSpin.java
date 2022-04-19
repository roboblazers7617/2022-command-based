// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Automations;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.Shooter.SpinShooter;
import frc.robot.commands.Tower.StopTower;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootBollsNoSpin extends SequentialCommandGroup {
  /** Creates a new ShootBolls. */
  public ShootBollsNoSpin(Shooter shooter, Tower tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    new ConditionalCommand(
      new SequentialCommandGroup(
        new WaitUntilCommand(shooter::shooterReady),
        new InstantCommand(() ->tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED+.17),tower),
        new WaitUntilCommand(shooter::getShooterSensor),
        new InstantCommand(() ->tower.setSpeedLower(Constants.LOWER_TOWER_SPEED),tower),
        new WaitCommand(.5),  // Possible fix for beam break issue   
        new WaitUntilCommand(() -> !tower.upperSensor.get()),
        new InstantCommand(() -> tower.stop()),
        new WaitUntilCommand(shooter::shooterReady),
        new InstantCommand(() ->tower.setSpeedUpper(Constants.UPPER_TOWER_SPEED+.17),tower),
        new WaitCommand(.5),
        new InstantCommand(shooter::stopShooter, shooter),
        new InstantCommand(tower::stop, tower)
      ), 
      new WaitCommand(0), 
      shooter::isShooterSpinning);
  }
}
