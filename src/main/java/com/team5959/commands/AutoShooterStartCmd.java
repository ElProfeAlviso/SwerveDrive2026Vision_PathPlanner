// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import com.team5959.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoShooterStartCmd extends Command {

  private final ShooterSubsystem shooter;
  private final Timer timer = new Timer();
  private final double durationSeconds;

  /** Creates a new AutoShooterStartCmd. */
  public AutoShooterStartCmd(ShooterSubsystem shooter, double seconds) {
    this.shooter = shooter;
    this.durationSeconds = seconds;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    shooter.setShooterPIDSpeed(-4000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterFeederSpeed(-0.8);
    shooter.setShooterIndexerSpeed(-0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();

    shooter.stopFeeder();
    shooter.stopIndexer();
    shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(durationSeconds);
  }
}
