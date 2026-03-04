// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class setShooterManualSpeed extends Command {

  private final ShooterSubsystem shooter; // Subsystem that controls the shooter
  private final double shooterspeed; // Desired setpoint for the shooter
  private final double feederspeed; // Desired setpoint for the shooter
  
  
  

  /** Creates a new ShooterPID. */
  public setShooterManualSpeed(ShooterSubsystem shooterSubsystem, double shooterspeed, double feederspeed) {
    this.shooter = shooterSubsystem;
    this.shooterspeed = shooterspeed;
     this.feederspeed = feederspeed;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterManualSpeed(shooterspeed);
    shooter.setShooterFeederSpeed(feederspeed);
    shooter.setShooterIndexerSpeed(feederspeed*1.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
