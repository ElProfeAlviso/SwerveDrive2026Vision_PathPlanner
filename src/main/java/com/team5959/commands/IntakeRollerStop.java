// the WPILib BSD license file in the root directory of this project.

package com.team5959.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.team5959.subsystems.IntakeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeRollerStop extends InstantCommand {
  IntakeSubsystem intakeSubsystem;

  public IntakeRollerStop(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.stopRollerMotor(); // Ajusta el valor según la posición deseada

  }
}
