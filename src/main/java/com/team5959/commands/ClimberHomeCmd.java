package com.team5959.commands;

import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.subsystems.ClimberSubsystem;

/**
 * Lowers the climber until the lower limit switch is triggered, then stops and resets encoders.
 */
public class ClimberHomeCmd extends Command {

  private final ClimberSubsystem climber;
  // Speed to lower the climber (negative to move down). Adjust as needed.
  private final double lowerSpeed = -0.2;
  // Safety timeout in milliseconds to avoid running forever if switch fails
  private final long timeoutMs = 3000;
  private long startTime = 0;

  

  public ClimberHomeCmd(ClimberSubsystem climber) {
    this.climber = climber;
    addRequirements(climber);
  }

  @Override
  public void initialize() {
    // Ensure PID is disabled so manual control drives the motor
    climber.setEnableClimberPID(false);
    // Temporarily disable soft limits so we can reach the limit switch if
    // the configured reverse soft limit would block the motion.
    climber.setSoftLimitsEnabled(false);
    startTime = System.currentTimeMillis();
  }

  @Override
  public void execute() {
    // Drive down slowly until the switch is hit
    climber.setClimberManualPosition(lowerSpeed);
  }

  @Override
  public boolean isFinished() {
    // Finish when the lower limit switch is pressed or when timeout expires
    boolean switchPressed = climber.isLowerLimitSwitchPressed();
    boolean timedOut = (System.currentTimeMillis() - startTime) > timeoutMs;
    return switchPressed || timedOut;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop motor and reset encoders to establish home position
    climber.ClimberStopMotor();
    climber.resetEncoderPosition();
    // Re-enable soft limits after homing
    climber.setSoftLimitsEnabled(true);
    // Optionally, re-enable PID or hold at zero position if desired
    climber.setEnableClimberPID(false);
  }
}
