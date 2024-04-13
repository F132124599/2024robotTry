// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubaystem;

public class ClimbDown extends Command {
  /** Creates a new climbDown. */
  private final ClimberSubaystem m_climberSubaystem;
  public ClimbDown(ClimberSubaystem climberSubaystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climberSubaystem = climberSubaystem;
    addRequirements(m_climberSubaystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climberSubaystem.climberDown();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climberSubaystem.stopClimb();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
