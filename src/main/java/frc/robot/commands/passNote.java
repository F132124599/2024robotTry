// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class passNote extends Command {
  /** Creates a new passNote. */
  private final ShooterSubsystem m_shooterSubsystem;
  
  private final IndexerSubsystem m_indexerSubsystem;
  public passNote(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_shooterSubsystem = shooterSubsystem;
    this.m_indexerSubsystem = indexerSubsystem;

    addRequirements(m_shooterSubsystem, m_indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooterSubsystem.passNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterSubsystem.passNote();
    if(Constants.ifFeed && m_shooterSubsystem.passNoteSpeedArrive()) {
      m_indexerSubsystem.feedNote();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterSubsystem.stopShoot();
    m_indexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
