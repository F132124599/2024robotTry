// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class NoteIntake extends Command {
  /** Creates a new NoteIntake. */
  private final IntakeSubsystem intakeSubsystem;

  private final IndexerSubsystem indexerSubsystem;

  public NoteIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
    intakeSubsystem = new IntakeSubsystem();
    indexerSubsystem = new IndexerSubsystem();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.noteIntake();
    indexerSubsystem.intakeNote();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopIntake();
    indexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
