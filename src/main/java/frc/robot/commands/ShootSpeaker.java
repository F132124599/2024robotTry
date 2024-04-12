// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSpeaker extends Command {
  /** Creates a new ShootSpeaker. */
  private final ShooterSubsystem shooterSubsystem;

  private final IndexerSubsystem indexerSubsystem;
  public ShootSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooterSubsystem = new ShooterSubsystem();
    indexerSubsystem = new IndexerSubsystem();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.shootSpeaker();
    if(Constants.ifFeed == true && shooterSubsystem.SpeakerSpeedArrive())
    indexerSubsystem.feedNote();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShoot();
    indexerSubsystem.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
