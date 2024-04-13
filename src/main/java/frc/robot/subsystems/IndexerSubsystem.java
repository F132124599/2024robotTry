// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {
  /** Creates a new indexerSubsystem. */
  private final CANSparkMax indexerMotor;
  public IndexerSubsystem() {
    indexerMotor = new CANSparkMax(IndexerConstants.indexerMotor_ID, MotorType.kBrushless);

    indexerMotor.restoreFactoryDefaults();

    indexerMotor.setIdleMode(IdleMode.kBrake);

    indexerMotor.setInverted(true);

    indexerMotor.burnFlash();

  }

  public void intakeNote() {
    indexerMotor.setVoltage(IndexerConstants.indexerVoltage);
  }

  public void feedNote() {
    indexerMotor.setVoltage(IndexerConstants.indexerVoltage);
  }
  
  public void stopIndexer() {
    indexerMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
