// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubaystem extends SubsystemBase {
  /** Creates a new climberSubaystem. */
  private final CANSparkMax leftClimberMotor;
  private final CANSparkMax rightClimberMotor;
  

  /*
   * 要加極限開關
   */
  public ClimberSubaystem() {
    leftClimberMotor = new CANSparkMax(ClimberConstants.leftClimberMotor_ID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(ClimberConstants.rightClimberMotor_ID, MotorType.kBrushless);

    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    leftClimberMotor.setIdleMode(IdleMode.kBrake);
    rightClimberMotor.setIdleMode(IdleMode.kBrake);

    //記得到時候要測方向
    leftClimberMotor.setInverted(true);
    rightClimberMotor.setInverted(true);

    leftClimberMotor.burnFlash();
    rightClimberMotor.burnFlash();
  }

  public void climbUp() {
    leftClimberMotor.setVoltage(ClimberConstants.climbVoltage);
    rightClimberMotor.setVoltage(ClimberConstants.climbVoltage);
  }

  public void climberDown() {
    leftClimberMotor.setVoltage(-ClimberConstants.climbVoltage);
    rightClimberMotor.setVoltage(-ClimberConstants.climbVoltage);
  }

  public void stopClimb() {
    leftClimberMotor.setVoltage(0);
    rightClimberMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
