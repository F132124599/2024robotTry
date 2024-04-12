// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new shooterSubsystem. */
  private final CANSparkMax shooterMotor;

  private final CANcoder absoluteEncoder;

  private final CANcoderConfiguration absoluteEncoderConfig;

  private final RelativeEncoder shooterMotorEncoder;
  public ShooterSubsystem() {
    shooterMotor = new CANSparkMax(shooterConstants.shooterMotor_ID, MotorType.kBrushless);

    absoluteEncoder = new CANcoder(shooterConstants.shooterMotor_ID);

    absoluteEncoderConfig = new CANcoderConfiguration();

    shooterMotorEncoder = shooterMotor.getEncoder();

    absoluteEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    absoluteEncoderConfig.MagnetSensor.MagnetOffset = shooterConstants.absoluteEncoderOffset;
    absoluteEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

    absoluteEncoder.getConfigurator().apply(absoluteEncoderConfig);
 
    shooterMotor.restoreFactoryDefaults();

    shooterMotor.setIdleMode(IdleMode.kBrake);

    shooterMotor.setInverted(true);

    shooterMotor.burnFlash();
  }

  public void shoot(){
    shooterMotor.setVoltage(shooterConstants.shootVoltage);
  }

  public void stopShoot(){
    shooterMotor.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
