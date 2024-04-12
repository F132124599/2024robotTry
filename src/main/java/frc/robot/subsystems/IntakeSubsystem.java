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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  private final CANSparkMax intakeWheel;
  private final CANSparkMax intakeArm;

  private final PIDController armPID;

  private final RelativeEncoder armEncoder;
  private final CANcoder absoluteArmEncoder;

  private final CANcoderConfiguration canCoderConfig;

  private  double pidOutput;

  private  double arriveAngle;
  public IntakeSubsystem(int intakeWheelID, int intakeArmID, int absoluteArmEncoderID) {
    intakeWheel = new CANSparkMax(intakeWheelID, MotorType.kBrushless);
    intakeArm = new CANSparkMax(intakeArmID, MotorType.kBrushless);

    armPID = new PIDController(Constants.intakeArmPID_Kp, Constants.intakeArmPID_Ki, Constants.intakeArmPID_Kd);

    armEncoder = intakeArm.getEncoder();
    absoluteArmEncoder = new CANcoder(absoluteArmEncoderID);
    canCoderConfig = new CANcoderConfiguration();

    canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = Constants.intakeCancoderOffset;
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    absoluteArmEncoder.getConfigurator().apply(canCoderConfig);

    intakeWheel.restoreFactoryDefaults();
    intakeArm.restoreFactoryDefaults();

    intakeWheel.setIdleMode(IdleMode.kBrake);
    intakeArm.setIdleMode(IdleMode.kBrake);

    intakeWheel.setInverted(true);
    intakeArm.setInverted(true);

    intakeWheel.burnFlash();
    intakeArm.burnFlash();

  }

  public void noteintake(){
    intakeWheel.setVoltage(Constants.intakewheelVoltage);
    intakeArm.setVoltage(5);

  }

  public void setArriveAngle(double angle){
    arriveAngle = angle;
  }

  public void getAngle(){
    absoluteArmEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pidOutput = armPID.calculate(5);
    
  }
}
