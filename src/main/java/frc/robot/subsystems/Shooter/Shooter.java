// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drive.CommandSwerveDrivetrain;

public class Shooter extends SubsystemBase {
  private TalonFX shooterMotor;
  private TalonFXConfiguration shooterConfig;

  private MotionMagicVelocityVoltage m_motionRequest;
  private VoltageOut m_voltageRequest;

  private ShooterState currentState = ShooterState.IDLE;
  private boolean autoGoalEnabled = false;

  private CommandSwerveDrivetrain m_swerveSubsystem;

  public Shooter() {
    shooterMotor = new TalonFX(ShooterConstants.kMotorId);

    shooterConfig = new TalonFXConfiguration()
                    .withMotorOutput(new MotorOutputConfigs()
                                    .withInverted(InvertedValue.Clockwise_Positive)
                                    .withNeutralMode(NeutralModeValue.Brake))
                    .withSlot0(new Slot0Configs()
                              .withKP(ShooterConstants.kP)
                              .withKI(ShooterConstants.kI)
                              .withKD(ShooterConstants.kD))
                    .withMotionMagic(new MotionMagicConfigs()
                                    .withMotionMagicCruiseVelocity(ShooterConstants.kCruiseVelocity)
                                    .withMotionMagicAcceleration(ShooterConstants.kAcceleration)
                                    .withMotionMagicJerk(ShooterConstants.kJerk))
                    .withFeedback(new FeedbackConfigs()
                                  .withSensorToMechanismRatio(ShooterConstants.kSensorToMechRatio)
                                  .withRotorToSensorRatio(ShooterConstants.kRotorToSensorRatio))
                    .withCurrentLimits(new CurrentLimitsConfigs()
                                    .withSupplyCurrentLimit(ShooterConstants.kSupplyCurrentLimit));
    shooterMotor.getConfigurator().apply(shooterConfig);

    m_voltageRequest = new VoltageOut(0);

    m_motionRequest = new MotionMagicVelocityVoltage(0).withSlot(0).withFeedForward(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();
  }

  public void setShooterVelocity() {
    m_motionRequest.Velocity = 1;
  }

  private void logMotorData() {
    DogLog.log("Subsystem/Shooter/ShooterState", currentState.name());

    DogLog.log("Subsystem/Shooter/ShooterMotorVelocity", shooterMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystem/Shooter/ShooterMotorSupplyCurrent", shooterMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystem/Shooter/ShooterMotorStatorCurrent", shooterMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystem/Shooter/ShooterMotorVoltage", shooterMotor.getMotorVoltage().getValueAsDouble());
  }
}