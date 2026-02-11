// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  private TalonFX spindexerMotor;
  private TalonFXConfiguration spindexerConfig;

  private TalonFX indexerMotor;
  private TalonFXConfiguration indexerConfig;

  private CANrange indexerSensor;
  private CANrangeConfiguration indexerSensorConfig;

  private IndexerState currentState = IndexerState.STOP;
  /** Creates a new Indexer. */
  public Indexer() {
    spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorId);

    spindexerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withNeutralMode(NeutralModeValue.Brake)
                                              .withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(IndexerConstants.kIndexerSupplyCurrentLimit));
    spindexerMotor.getConfigurator().apply(spindexerConfig);

    indexerMotor = new TalonFX(IndexerConstants.kIndexerMotorId);

    indexerConfig = new TalonFXConfiguration()
                        .withMotorOutput(new MotorOutputConfigs()
                                              .withNeutralMode(NeutralModeValue.Brake)
                                              .withInverted(InvertedValue.Clockwise_Positive))
                        .withCurrentLimits(new CurrentLimitsConfigs()
                                              .withSupplyCurrentLimit(IndexerConstants.kIndexerSupplyCurrentLimit));
    indexerMotor.getConfigurator().apply(indexerConfig);

    indexerSensor = new CANrange(IndexerConstants.kIndexerSensorId);

    indexerSensorConfig = new CANrangeConfiguration()
                        .withProximityParams(new ProximityParamsConfigs()
                                              .withMinSignalStrengthForValidMeasurement(IndexerConstants.kIndexerSensorMinSignalStrength)
                                              .withProximityThreshold(IndexerConstants.kIndexerSensorProximityThreshold))
                        .withToFParams(new ToFParamsConfigs()
                                              .withUpdateMode(UpdateModeValue.ShortRange100Hz));
    indexerSensor.getConfigurator().apply(indexerSensorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotorData();
  }

  public void setGoal(IndexerState desiredState) {
    currentState = desiredState;
    switch (desiredState) {
      case SPINDEX:
        spindexerMotor.set(IndexerConstants.kSpindexerInSpeed);
        indexerMotor.set(IndexerConstants.kIndexerInSpeed);
        break;
      case OUTTAKE:
        spindexerMotor.set(IndexerConstants.kSpindexerOutSpeed);
        indexerMotor.set(IndexerConstants.kIndexerOutSpeed);
        break;
      case IDLE:
        if(indexerSensor.getIsDetected().getValue()){
          spindexerMotor.stopMotor();
          indexerMotor.stopMotor();
        } else {
          spindexerMotor.set(IndexerConstants.kSpindexerInSpeed);
          indexerMotor.set(IndexerConstants.kIndexerInSpeed);
        }
        break;
      case STOP:
        spindexerMotor.stopMotor();
        indexerMotor.stopMotor();
        break;
    }
  }

  private void logMotorData(){
    DogLog.log("Subsystems/Indexer/IndexerState", currentState.name());
    
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorVelocity", spindexerMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorSupplyCurrent", spindexerMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorStatorCurrent", spindexerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/Spindexer/SpindexerMotorVoltage", spindexerMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Subsystems/Indexer/Basic/IndexerMotorVelocity", indexerMotor.getVelocity().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/IndexerMotorSupplyCurrent", indexerMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/IndexerMotorStatorCurrent", indexerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Subsystems/Indexer/Basic/IndexerMotorVoltage", indexerMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Subsystems/Indexer/Basic/IndexerSensor", indexerSensor.getIsDetected().getValue());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
