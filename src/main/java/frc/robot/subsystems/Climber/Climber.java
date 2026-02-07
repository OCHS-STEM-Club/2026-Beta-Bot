// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import dev.doglog.DogLog;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private Solenoid m_solenoid;
  private Compressor m_compressor;

  private ClimberState currentState = ClimberState.OFF;
  
  public Climber() {
    m_solenoid = new Solenoid(PneumaticsModuleType.REVPH, ClimberConstants.kSolenoidChannel);
    m_compressor = new Compressor(PneumaticsModuleType.REVPH);

  }

  public void solenoidOpen () {
    m_solenoid.set(true);
  }

  public void solenoidClose () {
    m_solenoid.set(false);
  }

  public void compressorEnable() {
    //Pick one of the three
    m_compressor.enableDigital();
    //m_compressor.enableAnalog(70, 120);
    //m_compressor.enableHybrid(70, 120); 
  }

  public void compressorDisable() {
    m_compressor.disable();
  }
  
  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case ON:
        solenoidOpen();
        compressorDisable();
        break;
      case OFF:
        solenoidClose();
        compressorEnable();
        break;
    }
  }

  public void logMotorData() {
    DogLog.log("Subsystems/Climber/ClimberState", currentState.name());
  }

  @Override
  public void periodic() {
    logMotorData();
  }
}
