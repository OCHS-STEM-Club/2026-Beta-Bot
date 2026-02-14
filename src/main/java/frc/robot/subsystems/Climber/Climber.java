// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import dev.doglog.DogLog;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private Solenoid solenoid;
  private Compressor compressor;

  private ClimberState currentState = ClimberState.RETRACT;
  
  public Climber() {
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ClimberConstants.kSolenoidChannel);
    compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  }

  
  
  public void setGoal(ClimberState desiredState) {
    currentState = desiredState;
    switch(desiredState){
      case EXTEND:
        solenoid.set(true);
        break;
      case RETRACT:
        solenoid.set(false);
        break;
    }
  }

  public void logPneumaticsData() {
    DogLog.log("Subsystems/Climber/ClimberState", currentState.name());
    SmartDashboard.putData(compressor);
    SmartDashboard.putData(solenoid);
  }

  @Override
  public void periodic() {
    logPneumaticsData();
    compressor.enableDigital();
  }
}
