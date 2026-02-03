// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public enum ShooterState{
  SHOOT_BLUE_HUB,
  SHOOT_RED_HUB,
  SHOOT_BLUE_DEPOT_SHUTTLING,
  SHOOT_RED_DEPOT_SHUTTLING,
  SHOOT_BLUE_OUTPOST_SHUTTLING,
  SHOOT_RED_OUTPOST_SHUTTLING,
  IDLE,
  STOP
}

