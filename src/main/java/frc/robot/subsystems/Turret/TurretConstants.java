// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

/** Add your docs here. */
public class TurretConstants {
    public static final int kMotorId = 22;

    public static final double kMinAngle = -180;
    public static final double kMaxAngle = 180;

    public static final double kP = 1;
    public static final double kI = 1;
    public static final double kD = 1;

    public static final double kCruiseVelocity = 1;
    public static final double kAcceleration = 1;
    public static final double kJerk = 1;

    public static final double kTurretTransformInchesX = 1;
    public static final double kTurretTransformInchesY = 10;

    public static final double kFeedforward = 0.0;
    
    public static final double kTolerance = 0.01;

    public static final double kSupplyCurrentLimit = 35;

    public static final double kSensorToMechanismRatio = .5;

    public static final double kRotorToSensorRatio = .5;

    public static final double kSpeed = 1;
}
