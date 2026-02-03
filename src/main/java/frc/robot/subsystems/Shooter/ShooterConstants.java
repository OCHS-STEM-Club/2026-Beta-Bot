// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import java.util.TreeMap;

/** Add your docs here. */
public class ShooterConstants {
    public static final int kMotorId = 1;

    public static final int kSupplyCurrentLimit = 15;

    public static final double kP = 1;
    public static final double kI = 1;
    public static final double kD = 1;

    public static final int kCruiseVelocity = 1000;
    public static final int kAcceleration = 1000;
    public static final int kJerk = 1000;

    public static final int kSensorToMechRatio = 1;
    public static final int kRotorToSensorRatio = 1;

    public static TreeMap<Integer, Double> kShooterMap = new TreeMap<>(); //may need to change data types

    public void fillShooterMap() {
        //left off here
    }
}

