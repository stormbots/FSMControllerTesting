// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Rollers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rollers extends SubsystemBase {
  /** Creates a new Rollers. */
  public Rollers() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("rollers/rotations", getAngle().in(Rotation));
  }

  public Angle getAngle(){
    return Rotations.of(getVelocity().in(RotationsPerSecond)*Timer.getFPGATimestamp());
  }
  public AngularVelocity getVelocity(){
    return RotationsPerSecond.of(2);
  }

}
