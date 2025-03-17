// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Wrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  public Wrist() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wrist/angle", getAngle().in(Degrees));
    // SmartDashboard.putBoolean("wrist/at target", isAtTarget.getAsBoolean());
    // SmartDashboard.putBoolean("wrist/at target (rough)", isRoughlyAtTarget.getAsBoolean());
  }

  public Angle getAngle(){
    return Degrees.of(0);
  }
}
