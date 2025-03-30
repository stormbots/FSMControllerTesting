// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Rollers;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.BooleanSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Rollers extends SubsystemBase {

  AngularVelocity velocity = RPM.of(0);
  Angle position = Degrees.of(0);

  boolean hasCoral = false;

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
    return velocity;
  }

  public Command stop(){
    return new InstantCommand(()->velocity = RPM.of(0),this);
  }

  public Command intake(){
    return new RunCommand(()->velocity = RPM.of(10),this)
    .withTimeout(1.5)
    .finallyDo((e)->this.hasCoral=true);
  }

  public Command eject(){
    return new RunCommand(()->velocity = RPM.of(-10),this)
    .withTimeout(1)
    .finallyDo((e)->this.hasCoral=false)
    ;
  }

  public Trigger isHoldingCoral = new Trigger(()->hasCoral);

}
