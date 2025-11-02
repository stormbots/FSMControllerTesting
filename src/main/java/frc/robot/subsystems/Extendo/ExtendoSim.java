// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Extendo;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class ExtendoSim {

  SparkFlex motor;
  SparkFlexSim simMotor;
  
  public ExtendoSim(SparkFlex motor){
    this.motor = motor;
    simMotor = new SparkFlexSim(this.motor, DCMotor.getNeoVortex(1));
  
    var startPosition=Inches.of(0);
    simArm.setState(0, 0);
    simMotor.setPosition(0);
    simArm.setState(0, 0);

    if(Robot.isSimulation()) new RunCommand(this::update).ignoringDisable(true).schedule();
  }

  ElevatorSim simArm = new ElevatorSim(
    DCMotor.getNeoVortex(1),
    50,
    Pounds.of(3).in(Kilograms),
    1/50.0,
    0,
    Inches.of(36).in(Meters),
    false,
    0
  );

  public void update() {
    var vbus = 12;
    var dt = 0.02;

    simArm.setInputVoltage(simMotor.getAppliedOutput()*vbus);
    simArm.update(dt);

    SmartDashboard.putNumber("extendo/distance(sim)", simArm.getPositionMeters());

    simMotor.iterate(
      MetersPerSecond.of(simArm.getVelocityMetersPerSecond()).in(InchesPerSecond),
      vbus,
      dt
    );
  }

  public Distance getDistance(){
    return Meters.of(simArm.getPositionMeters());
  }

}
