// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Robot;

/** Add your docs here. */
public class ArmSim {

  SparkFlex motor;
  SparkFlexSim simMotor;
  
  public ArmSim(SparkFlex motor){
    this.motor = motor;
    simMotor = new SparkFlexSim(this.motor, DCMotor.getNeoVortex(1));
  
    var startPosition=Degrees.of(75);
    simArm.setState(startPosition.in(Radians), 0);
    simMotor.setPosition(startPosition.in(Degrees));

    if(Robot.isSimulation()) new RunCommand(this::update).ignoringDisable(true).schedule();
  }

  double armlength = Inches.of(16).in(Meters);
  double armMass = Pounds.of(7.6).in(Kilograms);
  SingleJointedArmSim simArm = new SingleJointedArmSim(
    DCMotor.getNeoVortex(1), 
    5*5*64/18.0, 
    armMass*armlength*armlength, 
    armlength, 
    Degrees.of(-180).in(Radians), 
    Degrees.of(270).in(Radians), 
    true, 
    Degrees.of(90).in(Radians)
  );


  public void update() {
    var vbus = 12;
    var dt = 0.02;

    simArm.setInputVoltage(simMotor.getAppliedOutput()*vbus);
    simArm.update(dt);

    SmartDashboard.putNumber("arm/angle(sim)", Math.toDegrees(simArm.getAngleRads()));

    simMotor.iterate(
      RadiansPerSecond.of(simArm.getVelocityRadPerSec()).in(DegreesPerSecond),
      vbus,
      dt
    );
  }

  public Angle getAngle(){
    return Radians.of(simArm.getAngleRads());
  }

}
