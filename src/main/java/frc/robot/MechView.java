// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Rollers.Rollers;
import frc.robot.subsystems.Wrist.Wrist;

/** Add your docs here. */
public class MechView {

    Arm armSystem;
    Wrist wristSystem;
    Rollers rollersSystem;

    //visual constants
    double angledelta = 15;
    double barlength = 24;


    public Mechanism2d mech = new Mechanism2d(36, 72);
    MechanismRoot2d root = mech.getRoot("ArmRoot", 2, 7);

    //Major systems
    MechanismLigament2d arm = root.append(new MechanismLigament2d("Arm", 16, 0));
    MechanismLigament2d wrist = arm.append(new MechanismLigament2d("Wrist", 4, 0));
    MechanismLigament2d rollers = wrist.append(new MechanismLigament2d("Rollers", 1, 0));
    MechanismLigament2d rollersR = wrist.append(new MechanismLigament2d("RollersInverse", -1, 0));
    
    //Visualization bits
    MechanismLigament2d chassisTop = root.append(new MechanismLigament2d("ChassisTop", 16, 0));
    MechanismLigament2d chassisBack = root.append(new MechanismLigament2d("ChassisBack", 4, -90));
    MechanismLigament2d chassisFront = chassisTop.append(new MechanismLigament2d("ChassisBack", 4, -90));
    MechanismLigament2d wristBackstop = arm.append(new MechanismLigament2d("WristBackstop", 2, 0));


    public MechView( 
      Arm armSystem,
      Wrist wristSystem,
      Rollers rollersSystem
    ) {
      this.armSystem = armSystem;
      this.wristSystem = wristSystem;
      this.rollersSystem = rollersSystem;

      var barweight = 6;

      arm.setColor(new Color8Bit(Color.kDarkGray));
      arm.setLineWeight(barweight * 2);

      wrist.setColor(new Color8Bit(Color.kGray));
      wrist.setLineWeight(barweight);


      var rollersWidth=12;
      var rollersLength=1;
      rollers.setColor(new Color8Bit(Color.kOrange));
      rollers.setLineWeight(rollersWidth);
      rollers.setLength(rollersLength);
      rollersR.setColor(new Color8Bit(Color.kOrange));
      rollersR.setLineWeight(rollersWidth);
      rollersR.setLength(-rollersLength);

      chassisTop.setColor(new Color8Bit(Color.kDarkGray));
      chassisFront.setColor(new Color8Bit(Color.kDarkGray));
      chassisBack.setColor(new Color8Bit(Color.kDarkGray));
      chassisTop.setLineWeight(1);
      chassisFront.setLineWeight(1);
      chassisBack.setLineWeight(1);
      wristBackstop.setLineWeight(2);

      SmartDashboard.putData("mechanism/arm", mech);
      new RunCommand(this::update).ignoringDisable(true).schedule();
      
    }

    public void update() {
      arm.setAngle(armSystem.getAngle().in(Degree));
      wrist.setAngle(wristSystem.getAngle().in(Degree));
      
      rollers.setAngle(rollersSystem.getAngle().in(Degrees));
      rollersR.setAngle(rollersSystem.getAngle().in(Degrees));


      wristBackstop.setAngle(wristSystem.getAngle().plus(Degree.of(-90)).in(Degree));
      
      var color = Color.kOrange;
      var vel = rollersSystem.getVelocity().in(DegreesPerSecond);
      if( vel < 0) color = Color.kRed;
      if( vel > 0) color = Color.kGreen;
      rollers.setColor(new Color8Bit(color));
      rollersR.setColor(new Color8Bit(color));

    }
  }
