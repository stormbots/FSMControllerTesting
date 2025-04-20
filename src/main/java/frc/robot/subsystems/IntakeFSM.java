// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FSM.FSM;

public class IntakeFSM extends SubsystemBase {
  
  public enum IntakeState{
    Unloaded,
    CoralLoading,
    CoralAlignReverse,
    CoralAlignForward,
    CoralLoaded,
    CoralScore,
  }
  FSM<IntakeState> fsm = new FSM<>(IntakeState.CoralAlignReverse); //This does stuff by default :(
  
  SparkMax rollers = new SparkMax(21, MotorType.kBrushless);
  private boolean coralSensor=false;

  /** Creates a new IntakeFSM. */
  public IntakeFSM() {
    SmartDashboard.putData("intakefsm",fsm);

    fsm.addState(IntakeState.Unloaded, 
    ()->this.run(()->rollers.set(0)),
    ()->true
    );

    fsm.addState(IntakeState.CoralLoading, 
      this::fakeload,
      ()->coralSensor==true
    );

    fsm.addState(IntakeState.CoralAlignReverse, 
      this::fakereversealign,
      ()->coralSensor==false
    );

    fsm.addState(IntakeState.CoralAlignForward, 
      this::fakeforwardalign,
      ()->coralSensor==true
    );

    fsm.addState(IntakeState.CoralLoaded, 
      ()->this.run(()->rollers.set(0)),
      ()->true
    );
    fsm.addState(IntakeState.CoralScore, 
      this::fakescore,
      ()->coralSensor==false
    );

    //Normal scoring flow
    fsm.connect(IntakeState.Unloaded, IntakeState.CoralLoading, 1,false);
    fsm.connect(IntakeState.CoralLoading, IntakeState.CoralAlignReverse, 1,false);
    fsm.connect(IntakeState.CoralAlignReverse, IntakeState.CoralAlignForward,1,false);
    fsm.connect(IntakeState.CoralAlignForward, IntakeState.CoralLoaded, 1,false);
    fsm.connect(IntakeState.CoralLoaded, IntakeState.CoralScore, 1,false);
    fsm.connect(IntakeState.CoralScore, IntakeState.Unloaded, 1,false);

    //Cancelled score attempt
    fsm.connect(IntakeState.CoralScore, IntakeState.CoralAlignReverse, 1,false);

    //Configure the automatic transitions
    // fsm.addAutoTransition(IntakeState.CoralScore, IntakeState.Unloaded, ()->coralSensor==false);
    fsm.addAutoTransition(IntakeState.CoralScore, IntakeState.CoralAlignReverse, ()->coralSensor==true);
    fsm.addAutoTransition(IntakeState.CoralAlignReverse, IntakeState.CoralAlignForward);
    fsm.addAutoTransition(IntakeState.CoralAlignForward, IntakeState.CoralLoaded);

    fsm.addAutoTransition(IntakeState.CoralLoading, IntakeState.CoralAlignReverse, ()->coralSensor==true);
    fsm.addAutoTransition(IntakeState.CoralLoading, IntakeState.Unloaded, ()->coralSensor==false);
    // //Allow creating transition/ending states like so
    // fsm.addState(IntakeState.CoralAligning,()->this.run(()->{}).withTimeout(1),IntakeState.CoralLoaded);
    // // ... which is just a shortcut for a command that 
    // fsm.autoTransition(IntakeState.CoralAligning, IntakeState.CoralLoaded, ()->true);
  }


  private Command fakeload(){
    return Commands.sequence(
      Commands.runOnce(()->System.out.print("LOADING!")),
      Commands.waitSeconds(2).until(()->coralSensor==true),
      Commands.runOnce(()->coralSensor=true),
      Commands.none()
    );
  }
  private Command fakescore(){
    return Commands.sequence(
      Commands.runOnce(()->System.out.print("SCORING!")),
      Commands.waitSeconds(2).until(()->coralSensor==false),
      Commands.runOnce(()->coralSensor=false),
      Commands.runOnce(()->System.out.println(" ... Done!")),
      Commands.none()
    );
  }
  private Command fakereversealign(){
    return Commands.sequence(
      Commands.runOnce(()->System.out.print("ALIGN REVERSING!")),
      Commands.waitSeconds(1).until(()->coralSensor==false),
      Commands.runOnce(()->coralSensor=false),
      Commands.runOnce(()->System.out.println(" ... Done!")),
      Commands.none()
    );
  }
  private Command fakeforwardalign(){
    return Commands.sequence(
      Commands.runOnce(()->System.out.print("ALIGN FORWARD!")),
      Commands.waitSeconds(1).until(()->coralSensor==true),
      Commands.runOnce(()->coralSensor=true),
      Commands.runOnce(()->System.out.println(" ... Done!")),
      Commands.none()
    );
  }
  

  public Command intake(){
    //This will follow the normal flow
    return fsm.setWait(IntakeState.CoralLoading);
  }
  public Command score(){
    //Ignores everything and does a score action
    return fsm.forceState(IntakeState.CoralScore)
    .andThen(fsm.await())
    .andThen(Commands.waitSeconds(2))
    ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("intakefsm/current", fsm.getCurrentState().name.toString());
  }

  public Command testIntakeSequences(){
    return Commands.sequence(
      Commands.print("Starting"),
      // intake(),
      // Commands.waitSeconds(4),
      // fsm.await(),
      Commands.print("Score 1"),
      score(),
      Commands.print("Score 2"),
      score(),
      Commands.print("Intaking"),
      intake(),
      Commands.print("Score 3+4"),
      score(),
      score(),
      Commands.print("Intaking twice"),
      intake(),
      intake(),
      Commands.waitSeconds(1),
      Commands.print("Forcing"),
      fsm.forceState(IntakeState.CoralAlignReverse),
      Commands.print("Sequence over"),
      Commands.none()
    );
  }

}
