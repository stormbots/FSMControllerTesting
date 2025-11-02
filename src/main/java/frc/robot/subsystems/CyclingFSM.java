// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.logging.Level;
import java.util.logging.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FSM.FSM;

public class CyclingFSM extends SubsystemBase {
  public enum States{a,b,c,d,even,odd};
  public FSM<States> fsm = new FSM<>(States.a);

  /** Creates a new CyclingFSM. */
  public CyclingFSM() {

    fsm.addState(States.a, ()->printy("a"),()->false);
    fsm.addState(States.b, ()->printy("b"), ()->false);
    fsm.addState(States.c, ()->printy("c"), ()->false);
    fsm.addState(States.d, ()->printy("d"), ()->false);
    fsm.addState(States.even, ()->printy("even"), ()->false);
    fsm.addState(States.odd, ()->printy("odd"), ()->false);

    fsm.addConnection(States.a, States.b,1,false);
    fsm.addConnection(States.b, States.c,1,false);
    fsm.addConnection(States.c, States.d,1,false);
    fsm.addConnection(States.d, States.even,1,false);
    fsm.addConnection(States.d, States.odd,1,false);
    fsm.addConnection(States.even, States.a,1,false);
    fsm.addConnection(States.odd, States.a,1,false);

    fsm.addAutoTransition(States.a, States.b);
    fsm.addAutoTransition(States.b, States.c);
    fsm.addAutoTransition(States.c, States.d);
    fsm.addAutoTransition(States.d, States.even, ()->(int)Timer.getFPGATimestamp()%2==0);
    fsm.addAutoTransition(States.d, States.odd, ()->(int)Timer.getFPGATimestamp()%2==1);
    fsm.addAutoTransition(States.even, States.a);
    fsm.addAutoTransition(States.odd, States.a);

    //For giggles, reset it every few seconds
    // this.setDefaultCommand(
    //   Commands.waitSeconds(13)
    //   .andThen(fsm.setWait(States.a))
    //   .repeatedly()
    // );
  }

  public Command printy(String str){
    return new StartEndCommand(
      ()->System.out.println("Start  "+str),
      ()->System.out.println("Ending "+str),
      this
    )
    .withTimeout(1.5)
    ;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("cycle/current", fsm.getActiveState().toString());
  }
}
