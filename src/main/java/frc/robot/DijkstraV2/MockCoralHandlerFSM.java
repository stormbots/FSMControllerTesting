// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.DijkstraV2;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Creates a new MockCoralHandlerFSM. */
public class MockCoralHandlerFSM extends SubsystemBase {
    public static enum CS{
      intaking,
      alignReverse,
      alignForward,
      loaded,
      unloaded,
      scoring
  }

  SparkFlex motor = new SparkFlex(1, MotorType.kBrushless);
  FSM2<CS> fsm = new FSM2<>(CS.unloaded);

  DigitalInput coralInput=new DigitalInput(0);
  Trigger coralDetected = new Trigger(coralInput::get);

  //These will be externally controlled through commands or other interfaces
  boolean scoreSignal=false;
  boolean loadSignal=false;

  MockCoralHandlerFSM(){
    configureFSM();
  }

  private void configureFSM(){
    fsm.addState(CS.intaking, runTrapProfile(3))
    .addTransition(CS.alignReverse, coralDetected);
    ;

    fsm.addState(CS.alignReverse, ()->motor.set(-0.2))
    .addTransition(CS.alignForward, coralDetected.negate())
    ;

    fsm.addState(CS.alignForward, ()->motor.set(0.2))
    .addTransition(CS.loaded, coralDetected)
    ;

    fsm.addState(CS.loaded, ()->motor.set(0.0))
    .addTransition(CS.unloaded, coralDetected.negate())//falls out or something
    ;

    fsm.addState(CS.scoring, ()->motor.set(0.2))
    .addTransition(CS.unloaded, coralDetected.negate())
    .addTimerTransition(CS.unloaded, (t)->t>2);

    fsm.addState(CS.unloaded, ()->motor.set(0.0))
    .addTransition(CS.alignReverse, coralDetected)
    ;

    // Handle scoring from any loaded state
    fsm.addSignalTransition(CS.scoring,()->scoreSignal,
      CS.unloaded,
      CS.alignForward,
      CS.alignReverse,
      CS.loaded
    );

    //Handle loading from our unloaded state
    fsm.addSignalTransition(CS.intaking,()->loadSignal,
      CS.unloaded
    );

     //reseat game piece
    fsm.addSignalTransition(CS.alignReverse, ()->loadSignal,
      CS.loaded
    );

    fsm.validate();

    SmartDashboard.putData("FSM/Coral/fsm",fsm);
    SmartDashboard.putData("FSM/coral/chooser",fsm.getSelectableChooser());
    new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(()->fsm.setState(CS.unloaded)));

  }

  @Override
  public void periodic() {
    fsm.update();
  }

  //Send a signal that we should load if able
  public Command load(){
    return startEnd(()->loadSignal=true, ()->loadSignal=false);
  }

  //Send a signal that we should score if able
  public Command score(){
    return startEnd(()->scoreSignal=true, ()->scoreSignal=false);
  }

  Trigger isCoralLoaded=new Trigger(()->fsm.isInState(CS.intaking,CS.unloaded)).negate();
  Trigger isCoralLoading=new Trigger(()->fsm.isInState(CS.intaking,CS.alignForward,CS.alignReverse));
  Trigger isCoralEmpty=new Trigger(()->fsm.isInState(CS.unloaded));
  

  public Command runTrapProfile(double velocity){
      return startRun(()->{
        //init stuff
      }, ()->{
        //do the thing.
      });
  }
}
