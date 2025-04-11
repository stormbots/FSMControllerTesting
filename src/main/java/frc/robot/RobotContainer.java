// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FSM.FSM;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Rollers.Rollers;
import frc.robot.subsystems.Wrist.Wrist;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  final static Arm arm = new Arm();
  final static Wrist wrist = new Wrist(arm::getState);
  final static Rollers rollers = new Rollers();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(0);

  MechView mechanism = new MechView(arm,wrist,rollers);


  public enum BotState{
    Stow,
    L1,
    IntakeFloor,
    IntakeStation
  }
  FSM<BotState> fsm = new FSM<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    initStates();
    // Configure the trigger bindings
    configureBindings();

    // djsetup();
    
  }

  private void configureBindings() {

    driver.y().whileTrue(fsm.setRun(BotState.L1));
    driver.b().whileTrue(fsm.setRun(BotState.IntakeFloor));
    driver.x().whileTrue(fsm.setRun(BotState.IntakeStation));
    driver.a().whileTrue(fsm.setRun(BotState.Stow));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new SequentialCommandGroup(
    //   fsm.set(BotState.Stow),
    //   Commands.waitSeconds(1),
    //   fsm.setWait(BotState.IntakeStation),
    //   fsm.setWait(BotState.Stow),
    //   fsm.setWait(BotState.L1),
    //   fsm.setWait(BotState.Stow),
    //   fsm.setWait(BotState.IntakeFloor),
    //   fsm.setWait(BotState.Stow),
    //   fsm.setWait(BotState.L1),
    //   fsm.setRun(BotState.Stow)
    // );

    return new SequentialCommandGroup(
      Commands.print("AUTO initial stow"),
      fsm.forceState(BotState.Stow),
      fsm.await().withTimeout(3),

      Commands.print("AUTO Station"),
      fsm.pathToState(BotState.IntakeStation),

      Commands.print("AUTO L1"),
      fsm.pathToState(BotState.L1),

      Commands.print("AUTO Floor"),
      fsm.pathToState(BotState.IntakeFloor),

      Commands.print("AUTO L1 Again"),
      fsm.pathToState(BotState.L1),

      // fsm.pathToState(BotState.L1),
      // fsm.pathToState(BotState.IntakeFloor),
      // fsm.pathToState(BotState.L1),
      // fsm.pathToState(BotState.Stow),
      // Commands.waitSeconds(3),
      // fsm.pathToState(BotState.IntakeStation),
      // fsm.pathToState(BotState.L1)
      Commands.none()
    );


  }



  public void initStates(){
      fsm.addState(BotState.Stow,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->0),
              wrist.setAngle(()->120),
              rollers.stop()
          ),
          arm.isAtTarget.and(wrist.isAtTarget)
      );

      fsm.addState(BotState.L1,
        ()->new ParallelCommandGroup(
              arm.setAngle(()->45),
              wrist.setAngle(()->0),
              rollers.stop()
          ).until(arm.isAtTarget.and(wrist.isAtTarget))
          .andThen(rollers.eject()),
          rollers.isHoldingCoral.negate()
      );

      fsm.addState( BotState.IntakeStation,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->90),
              wrist.setAngle(()->10),
              rollers.stop()
          ).until(arm.isAtTarget.and(wrist.isAtTarget))
          .andThen(rollers.intake()),
          rollers.isHoldingCoral
      );

      fsm.addState( BotState.IntakeFloor,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->0),
              wrist.setAngle(()->-20),
              rollers.stop()
          ).until(arm.isAtTarget.and(wrist.isAtTarget))
          .andThen(rollers.intake()),
          rollers.isHoldingCoral
      );

      fsm.connect(BotState.Stow, BotState.IntakeStation, 1.0);
      fsm.connect(BotState.Stow, BotState.L1, 1.0);
      fsm.connect(BotState.Stow, BotState.IntakeFloor, 1.0);  
  }


  // enum ComplexBot{leftdown,leftup,left,right,rightdown,rightup};

  // Dijkstra<BotState> dj = new Dijkstra<>();
  // String outputString="";
  // BotState start=BotState.IntakeFloor;
  // BotState end=BotState.IntakeStation;

  // public void djsetup(){
  //   if(true) return;
  //   for(var k:BotState.values()) dj.addNode(k); // this can be shoved into dj

  //   dj.addConnection(BotState.Stow, BotState.IntakeStation, 1,true);
  //   dj.addConnection(BotState.Stow, BotState.IntakeFloor, 1,true);
  //   dj.addConnection(BotState.Stow, BotState.L1, 1,true);
  //   dj.addConnection(BotState.IntakeFloor, BotState.L1, 1,true);
    
  //   var selectstart=new SendableChooser<BotState>(); //TODO: Add this via FSM once
  //   for(var k:BotState.values()) selectstart.addOption(k.toString(), k);
  //   selectstart.setDefaultOption(BotState.IntakeStation.toString(), BotState.IntakeStation);
  //   selectstart.onChange(this::updateStart);

  //   var selectend=new SendableChooser<BotState>(); //TODO: Add this via FSM once
  //   for(var k:BotState.values()) selectend.addOption(k.toString(), k);
  //   selectend.setDefaultOption(BotState.L1.toString(), BotState.L1);
  //   selectend.onChange(this::updateEnd);

  //   SmartDashboard.putData("dj/start",selectstart);
  //   SmartDashboard.putData("dj/end",selectend);
  //   // SmartDashboard.putString("dj/output",outputString);
  //   // update();

  //   dj.computeCosts(BotState.IntakeStation,BotState.IntakeFloor);
  //   dj.computeCosts(BotState.Stow,BotState.L1);
  //   dj.computeCosts(BotState.L1,BotState.IntakeFloor);
  //   dj.computeCosts(BotState.L1,BotState.IntakeStation);

  // }

  // void updateStart(BotState state){
  //   start=state;
  //   update();
  // }
  // void updateEnd(BotState state){
  //   end=state;
  //   update();
  // }

  // void update(){
  //   var plswork=dj.computeCosts(start,end);
  //   outputString="";
  //   plswork.forEach((v)->outputString+=v.toString()+"("+dj.graph.get(v).cost+")->");
  //   SmartDashboard.putString("dj/output",outputString);
  //   System.out.println(outputString);
  // }


}
