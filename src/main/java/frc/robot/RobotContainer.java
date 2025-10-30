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
import frc.robot.subsystems.CyclingFSM;
import frc.robot.subsystems.IntakeFSM;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.CyclingFSM.States;
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

  // CyclingFSM cyclingFSM = new CyclingFSM();

  public enum BotState{
    Home,
    Stow,
    L1,
    L1_Score,
    IntakeFloor,
    IntakeStation
  }
  FSM<BotState> fsm = new FSM<>(BotState.Home);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    initStates();
    SmartDashboard.putData("fsmsendable",fsm);
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

    // driver.y().whileTrue(fsm.setRun(BotState.L1));
    driver.b().whileTrue(fsm.setRun(BotState.IntakeFloor));
    driver.x().whileTrue(fsm.setRun(BotState.IntakeStation));
    driver.a().whileTrue(fsm.setRun(BotState.Stow));

    //Tap to "prepare", and once you're there you can hold it to place it
    driver.y()
    .onTrue(fsm.setRun(BotState.L1_Score))
    .onFalse(fsm.setAsync(BotState.L1))
    ;


    //TODO cleaner API for this would be good.
    // cyclingFSM.fsm.addAutoTransition(States.a, States.b, driver.a());
  }

  // IntakeFSM intakefsm = new IntakeFSM();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // if(true)return intakefsm.testIntakeSequences();

    return new SequentialCommandGroup(
      Commands.print("AUTO initial stow"),
      // fsm.forceState(BotState.Stow),
      fsm.await().withTimeout(3),

      Commands.print("AUTO Station"),
      fsm.setWait(BotState.IntakeStation),

      Commands.print("AUTO L1"),
      fsm.setWait(BotState.L1),
      
      Commands.print("DRIVING"),
      new WaitCommand(0.5),

      Commands.print("AUTO L1"),
      fsm.setWait(BotState.L1_Score),

      Commands.print("AUTO Floor"),
      fsm.setWait(BotState.IntakeFloor),

      Commands.print("AUTO L1 Again"),
      fsm.setWait(BotState.L1_Score),

      Commands.print("AUTO Back  to stow"),
      fsm.setWait(BotState.Stow),

      Commands.none()
    );


  }



  public void initStates(){
      fsm.addState(BotState.Home,
        ()->new ParallelCommandGroup(
          //Do our homing process here.
        ),
        ()->true
      );

      fsm.addState(BotState.Stow,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->0),
              wrist.setAngle(()->120)
              // rollers.stop() //Left alone, so it runs defaultCommand
          ),
          arm.isAtTarget.and(wrist.isAtTarget)
      );

      fsm.addState(BotState.L1,
        ()->new ParallelCommandGroup(
              arm.setAngle(()->45),
              wrist.setAngle(()->0)
              // rollers.stop() //Left with defaultCommand
          ),
          arm.isAtTarget.and(wrist.isAtTarget).debounce(0.1)
      );

      fsm.addState(BotState.L1_Score,
        ()->rollers.eject(),
        rollers.isHoldingCoral.negate().debounce(0.2)
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
              wrist.setAngle(()->-20)//,
              // rollers.stop()
          ).until(arm.isAtTarget.and(wrist.isAtTarget))
          .andThen(rollers.intake()),
          rollers.isHoldingCoral
      );

      fsm.addConnection(BotState.Stow, BotState.L1);
      fsm.addConnectionHub(BotState.Stow, BotState.IntakeStation, BotState.L1, BotState.IntakeFloor);
      // fsm.addConnection(BotState.IntakeFloor,BotState.L1);
      fsm.addConnection(BotState.L1,BotState.L1_Score);
      
      // fsm.connect(BotState.IntakeFloor, BotState.L1, 1,false);
  }


}
