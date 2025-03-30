// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.FSM.FSM;
import frc.robot.FSM.FSM.MyBotStates;
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
  FSM fsm = new FSM(arm,wrist,rollers);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {

  }





  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new SequentialCommandGroup(
    //   arm.setAngle(()->0).until(arm.isAtTarget).withTimeout(5),
    //   new WaitCommand(0.5),
    //   wrist.setAngle(()->90).until(wrist.isAtTarget).withTimeout(5),
    //   new WaitCommand(0.5),
    //   arm.setAngle(()->90).alongWith(wrist.setAngle(()->0)).until(arm.isAtTarget.and(wrist.isAtTarget)),
    //   new WaitCommand(0.5),
    //   arm.setAngle(()->0).until(arm.isAtTarget)
    // );

    return new SequentialCommandGroup(
      fsm.setWait(MyBotStates.Stow),
      new WaitCommand(1),
      fsm.setWait(MyBotStates.IntakeStation),
      fsm.setWait(MyBotStates.L1),
      fsm.setWait(MyBotStates.IntakeFloor),
      fsm.setWait(MyBotStates.L1),
      fsm.setRun(MyBotStates.Stow)
    );
  }
}
