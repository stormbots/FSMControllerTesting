// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.FSM.FSM;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Extendo.Extendo;
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
  final static Extendo extendo = new Extendo(arm::getState);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driver = new CommandXboxController(0);

  MechView mechanism = new MechView(arm,wrist,extendo,rollers);
  ArmIK ik = new ArmIK(arm,extendo,wrist);

  //This is a demo FSM that just cycles between various states. 
  // Subsystem FSMs might wind up looking similar
  // CyclingFSM cyclingFSM = new CyclingFSM();

  //Another hastily put together demo subsystem relying heavily on automatic transitions
  //IntakeFSM intakeFSM = new IntakeFSM();

  public enum BotState{
    Home,
    Stow,
    Crossover,
    L1,
    L1_Score,
    IntakeFloor,
    IntakeStation,
    L2Front,
    L2Front_Score,
    L2Rear,
    L2Rear_Score,
    LockdownReady,
    LockdownLocked
  }
  FSM<BotState> fsm = new FSM<>(BotState.Home);

  Trigger atPosition = arm.isAtTarget.and(wrist.isAtTarget);//.and(extendo.isAtTarget);

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

    //Hold to score, tap/release to remain at the prepration pose
    driver.y()
    .whileTrue(fsm.setRun(BotState.L1_Score))
    .onFalse(fsm.setAsync(BotState.L1))
    ;

    //Allow arbitrary tinkering of the coral status to muck with the states
    driver.leftBumper().onTrue(rollers.giveCoral());
    driver.rightBumper().onTrue(rollers.takeCoral());

    driver.back().onTrue(fsm.forceState(BotState.L1_Score));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // if(true) return Commands.sequence(
    //   fsm.setWait(BotState.IntakeStation).until(rollers.isHoldingCoral),
    //   fsm.setWait(BotState.L2Rear_Score).until(rollers.isHoldingCoral.negate()),
    //   fsm.setWait(BotState.IntakeStation).until(rollers.isHoldingCoral),
    //   fsm.setWait(BotState.L2Front_Score).until(rollers.isHoldingCoral.negate())
    //   ).repeatedly();

    // if(true) return Commands.sequence(
    //   fsm.disable(),
    //   ik.interp(()->new Translation2d(+724, 0), InchesPerSecond.of(6), ()->Degrees.of(0)).withTimeout(5),
    //   ik.interp(()->new Translation2d(7, 24), InchesPerSecond.of(6), ()->Degrees.of(0)).withTimeout(5)
    // ).repeatedly();

    //Show off the IK by holding a point in space and moving in a circle
    // if(true) return Commands.sequence(
    //   fsm.disable(),
    //   ik.runIK(()->8, ()->24, ()->-180+(Timer.getFPGATimestamp()*20%360))
    // ).repeatedly();


    return new SequentialCommandGroup(
      Commands.print("AUTO initial stow"),
      // fsm.forceState(BotState.Home),
      // fsm.await().withTimeout(3),
      fsm.setWait(BotState.Stow),

      Commands.print("AUTO Station"),
      fsm.setWait(BotState.IntakeStation),
      Commands.print("AUTO L1"),
      fsm.setWait(BotState.L1_Score),

      Commands.print("AUTO Station"),
      fsm.setWait(BotState.IntakeStation),
      Commands.print("AUTO L1"),
      fsm.setWait(BotState.L2Front_Score),

      // Commands.print("DRIVING"),
      // new WaitCommand(0.5),
      Commands.print("AUTO Back  to stow"),
      fsm.setWait(BotState.Stow),

      Commands.print("AUTO Floor"),
      fsm.setWait(BotState.IntakeFloor),

      Commands.print("AUTO L1"),
      // fsm.setWait(BotState.L1),
      fsm.setWait(BotState.L1_Score),
      
      Commands.print("AUTO Floor"),
      fsm.setWait(BotState.IntakeFloor),
      Commands.print("AUTO L1"),
      // fsm.setWait(BotState.L1),
      fsm.setWait(BotState.L2Rear_Score),

      //Just for fun swing it back to L2Rear to show the FSM midpoint
      fsm.setWait(BotState.L2Front),

      Commands.print("AUTO Back  to stow"),
      // fsm.setWait(BotState.Stow),

      Commands.print("AUTO Lock the bot"),
      fsm.setWait(BotState.LockdownLocked),

      Commands.none()
    );


  }



  public void initStates(){
      fsm.addState(BotState.Home,
        //This would instead be a homing procecure, but this works for now. 
        ()->new ParallelCommandGroup(
              arm.setAngle(()->0),
              wrist.setAngle(()->120),
              extendo.setDistance(()->0)
        ),
        atPosition
      );

      fsm.addState(BotState.Stow,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->0),
              wrist.setAngle(()->120),
              extendo.setDistance(()->0)
              // rollers.stop() //Left alone, so it runs defaultCommand
          ),
          ()->{return 
            arm.getAngle().lte(Degrees.of(45))
            &&extendo.getDistance().lte(Inches.of(0.5))
            &&wrist.getAngle().gte(Degree.of(90));
          },
          atPosition
      );

      fsm.addState(BotState.Crossover,
      ()->new ParallelCommandGroup(
            arm.setAngle(()->60),
            wrist.setAngle(()->0),
            extendo.setDistance(()->0)
        ),
        ()->{return 
          extendo.getDistance().lte(Inches.of(0.5))
          && arm.getAngle().isNear(Degree.of(60), Degree.of(20));
        },
        atPosition
    );

      fsm.addState(BotState.L1,
        ()->new ParallelCommandGroup(
              arm.setAngle(()->45),
              wrist.setAngle(()->0),
              extendo.setDistance(()->0)
          ),
          atPosition
      );

      fsm.addState(BotState.L1_Score,
        //Only accessable from L1, so we're at implied to be at the right spot
        ()->rollers.eject(),
        ()->true, //We generally don't care about this case
        rollers.isHoldingCoral.negate()
      );

      fsm.addState( BotState.IntakeStation,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->90),
              wrist.setAngle(()->10),
              extendo.setDistance(()->2),
              rollers.stop()
          ).until(atPosition)
          .andThen(rollers.intake()),
          atPosition,
          rollers.isHoldingCoral
      );

      fsm.addState( BotState.IntakeFloor,
          ()->new ParallelCommandGroup(
              arm.setAngle(()->0),
              wrist.setAngle(()->-20),
              extendo.setDistance(()->0)
              // rollers.stop()
          ).until(atPosition)
          .andThen(rollers.intake()),
          atPosition,
          rollers.isHoldingCoral
      );


      fsm.addState(BotState.L2Front,
      ()->ik.runIK(()->20,()->30,()->-45),
        atPosition
      );

      fsm.addState(BotState.L2Front_Score,
        ()->rollers.eject(),
        ()->true, //We generally don't care about this case
        rollers.isHoldingCoral.negate()
      );

      fsm.addState(BotState.L2Rear,
      ()->ik.runIK(()->2,()->30,()->225),
        atPosition
      );

      fsm.addState(BotState.L2Rear_Score,
        ()->rollers.eject(),
        ()->true, //We generally don't care about this case
        rollers.isHoldingCoral.negate()
      );

      fsm.addState(BotState.LockdownReady,
      ()->new ParallelCommandGroup(
            arm.setAngle(()->0),
            wrist.setAngle(()->135),
            extendo.setDistance(()->0)
        ),
        atPosition.debounce(0.2)
      );

      fsm.addState(BotState.LockdownLocked,
      ()->new ParallelCommandGroup(
            arm.setAngle(()->0),
            wrist.setAngle(()->135),
            extendo.setDistance(()->1)
        ),
        atPosition
      );


      //Connect our transition poses
      fsm.addConnection(BotState.Stow, BotState.Crossover);

      //Connect up the front side
      fsm.addConnectionHub(BotState.Stow,BotState.L1, BotState.IntakeFloor);
      fsm.addDirectionalConnection(BotState.IntakeFloor,BotState.L1);
      fsm.addConnectionHub(BotState.Crossover,BotState.IntakeFloor,BotState.L1,BotState.L2Front);
      fsm.addDirectionalConnection(BotState.IntakeFloor,BotState.L1);

      //Connect up the reverse side
      fsm.addConnectionHub(BotState.Crossover,BotState.IntakeStation,BotState.L2Rear);

      //Connect scoring states up
      fsm.addConnection(BotState.L1, BotState.L1_Score);
      fsm.addConnection(BotState.L2Front, BotState.L2Front_Score);
      fsm.addConnection(BotState.L2Rear, BotState.L2Rear_Score);
      
      //Set a unidirectional connection from our homing process
      fsm.addDirectionalConnection(BotState.Home,BotState.Stow);
      fsm.addAutoTransition(BotState.Home, BotState.Stow);

      //Simple sequential setup
      fsm.addConnection(BotState.Stow,BotState.LockdownReady,BotState.LockdownLocked);

      // Automatically back out of some handling states when we're done there
      fsm.addAutoTransition(BotState.L1_Score, BotState.L1, rollers.isHoldingCoral.negate());
      fsm.addAutoTransition(BotState.L2Front_Score, BotState.L2Front, rollers.isHoldingCoral.negate());
      fsm.addAutoTransition(BotState.L2Rear_Score, BotState.L2Rear, rollers.isHoldingCoral.negate());

      fsm.addAutoTransition(BotState.IntakeFloor, BotState.Stow, rollers.isHoldingCoral,true);
      fsm.addAutoTransition(BotState.IntakeStation, BotState.Crossover, rollers.isHoldingCoral,false);

      //Note, auto-transitions use a routed sequence, and do not require or imply a direct path
      //between the two states!


  }


}
