package frc.robot.FSM;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Rollers.Rollers;
import frc.robot.subsystems.Wrist.Wrist;

public class FSM {
    Arm arm; 
    Wrist wrist; 
    Rollers rollers;

    public enum MyBotStates{
        Stow,
        L1,
        IntakeFloor,
        IntakeStation
    }

    HashMap<MyBotStates,BotState<MyBotStates>> stateMap = new HashMap<>();
    BotState currentState;
    BotState goalState;


    public FSM(Arm arm, Wrist wrist, Rollers rollers){
        this.arm = arm;
        this.wrist = wrist; 
        this.rollers = rollers;

        initStates();
        setRunAsync(MyBotStates.Stow); //Manditory to avoid nulls in botstates

        new RunCommand(this::periodic)
        .ignoringDisable(true)
        .withName("FSM Manager")
        .schedule();
    }

    public void periodic(){
        SmartDashboard.putString("fsm/currentState", currentState.name.toString());
        SmartDashboard.putString("fsm/goalState", goalState.name.toString());
        SmartDashboard.putBoolean("fsm/done",goalState.exitCondition.getAsBoolean());
    }

    public Command setWait(MyBotStates state){
        return setRun(state).until(stateMap.get(state).exitCondition);
    }

    public Command setRun(MyBotStates state){
        this.currentState=stateMap.get(state);
        this.goalState=this.currentState; //TODO Fix this once state transitions work
        return (Command)currentState.commandSupplier.get();
    }

    /** Run the state's command and proceed, ignoring when or how it completes
     * Useful for commands sequenced with drivetrain sequences
    */
    public Command setRunAsync(MyBotStates state){
        setRun(state).schedule();
        return new InstantCommand();
    }

    /** Wait for the currently set goal state to be reached */
    public Command await(){
        return new RunCommand(()->{})
        .until(()->currentState==goalState && goalState.exitCondition.getAsBoolean());
    }


    public void addState(BotState<MyBotStates> state){
        stateMap.put(state.name, state);
    }

    public void initStates(){

        addState(new BotState<MyBotStates>(MyBotStates.Stow,
            ()->new ParallelCommandGroup(
                arm.setAngle(()->0),
                wrist.setAngle(()->120),
                rollers.stop()
            ),
            arm.isAtTarget.and(wrist.isAtTarget)
        ));

        addState(new BotState<MyBotStates>(MyBotStates.L1,
        ()->new ParallelCommandGroup(
                arm.setAngle(()->45),
                wrist.setAngle(()->0),
                rollers.stop()
            ).until(arm.isAtTarget.and(wrist.isAtTarget))
            .andThen(rollers.eject()),
            rollers.isHoldingCoral.negate()
        ));

        addState( new BotState<MyBotStates>(MyBotStates.IntakeStation,
            ()->new ParallelCommandGroup(
                arm.setAngle(()->90),
                wrist.setAngle(()->10),
                rollers.stop()
            ).andThen(rollers.intake()),
            rollers.isHoldingCoral
        ));

        addState( new BotState<MyBotStates>(MyBotStates.IntakeFloor,
            ()->new ParallelCommandGroup(
                arm.setAngle(()->0),
                wrist.setAngle(()->-20),
                rollers.stop()
            ).andThen(rollers.intake()),
            rollers.isHoldingCoral
        ));

    }

        



    



}
