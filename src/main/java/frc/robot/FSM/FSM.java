package frc.robot.FSM;

import java.util.HashMap;
import java.util.Map;

import javax.sql.StatementEvent;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
    BotState<MyBotStates> activeState;
    BotState<MyBotStates> goalState;


    public FSM(Arm arm, Wrist wrist, Rollers rollers){
        this.arm = arm;
        this.wrist = wrist; 
        this.rollers = rollers;

        new RunCommand(this::periodic)
        .ignoringDisable(true)
        .withName("FSM Manager")
        .schedule();

    }

    public void periodic(){
        SmartDashboard.putString("fsm/currentState", activeState.name.toString());
        SmartDashboard.putString("fsm/goalState", goalState.name.toString());
        SmartDashboard.putBoolean("fsm/done",goalState.exitCondition.getAsBoolean());

        //Manage running the state commands
        for(MyBotStates state : MyBotStates.values()){
            if(activeState!=goalState && activeState.exitCondition.getAsBoolean()){
                //Transition
            }
        }

    }

    /** Set the state and wait for completion */
    public Command setWait(MyBotStates state){
        return setRun(state).until(stateMap.get(state).exitCondition);
    }

    /** Set the state and wait indefinitely */
    public Command setRun(MyBotStates state){
        return set(state).andThen(Commands.idle());
    }

    /** Run the state's command and proceed, ignoring when or how it completes
     * Useful for commands sequenced with drivetrain sequences
    */
    public Command set(MyBotStates state){
        this.activeState=stateMap.get(state);
        this.goalState=this.activeState; //TODO Fix this once state transitions work
        return new ScheduleCommand(activeState.commandSupplier.get());
    }

    /** Wait for the currently set goal state to be reached */
    public Command await(){
        return Commands.waitUntil(()->activeState==goalState && goalState.exitCondition.getAsBoolean());
    }

    public void addState(BotState<MyBotStates> state){
        if(activeState==null) activeState = state;
        if(goalState==null) activeState = state;
        stateMap.put(state.name, state);
    }


}
