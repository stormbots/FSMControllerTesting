package frc.robot.FSM;

import java.util.HashMap;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FSM {

    public enum StateEnum{
        Stow,
        L1,
        IntakeFloor,
        IntakeStation
    }

    HashMap<StateEnum,BotState<StateEnum>> stateMap = new HashMap<>();
    BotState<StateEnum> activeState;
    BotState<StateEnum> goalState;
    private Command activeCommand = Commands.none();
    private boolean running=false;


    public FSM(){

        new RunCommand(this::periodic)
        .withName("FSM Status")
        .ignoringDisable(true)
        .schedule();

        new Trigger(DriverStation::isEnabled)
        .whileTrue(Commands.run(this::manageStates)
            .beforeStarting(()->this.running=true)
            .finallyDo(()->this.running=false)
        );

    }

    private boolean transitioning=true;
    public void manageStates(){
        if(activeState==null || goalState==null) return;

        //Transitioning to goal state
        if(goalState==activeState && this.activeCommand.isScheduled()){
            transitioning = true;
            return;
        }
        //At goal state
        if(goalState==activeState && ! this.activeCommand.isScheduled()){
            transitioning = false;
            this.activeCommand.schedule();
            return;
        }
        //New goal state
        if(goalState != activeState){
            activeState = goalState;
            var newstate = stateMap.get(activeState.name);
            this.activeCommand = newstate.commandSupplier.get().until(newstate.exitCondition);
            this.activeCommand.schedule();
            return;
        }


    }

    public void periodic(){
        SmartDashboard.putString("fsm/currentState", activeState.name.toString());
        SmartDashboard.putString("fsm/goalState", goalState.name.toString());
        SmartDashboard.putBoolean("fsm/done",goalState.exitCondition.getAsBoolean());
        SmartDashboard.putNumber("fsm/timer",(int)Timer.getFPGATimestamp());
        SmartDashboard.putBoolean("fsm/scheduled",this.activeCommand.isScheduled());
        SmartDashboard.putBoolean("fsm/manager_running",this.running);
    }

    /** Set the state and wait for completion */
    public Command setWait(StateEnum state){
        return setRun(state).until(stateMap.get(state).exitCondition);
    }

    /** Set the state and wait indefinitely */
    public Command setRun(StateEnum state){
        return set(state).andThen(Commands.idle());
    }

    /** Run the state's command and proceed, ignoring when or how it completes
     * Useful for commands sequenced with drivetrain sequences
    */
    public Command set(StateEnum state){
        // return Commands.runOnce(()->this.goalState=stateMap.get(state));
        // return new ScheduleCommand(activeState.commandSupplier.get());

        return Commands.sequence(
            Commands.runOnce(()->this.goalState=stateMap.get(state))
            // new ScheduleCommand(activeState.commandSupplier.get())
        );
    }

    /** Wait for the currently set goal state to be reached */
    public Command await(){
        return Commands.waitUntil(()->activeState==goalState && goalState.exitCondition.getAsBoolean());
    }

    public void addState(BotState<StateEnum> state){
        if(activeState==null) activeState = state;
        if(goalState==null) goalState = activeState;
        stateMap.put(state.name, state);
    }
}
