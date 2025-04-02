package frc.robot.FSM;

import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class FSM<T extends Enum<T>> {

    HashMap<T,FSMState<T>> stateMap = new HashMap<>();
    FSMState<T> activeState;
    FSMState<T> goalState;
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
    public Command setWait(T state){
        return setRun(state).until(stateMap.get(state).exitCondition);
    }

    /** Set the state and wait indefinitely */
    public Command setRun(T state){
        return set(state).andThen(Commands.idle());
    }

    /** Run the state's command and proceed, ignoring when or how it completes
     * Useful for commands sequenced with drivetrain sequences
    */
    public Command set(T state){
        return Commands.runOnce(()->this.goalState=stateMap.get(state));
    }

    /** Wait for the currently set goal state to be reached */
    public Command await(){
        return Commands.waitUntil(()->activeState==goalState && goalState.exitCondition.getAsBoolean());
    }

    public void addState(FSMState<T> state){
        if(activeState==null) activeState = state;
        if(goalState==null) goalState = activeState;
        stateMap.put(state.name, state);
    }

    public void addState(T name, Supplier<Command> command,BooleanSupplier exitSupplier){
        addState(new FSMState<T>(name,command,exitSupplier));
    }


    /** Container for state data.
     */
    public static class FSMState<T extends Enum<T>>{
        public Supplier<Command> commandSupplier = ()->new InstantCommand();
        public BooleanSupplier exitCondition=()->false;
        T name;
        
        /**
         * Provide a state with name, command, and exit conditions.
         * @param name
         * @param commandSupplier
         * @param exitCondition
         */
        public FSMState(T name, Supplier<Command> commandSupplier, BooleanSupplier exitCondition){
            System.out.println(name.toString());
            this.name = name;
            this.commandSupplier = commandSupplier;
            this.exitCondition = exitCondition;
        }
    }

}
