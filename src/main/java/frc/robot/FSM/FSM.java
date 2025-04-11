package frc.robot.FSM;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Deque;
import java.util.HashMap;
import java.util.Stack;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Dijkstra;

public class FSM<T extends Enum<T>> {

    HashMap<T,FSMState<T>> stateMap = new HashMap<>();
    FSMState<T> activeState;
    FSMState<T> goalState;
    private Command activeCommand = Commands.none();
    private boolean running=false;

    Dijkstra<T> stateRouter = new Dijkstra<>();
    Deque<T> statePath=new ArrayDeque<>();

    public Trigger isAtGoalState=new Trigger(()->
        statePath.isEmpty() && this.activeState!=null && this.activeState.exitCondition.getAsBoolean()
    );

    public FSM(){
        //TODO: Consider forcing an "error" response for null maps, just to prevent confusion

        new RunCommand(this::periodic)
        .withName("FSM Status")
        .ignoringDisable(true)
        .schedule();

        new Trigger(DriverStation::isEnabled)
        .whileTrue(Commands.run(this::manageStatesListwise)
            .beforeStarting(()->this.running=true)
            .finallyDo(()->this.running=false)
        );
    }

    public void manageStates(){
        if(activeState==null || goalState==null) return;

        //Transitioning to goal state
        if(goalState==activeState && this.activeCommand.isScheduled()){
            return;
        }
        //At goal state
        if(goalState==activeState && ! this.activeCommand.isScheduled()){
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

    public void manageStatesListwise(){
        SmartDashboard.putBoolean("fsm/atgoalstate",isAtGoalState.getAsBoolean());

        //TODO Find a better way to handle these.
        if(activeState==null) return;
        //If uninitialized, do something smart, we just can't fallthrough properly otherwise.

        //Walk through our state queue
        //We only need to care about exit condition if we have more states.
        if(activeState.exitCondition.getAsBoolean() && statePath.size()>0){
            System.out.println("Updating to " +statePath.peek());
            activeCommand.cancel();
            activeState = stateMap.get(statePath.poll());
            activeCommand=activeState.commandSupplier.get();
            activeCommand.schedule();
        }
        else if(activeState==null && statePath.size()>0){
            System.out.println("Initial scheduling " + statePath.peek());
            activeCommand.cancel();
            activeState = stateMap.get(statePath.poll());
            activeCommand=activeState.commandSupplier.get();
            activeCommand.schedule();
        }
        else if(activeCommand.isFinished()){
            System.out.println("Permascheduling "+statePath.peek());
            //TODO: Check for automatic state transitions
            //Otherwise this generally shouldn't happen. Reschedule it without exit conditions
            activeCommand = activeState.commandSupplier.get().repeatedly();
            activeCommand.schedule();
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
        // return Commands.runOnce(()->this.goalState=stateMap.get(state));
        return Commands.runOnce(()->statePath = stateRouter.computeCosts(activeState.name, state));
    }

    /** Wait for the currently set goal state to be reached */
    public Command await(){
        // return Commands.waitUntil(()->activeState==goalState && goalState.exitCondition.getAsBoolean());
        return Commands.waitUntil(isAtGoalState);
    }

    public Command pathToState(T state){
        //TODO Optimize: If current state is between returned "active node" and current "target node",
        // skip returning to the current node.
        return 
            Commands.startRun(
                ()->this.statePath = stateRouter.computeCosts(activeState.name, state),
                ()->{}
            ).until(isAtGoalState);
    }

    /** Forcibly set the current state, bypassing and clearing the state pathing */
    public Command forceState(T state){
        return Commands.runOnce(()->{
            statePath.clear();
            this.activeState=stateMap.get(state);
            this.activeCommand = activeState.commandSupplier.get();
            this.activeCommand.schedule();
        });
    }


    public FSM<T> addState(FSMState<T> state){
        if(activeState==null) activeState = state;
        if(goalState==null) goalState = activeState;
        stateMap.put(state.name, state);

        stateRouter.addNode(state.name);
        return this;
    }

    public FSM<T> addState(T name, Supplier<Command> command,BooleanSupplier exitSupplier){
        addState(new FSMState<T>(name,command,exitSupplier));
        return this;

    }

    public FSM<T> connect(T state1, T state2, Double cost, boolean bidirectional){
        stateMap.get(state1).addConnection(state2, cost);
        if(bidirectional) stateMap.get(state2).addConnection(state1, cost);
        stateRouter.addConnection(state1, state2, cost, bidirectional);
        return this;
    }

    public FSM<T> connect(T state1, T state2, Double cost){
        connect(state1, state2, cost,true);
        return this;
    }


    /** Container for state data.
     */
    public static class FSMState<T extends Enum<T>>{
        public Supplier<Command> commandSupplier = ()->new InstantCommand();
        public BooleanSupplier exitCondition=()->false;
        T name;
        public HashMap<T,Double> connections = new HashMap<>();

        /**
         * Provide a state with name, command, and exit conditions.
         * @param name
         * @param commandSupplier
         * @param exitCondition
         */
        public FSMState(T name, Supplier<Command> commandSupplier, BooleanSupplier exitCondition){
            // System.out.println(name.toString());
            this.name = name;
            this.commandSupplier = commandSupplier;
            this.exitCondition = exitCondition;
        }

        public FSMState addConnection(T otherstate, Double cost){
            connections.put(otherstate,cost);
            return this;
        }
    }

    //TODO: Missing useful items
    // optional "transition command" to be used in place of decorating initial command: Jack in the bot uses this for sequentially moving arm and reversing it
    // Auto-transition to state when executed as target state and done 
    //"distance" function for the FSM to attempt state recovery
    //Automatically build a SendableChooser start->finish and show the paths so it's easy to proofread

    //Have the path tracker take <T enum, N node> and then do a thin wrapper to convert N to an internal type.
    // Then it can have an interface for changing strategies, and return a list of N without having to 
    // care about's actual implementation


}
