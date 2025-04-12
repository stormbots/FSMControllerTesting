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

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Dijkstra;

public class FSM<T extends Enum<T>>  implements Sendable{

    HashMap<T,FSMState<T>> stateMap = new HashMap<>();
    FSMState<T> activeState;
    // FSMState<T> goalState;
    private Command activeCommand = Commands.none();
    private boolean running=false;
    private T initialState;

    Dijkstra<T> stateRouter = new Dijkstra<>();
    Deque<T> statePath=new ArrayDeque<>();

    public Trigger isAtGoalState=new Trigger(()->
        statePath.isEmpty() && this.activeState!=null && this.activeState.exitCondition.getAsBoolean()
    );

    public FSM(T initialState){
        //This is just to prevent potential null references throughout the code.
        this.initialState=initialState;
        this.activeState=new FSMState<T>(initialState, Commands::none, ()->false);

        new Trigger(DriverStation::isEnabled)
        .whileTrue(Commands.run(this::manageStates)
            .beforeStarting(()->this.running=true)
            .finallyDo(()->this.running=false)
        );
    }


    public void manageStates(){
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


    /** Set the state and wait for completion. Good for sequencing. */
    public Command setWait(T state){
        return setRun(state).until(isAtGoalState);
    }

    /** Set the state and wait indefinitely; Good for buttons.*/
    public Command setRun(T state){
        //TODO Optimize: If current state is between returned "active node" and current "target node",
        // skip returning to the current node.

        return Commands.startRun(
            ()->{
                this.statePath = stateRouter.computeCosts(activeState.name, state);
                this.activeCommand.cancel();
                this.activeState = stateMap.get(statePath.peek());
                this.activeCommand = activeState.commandSupplier.get();
                this.activeCommand.schedule();
            },
            ()->{}
        );
    }

    /** Set the target state and return, ignoring when or how it completes.
     * Useful for setting up parallel sequencing without having to use deadlines 
    */
    public Command setAsync(T state){
        return setRun(state).until(()->true);
    }

    /** Wait for the currently set goal state to be reached. Pairs well with setAsync. */
    public Command await(){
        return Commands.waitUntil(isAtGoalState);
    }

    /** Forcibly set a state change, bypassing state pathing. */
    public Command forceState(T state){
        return Commands.runOnce(()->{
            statePath.clear();
            this.activeState=stateMap.get(state);
            this.activeCommand = activeState.commandSupplier.get();
            this.activeCommand.schedule();
        });
    }


    /**
     * Add an externally defined FSMState directly.
     * @param state
     * @return
     */
    public FSM<T> addState(FSMState<T> state){
        if(activeState==null) activeState = state;
        if(state.name==initialState) activeState = state;
        stateMap.put(state.name, state);

        stateRouter.addNode(state.name);
        return this;
    }

    /**
     * Add a new State, containing the minimial state actions. 
     * Generally, provided commands should not end directly, but instead provide a completionSupplier
     * which will handle transition logic. 
     * 
     * The CompletionSupplier should indicate that the state's primary work is complete. When the state is 
     * an intermediate state, this indicates to the manager that the next state can be transitioned to.
     * When a goal or target state is complete, FSM based commands like {@link #setWait(Enum)}
     * will terminate, allowing progression of sequences. 
     * 
     * @param name
     * @param commandSupplier
     * @param completionSupplier
     * @return
     */
    public FSM<T> addState(T name, Supplier<Command> commandSupplier, BooleanSupplier completionSupplier){
        addState(new FSMState<T>(name,commandSupplier,completionSupplier));
        return this;

    }

    public FSM<T> connect(T state1, T state2, double cost, boolean bidirectional){
        stateRouter.addConnection(state1, state2, cost, bidirectional);
        return this;
    }

    public FSM<T> connect(T state1, T state2, double cost){
        connect(state1, state2, cost,true);
        return this;
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
            // System.out.println(name.toString());
            this.name = name;
            this.commandSupplier = commandSupplier;
            this.exitCondition = exitCondition;
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

    @Override
    public void initSendable(SendableBuilder builder) {
        Supplier<T> goalState=()->{
            if(stateMap.isEmpty())return activeState.name;
            return statePath.peek();
        };

        builder.addBooleanProperty("Goal Complete", isAtGoalState, null);
        builder.addStringProperty("Current State", activeState.name::toString, null);
        builder.addStringProperty("Goal State", goalState.get()::toString, null);
        builder.addBooleanProperty("running", ()->this.running, null);
    }

}
