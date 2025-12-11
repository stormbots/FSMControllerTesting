package frc.robot.DijkstraV2;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class FSM2<T extends Enum<T>> implements Sendable{
    /** Initial state the FSM boots into, and used for resets */
    private T initialState;
    /** current state mode within the FSM */
    private T stateName;
    /** The state to transition into */
    private T nextState;
    /** Placeholder for the current state object, which is updated according to {@link #stateName}*/
    private State<T> state;
    private HashMap<T,State<T>> stateMap=new HashMap<>();
    private double stateEntryTime;


    /** Transition with a simple condition */
    static class Transition<V extends Enum<V>>{
        V destination;
        BooleanSupplier condition;
    }
    /** Transition where the condition is provided the runtime of the current state (in seconds) */
    static class TimedTransition<V extends Enum<V>>{
        V destination;
        Function<Double,Boolean> condition;
    }

    /** Class containing state logic and transitions. */
    static class State<U extends Enum<U>>{
        /** The core logic of the state */
        Runnable logic = ()->{};
        /** Transition conditions that check external logic  */
        ArrayList<Transition<U>> transitions = new ArrayList<>();
        /** Transition conditions that can interface with a state timer  */
        ArrayList<TimedTransition<U>> timedTransitions = new ArrayList<>();
        /** Pass in a switch style state selector */
        ArrayList<Supplier<U>> selectors = new ArrayList<>();
        /** Items to run when switching to a state */
        ArrayList<Runnable> onEntry = new ArrayList<>();
        /** Items to run when switching out of a state */
        ArrayList<Runnable> onExit = new ArrayList<>();

        public State<U> addTransition(U destinationState, BooleanSupplier transitionCondition){
            transitions.add(new Transition<>(){{
                destination=destinationState;
                condition=transitionCondition;
            }});
            return this;
        }
        /** Add a state transition that interacts with the current state runtime (in seconds) */
        public State<U> addTimerTransition(U destinationState, Function<Double,Boolean> transitionCondition){
            timedTransitions.add(new TimedTransition<U>(){{
                destination=destinationState;
                condition=transitionCondition;
            }});
            return this;
        }
        /** Add a function that directly computes the next state*/
        public State<U> addSelector(Supplier<U> selectorFunction){
            selectors.add(selectorFunction);
            return this;
        }
        //** Add a function that fires when entering the state */
        public State<U> onEntry(Runnable logic){
            onEntry.add(logic);
            return this;
        }
        //** Add a function that fires when exiting the state */
        public State<U> onExit(Runnable logic){
            onExit.add(logic);
            return this;
        }
    }

    /** Create a new FSM using the provided default state and enum type */
    public FSM2(T initialState){
        this.initialState = initialState;
        stateName = initialState;
        nextState = initialState;
        //Stuff an empty state in for now with no transitions and no logic. 
        state=new State<>();
        SmartDashboard.putData(String.format("FSM::%s",initialState.getClass().getSimpleName()),this);
        var kind = initialState.getClass();
    }

    /** Step through the state machine logic, updating as indicated by state transitions 
     * Should be run once per robot loop while robot is enabled
    */
    public void update(){
        state.logic.run();

        //Determine our next state
        for(var transition : state.transitions){
            if(transition.condition.getAsBoolean()){
                nextState=transition.destination;
            }
        }
        for(var transition : state.timedTransitions){
            if(transition.condition.apply(Timer.getFPGATimestamp()-stateEntryTime)){
                nextState=transition.destination;
            }
        }
        for(var selector : state.selectors){
            var newstate=selector.get();
            if(newstate == null) break; //TODO log as error?
            nextState=newstate;
        }

        // Verify errors, set new state
        if(stateMap.containsKey(nextState)==false) return; //TODO log as error

        if(nextState!=stateName){
            for(var exit : state.onExit) exit.run();

            System.out.printf("%s::%s->%s (%.2fs)\n",
                stateName.getClass().getSimpleName(),
                stateName,
                nextState,
                Timer.getFPGATimestamp()-stateEntryTime
            );
            state=stateMap.get(nextState);
            stateName=nextState;

            for(var entry : state.onEntry) entry.run();

            stateEntryTime=Timer.getFPGATimestamp();
        }
        state=stateMap.get(nextState);
    }

    /** Create a new state, returning it so that transitions can be added */
    public State<T> addState(T name, Runnable stateLogic){
        var newstate=new State<T>(){{logic=stateLogic;}};
        stateMap.put(name, newstate);
        return newstate;
    }

    /**
     * TODO Test me!
     * Create a state using a Command, ignoring it's typical requirements and 
     * exit condition.
     * </p>
     * This is what you want when the FSM itself requires all controlled hardware,
     * and conflicting Commands will halt the FSM, or simply do not exist.</p>
     * See also {@link #addState(Enum, Enum, Command)} to create a state that 
     * does transition.
     * @param name
     * @param stateCommand
     * @return
     */
    public State<T> addState(T name, Command stateCommand){
        var newstate=new State<T>(){{
            //Run the state logic
            super.onEntry.add(stateCommand::initialize);
            super.logic=stateCommand::execute; //The state controls it's own logic
            super.onExit.add(stateCommand::cancel);
            // super.addTransition(null, stateCommand::isFinished);
        }};
        stateMap.put(name, newstate);
        return newstate;
    }

    /**
     * TODO Test me!
     * <p>Create a state using a Command, ignoring it's typical requirements. 
     * When the command completes, will transition to the destination state.
     * </p>
     * <p>This is what you want when using FSM is the only exposed
     * "control interface" for subsystem commands, and not actuators.</p>
     * See also {@link #addState(Enum, Enum, Command)} to create a state that 
     * does transition.
     * @param name
     * @param destinationState
     * @param stateCommand
     * @return
     */
    public State<T> addState(T name, T destinationState, Command stateCommand){
        var newstate=new State<T>(){{
            super.onEntry.add(stateCommand::initialize);
            super.logic=stateCommand::execute; //The state controls it's own logic
            super.onExit.add(stateCommand::cancel);
            super.addTransition(destinationState, stateCommand::isFinished);
        }};
        stateMap.put(name, newstate);
        return newstate;
    }

    /** <p>Add a state, running a Command as it's state operation, meaning it runs 
     * with it's standard requirements.</p>
     * <p>Note: this will cause conflicts if the FSM <i>and</i> external commands both
     * require the same subsystem!</p> 
     * <p>Intended for use when the FSM is not a controlled resource, but state resources 
     * might be, such as for superstructure state machines </p> 
     * <p>If the provided state is cancelled, the FSM will not restart the command!</p> 
     * @param name
     * @param destinationState the state to transition to on successful completion of command
     * @param stateCommand 
     * @return
     */
    public State<T> addStateWithRequirements(
        T name,
        T destinationState,
        Command stateCommand
    ){
        var newstate=new State<T>(){{
            super.onEntry.add(()->stateCommand.schedule());
            // super.onEntry.add(()->stateCommand.initialize());  //State handles it
            super.logic=()->{}; //The state controls it's own logic
            super.onExit.add(stateCommand::cancel); // Cancel the command if it didn't exit normally
            super.addTransition(destinationState, stateCommand::isFinished);
        }};
        stateMap.put(name, newstate);
        return newstate;
    }


    /** Return the indicated state, or an empty state if one was not created yet */
    public State<T> getState(T name){
        return stateMap.getOrDefault(name, new State<>());
    }

    /** Create several transitions at once using a single condition. <br/>
     * Useful for external inputs such as buttons that might interact with several states, 
     * or for seperating internal/external logic in your state machine setup 
     * @param destinationState The state you want to transition to
     * @param transitionCondition The signal that you're looking for
     * @param signalAcceptingStates The states states that should respond to the signal
     * @return
     */
    FSM2<T> addSignalTransition(T destinationState, BooleanSupplier transitionCondition, T... signalAcceptingStates){
        for(var a : signalAcceptingStates){
            if(stateMap.containsKey(a)==false){
                throw new Error(String.format("Adding Signal Transition to %s failed due to undeclared state %s",destinationState,a));
            }
        }
        for(var a : signalAcceptingStates){
            stateMap.get(a).addTransition(destinationState, transitionCondition);
        }
        return this;
    }

    //////////////////////////
    /// General usage methods
    //////////////////////////

    /** Return the current operating state */
    public T getState(){
        return stateName;
    }

    /** Set the state machine's state to the indicated value */
    public void setState(T name){
        nextState=name;
    }

    /** Returns true if FSM is in one of the provided states */
    public boolean isInState(T... names){
        for(T tag : names){
            if( stateName==tag )return true;
        }
        return false;
    }

    /**
     * Convenience function to check if a specific state is in a list of states.
     * @param query A state of interest
     * @param names The list of states to match against
     * @return
     */
    public boolean isStateInList(T query, T... names){
        for(T tag : names){
            if( query==tag )return true;
        }
        return false;
    }
    

    /** Validate the state machine's completeness. <br/>
     * Does nothing if all states are defined. <br/>
     * If any states are missing, will throw an exception, halting robot code.<br/>
     * Recommended to include after declaring states to ensure all cases are handled.
     */
    public void validate(){
        String report="";

        for(var tag : stateName.getClass().getEnumConstants()){
            if(stateMap.containsKey(tag)==false){
                report+=String.format("State definition missing %s \n", tag);
                continue;
            }
        }

        if(report.length()==0)return;
        //Hard fault in this case.
        throw(new Error(report));
    }


    /////////////////////////////////////
    /// FRC Utility Functions below
    /////////////////////////////////////

    /** Allows the class to be directly represented on the NetworkTables to access debug information
     * 
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        //Don't blame me for this it's an absolute mess since java just _cannot_ properly handle generic enums
        
        //Prepare objects into necessary format for fetching and conversion
        HashMap<String,T> map = new HashMap<>();
        for(var tag : initialState.getClass().getEnumConstants()){
            map.put(tag.name(),(T)tag);
        }
        final String[] list = map.values().stream().map((v)->v.toString()).toList().toArray(new String[0]);

        System.out.println("It's the map");
        System.out.println(map);

        //Actualy just add them to the sendable
        builder.setActuator(true);
        builder.setSafeState(()->setState(initialState));
        builder.addStringProperty("State", 
            ()->stateName.toString(),
            (s)->nextState=(T)map.getOrDefault(s, initialState)
        );
        builder.addDoubleProperty("Time", 
            ()->Timer.getFPGATimestamp()-stateEntryTime, 
            (time)->stateEntryTime=Timer.getFPGATimestamp()+time
        );
        builder.addStringArrayProperty("State List", 
        ()->list,
        null);
    }

    /** Create a SelectableChooser you can include on the dashboard.
     * Selecting one of the indicated states will change the FSM state immediately
     * @return
      */
    public SendableChooser<T> getSelectableChooser(){
        SendableChooser<T> chooser = new SendableChooser<>();
        chooser.setDefaultOption(initialState.toString(), initialState);
        for(var tag : initialState.getClass().getEnumConstants()){
            if(tag==initialState)continue;
            chooser.addOption(tag.toString(), (T)tag);
        }
        chooser.onChange(this::setState);
        return chooser;
    }

    
}
