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

public class FSM2<T extends Enum<T>> implements Sendable{
    State<T> state;
    T initialState;
    T stateName;
    T nextState;
    HashMap<T,State<T>> stateMap=new HashMap<>();
    double stateEntryTime;
    FSM2<T> kind; //Attempt to streamline a long annoying line of code i have to use often


    /** Transition with a simple condition */
    class Transition<T>{
        T destination;
        BooleanSupplier condition;
    }
    /** Transition where the condition is provided the runtime of the current state (in seconds) */
    class TimedTransition<T>{
        T destination;
        Function<Double,Boolean> condition;
    }

    /** Class containing state logic and transitions. */
    class State<T>{
        /** The core logic of the state */
        Runnable logic = ()->{};
        /** Transition conditions that check external logic  */
        ArrayList<Transition<T>> transitions = new ArrayList<>();
        /** Transition conditions that can interface with a state timer  */
        ArrayList<TimedTransition<T>> timedTransitions = new ArrayList<>();
        //Pass in a switch
        ArrayList<Supplier<T>> selectors = new ArrayList<>();

        ArrayList<Runnable> onEntry = new ArrayList<>();
        ArrayList<Runnable> onExit = new ArrayList<>();

        public State<T> addTransition(T destinationState, BooleanSupplier transitionCondition){
            transitions.add(new Transition<>(){{
                destination=destinationState;
                condition=transitionCondition;
            }});
            return this;
        }
        /** Add a state transition that interacts with the current state runtime (in seconds) */
        public State<T> addTimerTransition(T destinationState, Function<Double,Boolean> transitionCondition){
            timedTransitions.add(new TimedTransition<T>(){{
                destination=destinationState;
                condition=transitionCondition;
            }});
            return this;
        }
        /** Add a function that directly computes the next state*/
        public State<T> addSelector(Supplier<T> selectorFunction){
            selectors.add(selectorFunction);
            return this;
        }
        //** Add a function that fires when entering the state */
        public State<T> onEntry(Runnable logic){
            onEntry.add(logic);
            return this;
        }
        //** Add a function that fires when exiting the state */
        public State<T> onExit(Runnable logic){
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

    /** Step through the state machine logic, updating as indicated by state transitions */
    void run(){
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
    State<T> addState(T name, Runnable stateLogic){
        var newstate=new State<T>(){{logic=stateLogic;}};
        stateMap.put(name, newstate);
        return newstate;
    }

    /** Return the indicated state, or an empty state if one was not created yet */
    State<T> getState(T name){
        return stateMap.getOrDefault(name, new State<>());
    }

    /** Set the state machine's state to the indicated value */
    void setState(T name){
        nextState=name;
    }

    /** Returns true if FSM is in one of the provided states */
    boolean inState(T... names){
        for(T tag : names){
            if( stateName==tag )return true;
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
