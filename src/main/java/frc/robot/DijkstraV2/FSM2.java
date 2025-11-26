package frc.robot.DijkstraV2;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;

public class FSM2<T extends Enum<T>>{
    State<T> state;
    T stateName;
    T nextState;
    HashMap<T,State<T>> stateMap=new HashMap<>();
    double stateEntryTime;

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

    public FSM2(T initialState){
        stateName = initialState;
        nextState = initialState;
        //Stuff an empty state in for now with no transitions and no logic. 
        state=new State<>();
    }

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

    /** Validate the state machine's completeness. <br/>
     * Does nothing if all states are defined. <br/>
     * If any states are missing, will throw an exception, halting robot code.
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
}
