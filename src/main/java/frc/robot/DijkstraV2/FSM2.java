package frc.robot.DijkstraV2;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FSM2<T extends Enum<T>>{
    State<T> state;
    T stateName;
    T nextState;
    HashMap<T,State<T>> stateMap=new HashMap<>();

    class Transition<T>{
        T destination;
        BooleanSupplier condition;
    }

    class State<T>{
        /** The core logic of the state */
        Runnable logic = ()->{};
        /** Transition conditions that can occour any time in this state  */
        ArrayList<Transition<T>> transitions = new ArrayList<>();
        //Pass in a switch
        ArrayList<Supplier<T>> selectors = new ArrayList<>();

        //TODO: Add onEntry and onExit runnables arrays?

        public State<T> addTransition(T destinationState, BooleanSupplier transitionCondition){
            transitions.add(new Transition<>(){{
                destination=destinationState;
                condition=transitionCondition;
            }});
            return this;
        }
        public State<T> addSelector(Supplier<T> selectorFunction){
            selectors.add(selectorFunction);
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
        for(var selector : state.selectors){
            var newstate=selector.get();
            if(newstate == null) break; //TODO log as error?
            nextState=newstate;
        }

        // Verify errors, set new state
        if(stateMap.containsKey(nextState)==false) return; //TODO log as error

        if(nextState!=stateName){
            System.out.printf("%s::%s->%s\n",
                stateName.getClass().getSimpleName(),
                stateName,
                nextState
            );
            state=stateMap.get(nextState);
            stateName=nextState;
        }
        state=stateMap.get(nextState);
    }

    /** Validate the state machine's completeness. Throw errors if not.*/
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

    State<T> addState(T name, Runnable stateLogic){
        var newstate=new State<T>(){{logic=stateLogic;}};
        stateMap.put(name, newstate);
        return newstate;
    }

    /** Grab the state for the provided value.*/
    State<T> getState(T name){
        return stateMap.getOrDefault(name, new State<>());
    }

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
