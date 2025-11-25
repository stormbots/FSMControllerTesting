package frc.robot.DijkstraV2;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class FSM2<T extends Enum<T>>{

    enum InternalState{
        pathThroughState,
        pathEnteringState,
        pathComplete,

        enteringState,
        runningState,

        disabled
    };
    InternalState fsmstate;

    State state;

    InternalState getNextState(){return InternalState.runningState;}

    void runStates(){
        // state.logic();
        //Now figure out what we're actually doing
        switch(fsmstate){
        //The pathing states
        case pathThroughState:
            if(state.isTransitionComplete.getAsBoolean()) fsmstate=getNextState();
        case pathEnteringState:
            if(state.isGoalComplete.getAsBoolean()) fsmstate=InternalState.pathComplete;
        case pathComplete:
            // Ignore transitions and stay here until commands done?
            // if(no fsm commands running) fsmstate=runningState;
            // If needed, verify things are scheduled?
            
        //The FSM States
        case enteringState:
            for(Transition transition : state.whenInState){
                if(transition.condition.getAsBoolean()) setGoalState(transition.destination);
            }
            if(state.isGoalComplete.getAsBoolean()) fsmstate=InternalState.runningState;
        case runningState:
            for(Transition transition : state.whenInState){
                if(transition.condition.getAsBoolean()) setGoalState(transition.destination);
            }
            for(Transition transition : state.whenGoalComplete){
                if(transition.condition.getAsBoolean()) setGoalState(transition.destination);
            }
            // if(transition.stateSelector) nextState=transition.stateselector();

        }
    }

    void setGoalState(T newState){
    }


    class Transition{
        T destination;
        BooleanSupplier condition;
    }

    class State{
        Runnable logic;
        //Path values
        BooleanSupplier isTransitionComplete; //maybe rename canTransition?
        BooleanSupplier isGoalComplete;

        boolean recalculateTransitionCosts;
        //Standard Logic
        ArrayList<Transition> whenInState;
        ArrayList<Transition> whenGoalComplete;
        //Pass in a switch
        ArrayList<Supplier<T>> stateSelector;
    }

}
