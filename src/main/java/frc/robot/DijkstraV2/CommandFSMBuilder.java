package frc.robot.DijkstraV2;

import java.util.HashMap;

import frc.robot.DijkstraV2.CommandFSM.DisabledAction;

public class CommandFSMBuilder<T extends Enum<T>> {
    T initialState;
    DijkstraV2<T> router;
    HashMap<T,CommandFSMState<T>> states=new HashMap<>();
    DisabledAction disabledAction = DisabledAction.retain;


    public CommandFSMBuilder(T initialState){
        this.initialState=initialState;
        this.router=new DijkstraV2<>(this.initialState);
    }
    
    public CommandFSMBuilder<T> addDirectedConnections(double cost, T... states){
        router.addDirectionalSequence(cost, states);
        return this;
    }

    public CommandFSMBuilder<T> addBidirectionalConnections(double cost, T... states){
        router.addBidirectionalSequence(cost, states);
        return this;
    }

    public CommandFSMBuilder<T> setDisabledAction(DisabledAction action){
        this.disabledAction = action;
        return this;
    }

    /** Verify that the state machine is in a good state; Prints a report and crashes
     * immediately if anything is wrong.
     */
    public CommandFSMBuilder<T> validate(){
        for(var t : initialState.getClass().getEnumConstants()){
            //does state exist?
            //Are any states unreachable?
        }
        return this;
    }

    /** Add the State to the FSM; You probably want to use 
     * {@link #CommandFSMBuilder(Enum)} for generating that state.
     * @param state
     * @return
     */
    public CommandFSMBuilder<T> addState(CommandFSMState<T> state){
        states.put(state.id, state);
        return this;
    }

    public CommandFSM<T> build(){
        //Validate to make sure all states declared
        //iterate through state combinations as a warm up
        return new CommandFSM<T>(initialState);
    }


}
