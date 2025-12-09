package frc.robot.DijkstraV2;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DijkstraV2.CommandFSM.DisabledAction;
import frc.robot.DijkstraV2.CommandFSMState.Transition;

public class CommandFSMStateBuilder<T extends Enum<T>>{
    public T id;
    public Command command;
    public BooleanSupplier traversalComplete;
    public BooleanSupplier goalComplete;
    public ArrayList<Transition<T>> whenInState = new ArrayList<>();
    public ArrayList<Transition<T>> whenGoalComplete = new ArrayList<>();
    public Optional<DoubleSupplier> stateRecoveryCost=Optional.empty();


    public CommandFSMStateBuilder(T id){
    }

    /** The Command to execute when this state is active. */
    public CommandFSMStateBuilder<T> addCommand(Command command){
        this.command=command;
        return this;
    }

    /** The command to execute when this state is active. <br/><br/>
     * Syntactic sugar for `addCommand(Commands.run(action, requirements));` <br/><br/>
     * Commands will be forcibly terminated on state changes, so only one
     * state command will run at a time, regardless of requirements provided. 
     */
    public CommandFSMStateBuilder<T> addCommand(Runnable action, Subsystem... requirements){
        addCommand(Commands.run(action, requirements));
        return this;
    }

    /** The function that indicates the completion function for both
     * {@link #traversalComplete} and {@link #goalComplete} conditions. 
     * 
     */
    public CommandFSMStateBuilder<T> addCompletion(BooleanSupplier supplier){
        this.traversalComplete=supplier;
        this.goalComplete=this.traversalComplete;
        return this;
    }
    
    /** During multi-state traversal, this function is used to indicate
     * that the current state's intermediate step is complete, and the FSM can 
     * proceed to the next state.
     */
    public CommandFSMStateBuilder<T> addTraversalCompletion(BooleanSupplier supplier){
        this.traversalComplete=supplier;
        return this;
    }
    
    /** When this state is the goal state, this state is used to indicate a completed 
     * operation. Certain other utilities and triggers will check this condition, 
     * including {@link #whenGoalComplete} triggers and several FSM oriented commands.
     * @param supplier
     * @return
     */
    public CommandFSMStateBuilder<T> addGoalCompletion(BooleanSupplier supplier){
        this.goalComplete=supplier;
        return this;
    }

    /** Add an automated state transition from this state to another. Only checked
     * when this state is the goal state, and not currently commanded by a FSM control
     * command.
     * @param destinationState the state to transition to
     * @param requireGoalCompletion require the goal state's completion condition or not
     * @param condition the condition in which to transition
     * @return
     */
    public CommandFSMStateBuilder<T> addTriggeredTransition(
        T destinationState, 
        Boolean requireGoalCompletion, 
        BooleanSupplier condition
    ){
        Transition<T> t = new Transition<>();
        t.condition=condition;
        t.dest=destinationState;

        if(requireGoalCompletion){
            whenGoalComplete.add(t);
        }
        else{
            whenInState.add(t);
        }
        return this;
    }

    public CommandFSMStateBuilder<T> addRecoveryCost(DoubleSupplier costFunction){
        stateRecoveryCost=Optional.of(costFunction);
        return this;
    }


    /** Validate and return the newly generated state */
    public CommandFSMState<T> build(){
        var name=id.getClass().getSimpleName();
        if(command==null){
            throw(new Error(String.format("Builder %s::%s missing Command data\n", name,id)));
        }
        if(goalComplete==null && traversalComplete==null){
            throw(new Error(String.format("Builder %s::%s has no exit conditions \n", name,id)));
        }
        return new CommandFSMState<T>(
            this.id,
            this.command,
            traversalComplete!=null ? this.traversalComplete : this.goalComplete,
            goalComplete!=null ? this.goalComplete : this.traversalComplete
        );
    }
}
