package frc.robot.DijkstraV2;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.DijkstraV2.CommandFSM.DisabledAction;

public class CommandFSMState<T extends Enum<T>>{
    public static class Transition<U extends Enum<U>>{
        public U dest;
        public BooleanSupplier condition;
    }

    /**  */
    public final T id;
    public Command command;
    public BooleanSupplier traversalComplete;
    public BooleanSupplier goalComplete;
    public ArrayList<Transition<T>> whenInState = new ArrayList<>();
    public ArrayList<Transition<T>> whenGoalComplete = new ArrayList<>();
    public Optional<DoubleSupplier> stateRecoveryCost=Optional.empty();

    /** 
     * <p>Create a new State for the Command FSM.</p>
     * <p>This minimally needs two methods</p>
     * <li>{@link #addCommand(Command)}</li>
     * <li>{@link #addCompletion(BooleanSupplier)}</li>
     * in order to work correctly, and will throw errors if forgotten. 
     * Alternatively, the completion can be explicitly split using 
     * <li>{@link #addGoalCompletion(BooleanSupplier)}</li>
     * <li>{@link #addTraversalCompletion(BooleanSupplier)}</li>
     * </p>
     * <p>All methods for this state return itself, allowing for streamlined
     * creation of an object.</p>
     * @param id
     */
    public CommandFSMState(T id){
        this.id = id;
    }

    /** The Command to execute when this state is active. */
    public CommandFSMState<T> addCommand(Command command){
        this.command=command;
        return this;
    }

    /** The command to execute when this state is active. <br/><br/>
     * Syntactic sugar for `addCommand(Commands.run(action, requirements));` <br/><br/>
     * Commands will be forcibly terminated on state changes, so only one
     * state command will run at a time, regardless of requirements provided. 
     */
    public CommandFSMState<T> addCommand(Runnable action, Subsystem... requirements){
        addCommand(Commands.run(action, requirements));
        return this;
    }

    /** The function that indicates the completion function for both
     * {@link #traversalComplete} and {@link #goalComplete} conditions. 
     * 
     */
    public CommandFSMState<T> addCompletion(BooleanSupplier supplier){
        this.traversalComplete=supplier;
        this.goalComplete=this.traversalComplete;
        return this;
    }
    
    /** During multi-state traversal, this function is used to indicate
     * that the current state's intermediate step is complete, and the FSM can 
     * proceed to the next state.
     */
    public CommandFSMState<T> addTraversalCompletion(BooleanSupplier supplier){
        this.traversalComplete=supplier;
        return this;
    }
    
    /** When this state is the goal state, this state is used to indicate a completed 
     * operation. Certain other utilities and triggers will check this condition, 
     * including {@link #whenGoalComplete} triggers and several FSM oriented commands.
     * @param supplier
     * @return
     */
    public CommandFSMState<T> addGoalCompletion(BooleanSupplier supplier){
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
    public CommandFSMState<T> addTriggeredTransition(
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

    /** Adds cost/distance function that can be used to recover the system state 
     * if the state machine is disrupted by external factors. 
     * The smallest value generated during recovery is considered the nearest state.
    */
    public CommandFSMState<T> addRecoveryCost(DoubleSupplier costFunction){
        stateRecoveryCost=Optional.of(costFunction);
        return this;
    }


    /** Validate and return the newly generated state. */
    public CommandFSMState<T> validate(){
        var name=id.getClass().getSimpleName();
        String report="";
        if(command==null){
            report+="missing Command data\n";
        }
        if(goalComplete==null){
            report+="Contains no goal exit conditions\n";
        }

        if(traversalComplete==null){
            report+="Contains no traversal exit conditions\n";
        }

        if(report.isBlank()==false){
            throw(new Error("State %s::%s Contains errors:\n"+report));
        }
        return this;
    }
}
