package frc.robot.DijkstraV2;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DijkstraV2.CommandFSM.DisabledAction;

public class CommandFSMStateBuilder<T extends Enum<T>>{
    private T id;
    private Command command;
    private BooleanSupplier traversalComplete;
    private BooleanSupplier goalComplete;

    public CommandFSMStateBuilder(T id){
    }

    public CommandFSMStateBuilder<T> addCommand(Command command){
        this.command=command;
        return this;
    }

    /** Set the completion check for this 
     * {@link #traversalComplete} and {@link #goalComplete}
     */
    public CommandFSMStateBuilder<T> addCompletion(BooleanSupplier supplier){
        this.traversalComplete=supplier;
        this.goalComplete=this.traversalComplete;
        return this;
    }
    
    /** Set the check that indicates 
     */
    public CommandFSMStateBuilder<T> addTraversalCompletion(BooleanSupplier supplier){
        this.traversalComplete=supplier;
        return this;
    }
    
    public CommandFSMStateBuilder<T> addGoalCompletion(BooleanSupplier supplier){
        this.goalComplete=supplier;
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
