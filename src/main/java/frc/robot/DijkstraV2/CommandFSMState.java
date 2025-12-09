package frc.robot.DijkstraV2;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class CommandFSMState<T extends Enum<T>>{
    public static class Transition<U extends Enum<U>>{
        public U dest;
        public BooleanSupplier condition;
    }

    /** The enum representing the state's name */
    public final T id;
    /** This command runs during iterations where this is the active state */
    public final Command command;
    /** <p>The condition that indicates transition from a previous state 
     * to this state can be considered complete. Will be referenced while 
     * iterating through states to the goal state.</p>
     */
    public final BooleanSupplier traversalComplete;
    /** <p>The condition that indicates whether the state's goal or main work is complete.</p>
     * <p> Only applies when the state is the FSM Goal state.</p> 
    */
    public final BooleanSupplier goalComplete;

    /** Automated transitioins that can occour when the FSM is not otherwise being commanded */
    public ArrayList<Transition<T>> whenInState = new ArrayList<>();

    /** An automated transition that can occour when the goal is met and the FSM is not being commanded */
    public ArrayList<Transition<T>> whenGoalComplete = new ArrayList<>();
    //Note to self: This allows a distinction between onTrue+onFalse and whileHeld, since one would exit
    //making the system non-commanded!

    /** A cost/distance function that can be used to recover the system state 
     * if the state machine is disrupted by external factors. 
     * The smallest value is considered the nearest state. 
    */
    public Optional<DoubleSupplier> stateRecoveryCost=Optional.empty();

    public CommandFSMState(
        T id,
        Command command,
        BooleanSupplier traversalComplete,
        BooleanSupplier goalComplete
    ){
        this.id=id;
        this.command=command;
        this.traversalComplete=traversalComplete;
        this.goalComplete=goalComplete;
    }
}