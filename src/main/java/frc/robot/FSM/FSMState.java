package frc.robot.FSM;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Container for state data. */
public class FSMState<T extends Enum<T>>{
    public Supplier<Command> commandSupplier = ()->new InstantCommand();
    public BooleanSupplier transitionCompletionCondition=()->false;
    public BooleanSupplier goalCompletionCondition=()->false;
    public T name;
    public Optional<DoubleSupplier> recoveryCost=Optional.empty();

    public class AutoTransition<T>{
        public BooleanSupplier condition;
        public T destination;
        public Boolean requiresCompletion;
        public String toString(){
            return String.format("AT(%s->%s|()->%s|()->%s && %s)",
                name.toString(),
                destination.toString(),
                condition.getAsBoolean(),
                goalCompletionCondition.getAsBoolean(),
                requiresCompletion
            );
        }
    }
    /** List of transitions to check when at the goal state and our state objective is done */
    public ArrayList<AutoTransition<T>> autoWhenComplete = new ArrayList<>();
    /** List of transitions to check when at the goal state */
    public ArrayList<AutoTransition<T>> autoAnytime = new ArrayList<>();

    /**
     * Provide a state with name, command, and exit conditions.
     * @param name
     * @param commandSupplier
     * @param transitionCompletionCondition
     */
    public FSMState(
            T name,
            Supplier<Command> commandSupplier,
            BooleanSupplier transitionCompletionCondition,
            BooleanSupplier goalCompletionCondition
        ){
        this.name = name;
        this.commandSupplier = commandSupplier;
        this.transitionCompletionCondition = transitionCompletionCondition;
        this.goalCompletionCondition = goalCompletionCondition;
    }

    /**
     * Configure an automatic transition from this state to another
     * @param destination
     * @param condition the boolean condition under which we transition
     * @param requiresGoalCompletion Whether we also require the state's completion condition to be met.
     */
    public void addAutoTransition(T destination, BooleanSupplier condition, Boolean requiresGoalCompletion){
        var t = new AutoTransition<T>();
        t.destination = destination;
        t.condition = condition;
        t.requiresCompletion=requiresGoalCompletion;
        autoWhenComplete.add(t);
    }
}
