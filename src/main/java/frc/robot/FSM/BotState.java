package frc.robot.FSM;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class BotState<T extends Enum<T>>{
    public Supplier<Command> commandSupplier = ()->new InstantCommand();
    public BooleanSupplier exitCondition=()->false;

    //This works! 
    T name;
    // public BotState(T name, Command command, BooleanSupplier exitCondition){
    // which would then go with 
    // new BotState<SpecificBotState>(SpecificBotState.thing,
    
    
    /**
     * Provide a state with name, command, and exit conditions.
     * @param name
     * @param commandSupplier
     * @param exitCondition
     */
    public BotState(T name, Supplier<Command> commandSupplier, BooleanSupplier exitCondition){
        System.out.println(name.toString());
        this.name = name;
        this.commandSupplier = commandSupplier;
        this.exitCondition = exitCondition;
    }

    // public BotState<T> withName(String name){
    //     this.name=name;
    //     return this;
    // }
}
