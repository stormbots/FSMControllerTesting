package frc.robot.DijkstraV2;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.DijkstraV2.CommandFSM.DisabledAction;

public class NewTest {
        
    enum Arm{
        bootup,
        stow,
        climb,
        l1,
        l2,
        l3,
        l4,
        station
    }
    enum Coral{
        intaking,
        alignReverse,
        alignForward,
        loaded,
        unloaded,
        scoring
    }


    CommandFSM<Coral> coral = buildCoralStateMachine();

    private CommandFSM<Coral> buildCoralStateMachine(){
        var builder = new CommandFSMBuilder<>(Coral.unloaded);
        var sensor = new DigitalInput(0);
        var motor = new SparkFlex(11, MotorType.kBrushless);

        builder.setDisabledAction(DisabledAction.reset);

        builder.addDirectedConnections( 1,
            Coral.unloaded,
            Coral.alignReverse,
            Coral.alignForward,
            Coral.loaded,
            Coral.scoring,
            Coral.unloaded
        );
        builder.addDirectedConnections(1, 
            Coral.unloaded,
            Coral.intaking
        );

        builder.addState(
            builder.buildState(Coral.intaking)
            .addCommand(()->motor.set(1))
            .addGoalCompletion(()->sensor.get())
            .addTraversalCompletion(()->true)
            //if we're uncommanded in this state and recieve a coral
            .addTriggeredTransition(Coral.alignReverse, false, ()->sensor.get())
            //if we end up in this state uncommanded, but have no coral
            .addTriggeredTransition(Coral.unloaded, false, ()->sensor.get()==false)
        );


        builder.addState(
            builder.buildState(Coral.unloaded)
            .addCommand(()->motor.set(0))
            .addGoalCompletion(()->false)
            .addTraversalCompletion(()->true)
            .addTriggeredTransition(Coral.alignReverse, false, ()->sensor.get())
        );

        builder.addState(
            builder.buildState(Coral.alignReverse)
            .addCommand(()->motor.set(-0.5))
            .addGoalCompletion(()->sensor.get()==false)
            .addTraversalCompletion(()->true)
            .addTriggeredTransition(Coral.alignForward, true, ()->sensor.get())
        );

        builder.addState(
            builder.buildState(Coral.alignForward)
            .addCommand(()->motor.set(0.2))
            .addGoalCompletion(()->sensor.get())
            .addTraversalCompletion(()->true)
            .addTriggeredTransition(Coral.loaded, true, ()->sensor.get())
        );

        builder.addState(
            builder.buildState(Coral.loaded)
            .addCommand(()->motor.set(0))
            .addGoalCompletion(()->false)
            .addTraversalCompletion(()->true)
            //In case it's dropped or removed
            .addTriggeredTransition(Coral.unloaded, false, ()->sensor.get()==false)
        );

        builder.addState(
            builder.buildState(Coral.scoring)
            .addCommand(()->motor.set(1))
            .addCompletion(()->sensor.get()==false)
            .addTriggeredTransition(Coral.unloaded, true, ()->sensor.get())
        );

        return builder.build();
    }
}
