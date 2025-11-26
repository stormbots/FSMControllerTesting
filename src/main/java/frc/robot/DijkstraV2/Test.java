package frc.robot.DijkstraV2;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Test {
    enum Toy{a,b,c,d};
    FSM2<Toy> fsm = new FSM2<>(Toy.a);

    public Test(){
        System.out.println("Starting coral handler");
        setUpCoralHandler();
    }

    public void setUpToy(){

        System.out.println("Creating test toy");
        fsm.addState(Toy.a, ()->{})
        // .addTransition(Toy.b, ()->Timer.getFPGATimestamp()%3>1);
        .addTimerTransition(Toy.b, (t)->t>1);

        fsm.addState(Toy.b, ()->{})
        // .addTransition(Toy.c, ()->Timer.getFPGATimestamp()%3<1);
        .addTimerTransition(Toy.c, (t)->t>1)
        .onEntry(()->System.out.print("2B"))
        .onExit(()->System.out.println("...not 2B"))
        ;

        fsm.addState(Toy.c, ()->{})
        // .addTransition(Toy.d, ()->Timer.getFPGATimestamp()%3>1);
        .addTimerTransition(Toy.d, (t)->t>1);

        fsm.addState(Toy.d, ()->{})
        // .addSelector(()->null)
        .addSelector(()->Timer.getFPGATimestamp()%2==0 ? Toy.a : Toy.b);

        fsm.validate();

        new Trigger(DriverStation::isEnabled).onTrue(Commands.run(()->{
            fsm.run();
        }).ignoringDisable(true));

    }
    
    enum CS{
        intaking,
        alignReverse,
        alignForward,
        loaded,
        unloaded,
        scoring,
        disabled,
    }
    void setUpCoralHandler(){
        var motor = new SparkFlex(1, MotorType.kBrushless);
        FSM2<CS> coral = new FSM2<>(CS.unloaded);

        Trigger coralsensor=new Trigger(()->Timer.getFPGATimestamp()%2<0.1);
        Trigger activationButton=new Trigger(()->Timer.getFPGATimestamp()%10<0.1);

        coral.addState(CS.intaking, ()->motor.set(1))
        .addTransition(CS.alignReverse, coralsensor);
        ;
        coral.addState(CS.alignReverse, ()->motor.set(-0.2))
        .addTransition(CS.alignForward, coralsensor.negate());

        coral.addState(CS.alignForward, ()->motor.set(0.2))
        .addTransition(CS.loaded, coralsensor);

        coral.addState(CS.loaded, ()->motor.set(0.0))
        .addTransition(CS.scoring, activationButton)
        // .addTransition(CS.unloaded, coralsensor.negate())//falls out or something
        //loaded handled by button
        ;

        coral.addState(CS.scoring, ()->motor.set(0.2))
        .addTransition(CS.unloaded, coralsensor.negate())
        .addTimerTransition(CS.unloaded, (t)->t>2);

        coral.addState(CS.unloaded, ()->motor.set(0.0))
        .addTransition(CS.alignReverse, coralsensor);

        coral.addState(CS.disabled, ()->motor.set(0.0));

        coral.validate();

        //External API : report status based on states
        // Trigger isCoralLoaded=new Trigger(()->coral.inState(CS.unloaded));
        // Trigger isCoralEmpty=new Trigger(()->coral.inState(CS.unloaded)).negate();

        new Trigger(DriverStation::isEnabled).whileTrue(new RunCommand(()->coral.run()).ignoringDisable(true));

        SmartDashboard.putData("FSM::coral",coral.getSelectableChooser());
    }
}
