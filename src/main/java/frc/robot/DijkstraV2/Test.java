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

    public Test(){
        System.out.println("Starting coral handler");
        // setUpCoralHandler();
        djtest();
    }

    enum CS{
        intaking,
        alignReverse,
        alignForward,
        loaded,
        unloaded,
        scoring
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

        coral.validate();

        //External API can be defined by checking groups of states
        Trigger isCoralLoaded=new Trigger(()->coral.inState(CS.unloaded)).negate();
        Trigger isCoralLoading=new Trigger(()->coral.inState(CS.alignForward,CS.alignReverse));
        Trigger isCoralEmpty=new Trigger(()->coral.inState(CS.unloaded));

        new Trigger(DriverStation::isEnabled).whileTrue(new RunCommand(()->coral.run()).ignoringDisable(true));

        SmartDashboard.putData("FSM::coral",coral.getSelectableChooser());
    }


    enum Bowtie{
        a,b,c,d,e,f,oneway
    }
    void djtest(){
        var dj = new DijkstraV2<Bowtie>(Bowtie.a);

        dj.connect(Bowtie.a, Bowtie.b, 1);
        dj.connect(Bowtie.b, Bowtie.c, 1);
        dj.connect(Bowtie.c, Bowtie.a, 1);

        dj.connect(Bowtie.d, Bowtie.e, 1);
        dj.connect(Bowtie.e, Bowtie.f, 1);
        dj.connect(Bowtie.f, Bowtie.d, 1);

        dj.connect(Bowtie.c, Bowtie.d, 1);
        dj.connect(Bowtie.d, Bowtie.c, 1);
        dj.connect(Bowtie.f, Bowtie.oneway, 1);

        dj.compute(Bowtie.a, Bowtie.c);
        dj.compute(Bowtie.c, Bowtie.a);
        dj.compute(Bowtie.a, Bowtie.f);
        dj.compute(Bowtie.c, Bowtie.d);
        dj.compute(Bowtie.f, Bowtie.a);
        dj.compute(Bowtie.b, Bowtie.oneway);
        dj.compute(Bowtie.oneway, Bowtie.b);


    }
}
