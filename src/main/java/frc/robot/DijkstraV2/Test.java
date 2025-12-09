package frc.robot.DijkstraV2;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.DijkTesting.BuildBuild;
import frc.robot.DijkTesting.InstInst;
import frc.robot.DijkTesting.TestTest;
import frc.robot.DijkTesting.WorkingComboBuilderKinda;
import frc.robot.DijkTesting.WorkingComboBuilderKinda.Instance;

public class Test {

    public Test(){
        System.out.println("Starting coral handler");
        // setUpCoralHandler();
        djtest();
        //This sucks because the first part is nonobvious and ide fails to help
        WorkingComboBuilderKinda<CS>.Instance<CS> instance = new WorkingComboBuilderKinda<CS>().build();//gross
        
        //This works but isn't exactly less clunky
        // InstInst<CS> trial = new BuildBuild<CS>().method().build();
        // InstInst<CS> test2 = new InstInst<>();
        // TestTest<CS> trial = new TestTest.Builder<CS>().set().build();

        // FSMState2<CS> aaa = new FSMState2Builder<CS>().build();
        var armcom = new CommandFSMBuilder<>(Arm.stow);
        armcom.addBidirectionalConnections(1,Arm.stow,Arm.l1,Arm.l2,Arm.l3);
        armcom.addDirectedConnections(1,Arm.bootup,Arm.stow);

        armcom.addState(new CommandFSMState<>(Arm.stow)
            .addCommand(Commands.idle())
            .addCompletion(()->true)
            .validate()
        );
        //repeat.

        //TODO: Add timed transitions to command

    }

    void setupCoralCommands(){


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

        new Trigger(DriverStation::isEnabled).whileTrue(new RunCommand(()->coral.update()).ignoringDisable(true));

        SmartDashboard.putData("FSM::coral",coral.getSelectableChooser());
    }


    void benchmark(DijkstraV2<Bowtie> path){
        var counts=0;
        var timer=Timer.getFPGATimestamp();

        for(var a : Bowtie.class.getEnumConstants()){
            for(var b : Bowtie.class.getEnumConstants()){
                counts++;
                path.findPath(a, b);
            }
        }
        timer=Timer.getFPGATimestamp()-timer;
        System.out.printf("Calculated %s paths in %.5fs\n", counts,timer);
        System.out.printf("approx %.5fs per path\n", timer/counts);
        System.out.printf("approx %.5f%% of loop time\n", (timer/counts)/0.02);
    }

    enum Bowtie{
        lbl,lbr,lc,ltr,ltl,
        rc,rbl,rbr,rtr,rtl,
        ct,cb
    }
    void djtest(){
        var dj = new DijkstraV2<Bowtie>(Bowtie.rc);

        dj.addBidirectionalSequence(2,Bowtie.lc,Bowtie.ltr,Bowtie.ltl,Bowtie.lbl,Bowtie.lbr,Bowtie.lc);
        dj.addDirectionalSequence(5,Bowtie.lc,Bowtie.cb,Bowtie.rc,Bowtie.ct);
        dj.addDirectionalSequence(1,Bowtie.rc,Bowtie.rtl,Bowtie.rtr,Bowtie.rbr,Bowtie.rbl,Bowtie.rc);

        benchmark(dj);
        benchmark(dj);
        benchmark(dj);

        System.out.println(dj.edges);
    }

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
    void armtest(){
        var dj=new DijkstraV2<>(Arm.bootup);

        dj.addBidirectionalSequence(1, Arm.stow,Arm.l1,Arm.l2,Arm.l3,Arm.l4);
        dj.addBidirectionalSequence(1, Arm.stow, Arm.climb);
        dj.addBidirectionalSequence(1, Arm.stow, Arm.station);

    }
}
