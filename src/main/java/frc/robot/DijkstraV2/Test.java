package frc.robot.DijkstraV2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;

public class Test {

    public Test(){
        System.out.println("Starting coral handler");
        // setUpCoralHandler();
        djtest();

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
