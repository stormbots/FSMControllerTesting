package frc.robot.DijkstraV2;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Test {
    enum Toy{a,b,c,d};
    FSM2<Toy> fsm = new FSM2<>(Toy.a);

    public Test(){
        System.out.println("Creating test toy");
        fsm.addState(Toy.a, ()->{})
        // .addTransition(Toy.b, ()->Timer.getFPGATimestamp()%3>1);
        .addTimerTransition(Toy.b, (t)->t>1);

        fsm.addState(Toy.b, ()->{})
        // .addTransition(Toy.c, ()->Timer.getFPGATimestamp()%3<1);
        .addTimerTransition(Toy.c, (t)->t>1);

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
    
}
