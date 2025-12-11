package frc.robot.DijkstraV2;

import java.lang.Thread.State;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class MockArmWithCommands extends SubsystemBase{
    SparkFlex arm = new SparkFlex(13, MotorType.kBrushless);
    SparkFlex elevator = new SparkFlex(14, MotorType.kBrushless);

    enum ArmState{
        unhomed(0,90),
        stow(10,90),
        l1(20,-10),
        l2(20,0),
        l3(30,0),
        l4(40,0);
        public double height;
        public double angle;
        private ArmState(double h,double a){this.height=h;this.angle=a;}

    }
    private ArmState currentState = ArmState.unhomed;
    private ArmState goalState = ArmState.unhomed;
    
    FSM2<ArmState> fsm = new FSM2<>(ArmState.unhomed);

    public MockArmWithCommands(){
        configFSM();
    }

    public Command setPos(ArmState state){
        //Placeholder operation
        return Commands.run(()->{});
    }

    public Trigger isAtGoalPosition = new Trigger(()->true);


    private void configFSM(){
        fsm.addState(ArmState.unhomed, Commands.sequence(
            Commands.run(()->{/* set arm upward */}),
            Commands.run(()->{/* run motor down */})
        ))
        .addTransition(ArmState.stow, isAtGoalPosition)
        ;
        fsm.addState(ArmState.stow, setPos(ArmState.stow));
        fsm.addState(ArmState.l1, setPos(ArmState.l1));
        fsm.addState(ArmState.l2, setPos(ArmState.l2));
        fsm.addState(ArmState.l3, setPos(ArmState.l3));
        fsm.addState(ArmState.l4, setPos(ArmState.l4));
    }

    //WANT L4
    //l1

    DijkstraV2<ArmState> router = new DijkstraV2<>(ArmState.unhomed)
    .addBidirectionalSequence(1, 
        ArmState.stow,
        ArmState.l1,
        ArmState.l2,
        ArmState.l3,
        ArmState.l4
    );
    private ArmState handleTransitionsPathing(){
        //We only update states once we've reached our  goal position
        if(isAtGoalPosition.getAsBoolean()==false){return currentState;}

        //Update our state router to figure out where we're going
        router.findPath(currentState, goalState);

        //And, set that as the current state
        return router.popNextState(currentState).orElse(currentState);
    }


    // Standard state-oriented notation for state checks
    private ArmState handleTransitionsStandardFSM(){
        //Not ready to transition
        if(isAtGoalPosition.getAsBoolean()==false){return currentState;}
    
        //This is easier to reason about (and very standard way to check transitions, 
        // as they're in the state logic as it were), but feels very clunky
        switch(currentState){
            case l1:
                //Check above, move to next state
                if(fsm.isStateInList(goalState,ArmState.l1,ArmState.l2,ArmState.l3, ArmState.l4)) 
                    return ArmState.l2 ;
                break;
            case l2:
                //Check above, move to next state
                if(fsm.isStateInList(goalState, ArmState.l3, ArmState.l4)) return ArmState.l3 ;
                //Check below, move to next state
                if(fsm.isStateInList(goalState, ArmState.stow, ArmState.l1)) return ArmState.l1 ;
                break;
            case l3:
                //etc
                break;
            case l4:
            break;
            case stow:
                break;
            case unhomed:
                break;
            default:
                break;
        }

        return ArmState.unhomed;
    }

    //Transition handler nicked from SK-9999's bot, adapted to Reefscape/Skipper
    private ArmState HandleTransitionsSKGoalwiseStyle(){
        //Not ready to transition
        if(isAtGoalPosition.getAsBoolean()==false){return currentState;}
        //
        // I'm not sure how this actually iterates across multi-step transitions
        // Such as going Stow->L2->L3->L4
        switch(goalState){
            case l1:
                if(fsm.isInState(ArmState.stow,ArmState.l2)) return ArmState.l1;
                break;
            case l2:
                if(fsm.isInState(ArmState.l1,ArmState.l3)) return ArmState.l2;
                break;
            case l3:
                if(fsm.isInState(ArmState.l2,ArmState.l4)) return ArmState.l3;
                break;
            case l4:
                if(fsm.isInState(ArmState.l3)) return ArmState.l4;
            break;
            case stow:
                if(fsm.isInState(ArmState.l1)) return ArmState.stow;
                break;
            case unhomed:
                break;
            default:
                break;
        }
        return ArmState.unhomed;
    }


    public void periodic(){

    }
    
}
