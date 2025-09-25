package frc.robot.FSM;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Dijkstra;

public class FSM<T extends Enum<T>>  implements Sendable{

    HashMap<T,FSMState<T>> stateMap = new HashMap<>();
    FSMState<T> activeState;
    // FSMState<T> goalState;
    public Command activeCommand = Commands.none();
    private boolean running=false;
    private T initialState;

    Dijkstra<T> stateRouter = new Dijkstra<>();
    Deque<T> statePath=new ArrayDeque<>();

    /** This exists to efficiently convert Strings back to state names */
    private Map<String,Enum<T>> stringMap = new HashMap<>();


    public Trigger isAtGoalState=new Trigger(()->
        statePath.isEmpty() && this.activeState!=null && this.activeState.exitCondition.getAsBoolean()
    );

    public FSM(T initialState){
        //This is just to prevent potential null references throughout the code.
        //Adding the state definition later will overwrite this. 
        this.initialState=initialState;
        this.activeState=new FSMState<T>(initialState, Commands::none, ()->false);

        new Trigger(DriverStation::isEnabled)
        .whileTrue(Commands.run(this::manageStates)
            .beforeStarting(()->this.running=true)
            .finallyDo(()->this.running=false)
        );

        SendableChooser<T> chooser = new SendableChooser<>();
        chooser.setDefaultOption(initialState.toString(), initialState);

        System.out.println("Building string map");
        for(var s: initialState.getClass().getEnumConstants()){
            System.out.println(s);
            stringMap.put(s.toString(), (T)s);
            chooser.addOption(s.toString(), (T)s);
        }
        System.out.println(stringMap);
        SmartDashboard.putData(this.toString()+"/chooser",chooser);

    }


    public void manageStates(){
        // SmartDashboard.putBoolean("fsm/atgoalstate",isAtGoalState.getAsBoolean());

        //Walk through our state queue
        //We only need to care about exit condition if we have more states.
        if(activeState.exitCondition.getAsBoolean() && statePath.size()>0){
            System.out.println("Updating to "+statePath.peek());
            activeCommand.cancel();
            activeState = stateMap.get(statePath.poll());
            activeCommand=activeState.commandSupplier.get();
            activeCommand.schedule();
        }
        else if(activeCommand.isScheduled()==false && activeState.autotransitions.size()>0){
            //We have automated transition conditions. Check them. 
            for(var transition: activeState.autotransitions){
                if(transition.condition.getAsBoolean()){
                    System.out.println("Auto Transition from  "+activeState.name +" to " +transition.destination);
                    activeCommand.cancel();//redundant but nonissue
                    activeState = stateMap.get(transition.destination);
                    activeCommand=activeState.commandSupplier.get();
                    activeCommand.schedule();
                    break; //Don't check more conditions; First come first served.
                }
                // else{
                //     System.out.println("Not transitioning from  "+activeState.name +" to " +transition.destination);
                // }
            }
        }
        else if(activeCommand.isScheduled()==false){
            //Goal commands generally shouldn't exit without some automated transition defined.
            //In this case, just reschedule it 
            System.out.println("Permascheduling "+activeState.name);
            //Otherwise this generally shouldn't happen. Reschedule it without exit conditions
            activeCommand = activeState.commandSupplier.get().repeatedly();
            activeCommand.schedule();
        }

        //NOTE TO SELF: command.isFinished() often gets broken by wrapper commands,
        //causing transition failures. Checking isScheduled() works as expected.

    }

    public FSMState<T> getCurrentState(){
        return activeState;
    }

    /** Set the state and wait for completion. Good for sequencing. */
    public Command setWait(T state){
        return setRun(state).until(isAtGoalState);
    }

    /** Set the state and wait indefinitely; Good for buttons.*/
    public Command setRun(T state){
        //TODO Optimize: If current state is between returned "active node" and current "target node",
        // skip returning to the current node.

        return Commands.startRun(
            ()->{
                this.statePath = stateRouter.computeCosts(activeState.name, state);
                this.activeCommand.cancel();
                this.activeState = stateMap.get(statePath.peek());
                this.activeCommand = activeState.commandSupplier.get();
                this.activeCommand.schedule();
            },
            ()->{}
        );
    }

    /** Set the target state and return, ignoring when or how it completes.
     * Useful for setting up parallel sequencing without having to use deadlines 
    */
    public Command setAsync(T state){
        return setRun(state).until(()->true);
    }

    /** Wait for the currently set goal state to be reached. Pairs well with setAsync. */
    public Command await(){
        return Commands.waitUntil(isAtGoalState);
    }

    /** Forcibly set a state change, bypassing state pathing. */
    public Command forceState(T state){
        return Commands.runOnce(()->{
            statePath.clear();
            this.activeState=stateMap.get(state);
            this.activeCommand = activeState.commandSupplier.get();
            this.activeCommand.schedule();
        });
    }


    /**
     * Add an externally defined FSMState directly.
     * @param state
     * @return
     */
    public FSM<T> addState(FSMState<T> state){
        if(state.name==initialState) activeState = state;
        stateMap.put(state.name, state);
        stateRouter.addNode(state.name);
        return this;
    }

    /**
     * Add a new State, containing the minimial state actions. 
     * Generally, provided commands should not end directly, but instead provide a completionSupplier
     * which will handle transition logic. 
     * 
     * The CompletionSupplier should indicate that the state's primary work is complete. When the state is 
     * an intermediate state, this indicates to the manager that the next state can be transitioned to.
     * When a goal or target state is complete, FSM based commands like {@link #setWait(Enum)}
     * will terminate, allowing progression of sequences. 
     * 
     * @param name
     * @param commandSupplier
     * @param completionSupplier
     * @return
     */
    public FSM<T> addState(T name, Supplier<Command> commandSupplier, BooleanSupplier completionSupplier){
        addState(new FSMState<T>(name,commandSupplier,completionSupplier));
        return this;

    }

    /** Connect two states, allowing unidirectional connections */
    public FSM<T> connect(T state1, T state2, double cost, boolean bidirectional){
        stateRouter.addConnection(state1, state2, cost, bidirectional);
        return this;
    }

    /** Connect two states with a bidirectional link */
    public FSM<T> connect(T state1, T state2, double cost){
        connect(state1, state2, cost,true);
        return this;
    }

    public FSM<T> addAutoTransition(T state1, T state2, BooleanSupplier condition){
        var state = stateMap.get(state1);
        if(state==null){
            throw new Error("Initial state not present in known FSM states. Add before setting transitions.");
        }
        state.addAutoTransition(state2,condition);
        return this;
    }

    public FSM<T> addAutoTransition(T state1, T state2){
        addAutoTransition(state1,state2,()->true);
        return this;
    }




    /** Container for state data.
     */
    public static class FSMState<T extends Enum<T>>{
        public Supplier<Command> commandSupplier = ()->new InstantCommand();
        public BooleanSupplier exitCondition=()->false;
        public T name;

        public class AutoTransition<T>{
            public BooleanSupplier condition;
            public T destination;
        }
        public ArrayList<AutoTransition<T>> autotransitions = new ArrayList<>();

        /**
         * Provide a state with name, command, and exit conditions.
         * @param name
         * @param commandSupplier
         * @param exitCondition
         */
        public FSMState(T name, Supplier<Command> commandSupplier, BooleanSupplier exitCondition){
            // System.out.println(name.toString());
            this.name = name;
            this.commandSupplier = commandSupplier;
            this.exitCondition = exitCondition;
        }

        public void addAutoTransition(T destination, BooleanSupplier condition){
            var t = new AutoTransition<T>();
            t.destination = destination;
            t.condition = condition;
            autotransitions.add(t);
        }
        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Goal Complete", isAtGoalState, null);
        builder.addStringProperty("Current State", activeState.name::toString, null);
        builder.addBooleanProperty("running", ()->this.running, null);
    }


    //TODO: Missing useful items
    // optional "transition command" to be used in place of decorating initial command: Jack in the bot uses this for sequentially moving arm and reversing it
    // Auto-transition to state when executed as target state and done 
    //  -> Give Booleansupplier+node, if goal node, check conditions and auto-transition
    //"distance" function for the FSM to attempt state recovery
    //Automatically build a SendableChooser start->finish and show the paths so it's easy to proofread

    // add state builder of addState(id,command,transitionTo)) to allow for 
    //   sequences that just end normally to transition without complex workarounds

    //Optional config to ignore enable/disable when scheduling

}
