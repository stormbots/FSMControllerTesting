package frc.robot.FSM;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Deque;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
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
    FSMState<T> priorState;
    public Command activeCommand = Commands.none();
    private boolean running=false;
    private T initialState;

    Dijkstra<T> stateRouter = new Dijkstra<>();
    Deque<T> statePath=new ArrayDeque<>();

    /** This exists to efficiently convert Strings back to state names */
    private Map<String,Enum<T>> stringMap = new HashMap<>();

    /** Allow routing to backtrack along edge transitions, 
     * allowing drivers to more easily "undo" longer motions
     */
    private final boolean TRANSITION_BACKTRACKING = true;

    /** Generic test for reaching the intended goal state */
    public Trigger isAtGoalState=new Trigger(()->
        statePath.isEmpty() && this.activeState!=null && this.activeState.exitCondition.getAsBoolean()
    );

    /**
     * Create a new Finite State Machine. 
     * Will begin in the provided initial state
     * @param initialState
     */
    public FSM(T initialState){
        //This is just to prevent potential null references throughout the code.
        //Adding the state definition later will overwrite this. 
        this.initialState=initialState;
        this.activeState=new FSMState<T>(initialState, Commands::none, ()->false);
        this.priorState=activeState;

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


    private void reschedule(T state){
        System.out.println("Rescheduling to "+state);
        activeCommand.cancel();
        priorState=activeState;
        activeState = stateMap.get(state);
        activeCommand=activeState.commandSupplier.get();
        activeCommand.schedule();
    }

    public void manageStates(){
        // SmartDashboard.putBoolean("fsm/atgoalstate",isAtGoalState.getAsBoolean());

        //Walk through our state queue
        //We only need to care about exit condition if we have more states.
        if(activeState.exitCondition.getAsBoolean() && statePath.size()>0){
            reschedule(statePath.poll());
        }
        else if(activeCommand.isScheduled()==false && activeState.autotransitions.size()>0){
            //Command exits and has auto-transitions
            //TODO: Verify sugar for creating auto-transitioning commands. If there's no further quirks, 
            //  this likely does not need to be a seperated branch/check with  special handling
            //  Nominally, 
            for(var transition: activeState.autotransitions){
                if(transition.condition.getAsBoolean()){
                    System.out.println("Auto Transition from  "+activeState.name +" to " +transition.destination);
                    reschedule(transition.destination);
                    break; //Don't check more conditions; First come first served.
                }
            }
            //No valid auto-transitions.? Do we need to permanently schedule? This else if checking isn't helping
        }
        else if(activeCommand.isScheduled() && activeState.exitCondition.getAsBoolean()){
            //command does *not* exit, so check it's automatic transitions
            for(var transition: activeState.autotransitions){
                if(transition.condition.getAsBoolean()){
                    System.out.println("Auto Transition from  "+activeState.name +" to " +transition.destination);
                    reschedule(transition.destination);
                    break; //Don't check more conditions; First come first served.
                }
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

    /** Return the tag for the current state */
    public T getActiveState(){
        return activeState.name;
    }

    /** Returns true if the FSM is at the indicated state and the completion supplier is true. */
    public boolean isAtState(T state){
        return activeState.name==state && isAtGoalState.getAsBoolean();
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
                var statePath=stateRouter.computeCosts(activeState.name, state);
                var eewgross = List.copyOf(statePath); //FIXME: can't index into deques, so gross workaround.
                //If we're on the destination transition, backtrack along the transition
                if(TRANSITION_BACKTRACKING && statePath.size()>=2 && eewgross.get(1)==priorState.name){
                    System.out.printf("Backtracking detected: %s -> %s\n",
                        eewgross.get(0),
                        eewgross.get(1)
                    );
                    statePath.poll();
                    //The backtrack problem! 
                    // if we can go from current -> prior, also path from prior->target and take
                    // the lower valued route maybe?
                    // Currently, it's doing what it *should* in the case that the state exit conditions
                    // are unhelpfully tied to things like game pieces

                    //Ideas: If node not done but connects to prior node, fsm from prior node instead
                    //if both nodes connect to new target, just ignore current node state?

                    //The problem: 
                    //- When changing states from a state with a unpredictable exit condition, 
                    //  the process breaks down
                    //- Avoid relying on difficult exit conditions?
                    //- Add a bypass option
                    //- Or.... just do auto-transitions but don't rely on externally controlled things? 

                    //NO! Allow an option for "transitional" exit conditions. When the current node goes from
                    // target->transition the exit condition may get swapped out with a more favorable one.
                    // This allows automatic exits/leaf node exits and sudden updates
                    // If it's optional it doesn't forcibly clutter the api, and the scheduler can use the 
                    // most relevant one.
                    
                    //TODO: Do not cancel/reschedule the same node. This is causing glitchy motions when you button mash
                    
                }
                this.statePath = statePath;
                // var path = statePath.stream().map(String::toString)co
                // System.out.printf("Pathing from %s -> %s\n",
                // )

                //TODO: I'm pretty sure I can pop this (contains the current node), see if the peek node is prior node,
                //and then skip directly to a new pop
                reschedule(statePath.peek());
            },
            ()->{}
        );
    }

    /** Set the target state and return, ignoring when or how it completes.
     * Useful for setting up parallel sequencing without having to use deadlines
     * when combined with {@link #await()}.
    */
    public Command setAsync(T state){
        return setRun(state).until(()->true);
    }

    /** Wait for the currently set goal state to be reached. Pairs well with {@link #setAsync(Enum)}. */
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
     * You likely want the {@link #addState(Enum, Supplier, BooleanSupplier)} version
     * @param state
     * @return
     */
    public FSM<T> addState(FSMState<T> state){
        if(stateMap.containsKey(state.name)){
            var str = String.format("Duplicate definition of state %s",
            state.name.toString()
        );
        throw new Error(str);

        }

        if(state.name==initialState) activeState = state;
        stateMap.put(state.name, state);
        stateRouter.addNode(state.name);
        return this;
    }

    /**
     * Add a new State, containing the minimial state actions. 
     * <br/>
     * States represent the actions the robot should be undertaking at the point in time, 
     * using the provided Commands. 
     * Generally, provided commands should not, but instead rely on the completionSupplier
     * to terminate the command as needed.
     * <br>/
     * The CompletionSupplier should indicate that the state's primary work is complete. 
     * When the state is an intermediate state during a longer transition, the CompletionSupplier
     * reporting true means it can progress to the next step. 
     * <br/>
     * When a goal or target state is complete, FSM based commands like {@link #setWait(Enum)}
     * will terminate, allowing progression of sequences.
     * <br/>
     * Automatic transitions provided by {@link #addAutoTransition(Enum, Enum, BooleanSupplier)} 
     * will be checked when the state's command ends or when it's completion status is true.
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

    /**
     * Connect two states, allowing unidirectional connections
     * @param state1
     * @param state2
     * @param cost Must be greater than zero: 1 is a good default.
     * @param bidirectional
     * @return
     */
    public FSM<T> addConnection(T state1, T state2, double cost, boolean bidirectional){
        if(cost<=0){
            var str = String.format("Cost must be greater than zero for transition %s -> %s (given %n)",
                state1.toString(),state2.toString(),cost
            );
            throw new Error(str);
        }
        stateRouter.addConnection(state1, state2, cost, bidirectional);
        return this;
    }

    /** Connect two states with a bidirectional link */
    public FSM<T> addConnection(T state1, T state2, double cost){
        addConnection(state1, state2, cost,true);
        return this;
    }

    /** Connect two or more states sequences using default costs.
     * Can be used to define a full sequence in one go.
     * Connections will be bi-directional.
     * @param states
     * @return
    */
    public FSM<T> addConnection(T... states){
        if(states.length<=1){
            throw new Error("Need two or more connections");
        }
        for(int i=1; i<states.length; i++){
            addConnection(states[i-1],states[i],1);
        }
        
        return this;
    }

    /** 
     * Connect multiple states in a using default costs. 
     * The connections will be uni-directional.
     * @param states
     * @return
     */
    public FSM<T> addDirectionalConnection(T... states){
        if(states.length<=1){
            throw new Error("Need two or more connections");
        }
        for(int i=1; i<states.length; i++){
            addConnection(states[i-1],states[i],1,false);
        }
        
        return this;
    }

    

    /** Connect one hub state to multiple other states simultaneously. Uses default cost of 1*/
    public FSM<T> addConnectionHub(T hubState, T... states){
        if(states.length<1){
            throw new Error("Need one or more connections");
        }
        for(int i=0; i<states.length; i++){
            addConnection(hubState,states[i],1);
        }
        
        return this;
    }


    /** Automatically transition from one state to another when a condition is met.
     * This only occours when the fromStates state's completion condition has been met.
     * (eg, it's at a position or finished it's other task)
     * @param fromState 
     * @param toState 
     * @param condition 
     * @return
     */
    public FSM<T> addAutoTransition(T fromState, T toState, BooleanSupplier condition){
        var state = stateMap.get(fromState);
        if(state==null){
            throw new Error("Initial state not present in known FSM states. Add before setting transitions.");
        }
        state.addAutoTransition(toState,condition);
        return this;
    }

    /** Automatically transition from one state to another once its completion condition has been met.
     * @param fromState
     * @param toState
     * @return
     */
    public FSM<T> addAutoTransition(T fromState, T toState){
        addAutoTransition(fromState,toState,()->true);
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
    //"distance" function for the FSM to attempt state recovery
    //Automatically build a SendableChooser start->finish and show the paths so it's easy to proofread

    // add state builder of addState(id,command,transitionTo)) to allow for 
    //   sequences that just end normally to transition without complex workarounds

    //Optional config to ignore enable/disable when scheduling

}
