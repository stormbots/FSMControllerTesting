package frc.robot.FSM;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
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

    private HashMap<T,FSMState<T>> stateMap = new HashMap<>();
    private FSMState<T> activeState;
    private FSMState<T> priorState;
    private Command activeCommand = Commands.idle();
    private boolean running=false;
    private T initialState;

    Dijkstra<T> stateRouter = new Dijkstra<>();
    Deque<T> statePath=new ArrayDeque<>();

    public enum InternalState{
        Traversing,
        ApproachingGoal,
        AtGoal,
        Unscheduled,
        Forced,
        Updating
    }
    private InternalState internalState=InternalState.Unscheduled;


    /** This exists to efficiently convert Strings back to state names */
    private Map<String,Enum<T>> stringMap = new HashMap<>();

    /** Allow routing to backtrack along edge transitions, 
     * allowing drivers to more easily "undo" longer motions
     * when those states have bidirectional links.
     */
    private boolean enableStateBacktracking=true;

    /** The default cost when setting up connections */
    private double defaultCost=1;

    /** Generic test for reaching the intended goal state */
    public Trigger isGoalStateComplete=new Trigger(()->
        // statePath.isEmpty() && this.activeState!=null && this.activeState.goalCompletionCondition.getAsBoolean()
        internalState==InternalState.AtGoal && activeState.goalCompletionCondition.getAsBoolean()
    );
    public Trigger isAtGoalState=new Trigger(()->
        (internalState==InternalState.ApproachingGoal || internalState==InternalState.AtGoal)
        && activeState.transitionCompletionCondition.getAsBoolean()
    ).or(isGoalStateComplete);
    ;

    /**
     * Create a new Finite State Machine. 
     * Will begin in the provided initial state
     * @param initialState
     */
    public FSM(T initialState){
        //This is just to prevent potential null references throughout the code.
        //Adding the state definition later will overwrite this. 
        this.initialState=initialState;
        this.activeState=new FSMState<T>(initialState, Commands::none, ()->true, ()->false);
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


    /** Schedule a target state command, cancelling any existing commands */
    private void reschedule(T state){
        //Avoid re-scheduling running commands
        if(activeState.name==state && activeCommand.isScheduled()) return;

        System.out.println("Rescheduling to "+state);
        activeCommand.cancel();
        priorState=activeState;
        activeState = stateMap.get(state);
        activeCommand=activeState.commandSupplier.get();
        activeCommand.schedule();
    }

    /** Generate a path between various states, considering the backtracking considerations */
    private void updatePath(T start,T goal, T previousState, boolean enableBacktracking){
        var statePath=stateRouter.computeCosts(start, goal);
        var eewgross = List.copyOf(statePath); //FIXME: can't index into deques, so gross workaround. Fix backing data structure
        //If we're on the destination transition, backtrack along the transition
        if(enableBacktracking && statePath.size()>=2 && eewgross.get(1)==previousState){
            System.out.printf("Backtracking detected: %s -> %s\n",
                eewgross.get(0),
                eewgross.get(1)
            );
            statePath.poll();
        }
        this.statePath = statePath;
    }

    /**
     * Check the auto transitions, changing states if
     * - Our state is completed, and a matching condition applies
     * - OR we don't care about the completion, but have a matching transition
     * Backtracking/shortcutting is permitted on states where the goal transition state is not required.
     * @return
     */
    private boolean checkAutoTransitions(){
        var activeStateComplete=activeState.goalCompletionCondition.getAsBoolean();
        for(var transition: activeState.autotransitions){
            if( (activeStateComplete || transition.requiresCompletion==false) && transition.condition.getAsBoolean()){
                System.out.println("Auto Transition from  "+activeState.name +" to " +transition.destination);
                System.out.println(transition);
                //TODO: Test backtracking assumptions under a variety of conditions. This seems to do the Right Thing
                //but worth being careful of and noting.
                updatePath(activeState.name, transition.destination, priorState.name, transition.requiresCompletion==false);
                reschedule(statePath.peek());
                internalState = InternalState.Updating;
                return true; //from loop //Don't check more conditions; First come first served.
            }
        }
        return false;
    }


    /**
     * The primary operation to manage the state changes 
     */
    private void manageStates(){
        /*IMPLEMENTATION NOTE:
        * This switch has several intentional fallthroughs to enable a single bot cycle
        * to make multiple internal state transitions at once. Because this is running
        * on a potato, has a real-world time savings of 20ms per fallthrough, meaning
        * we save up to 0.1s for certain interactions.
        */
        switch(internalState){
        case Traversing:
            if(activeState.transitionCompletionCondition.getAsBoolean()){
                internalState=InternalState.Updating;
                //intentional fallthrough to next case
            }else{
                break;
            }

        case Updating:
            if(statePath.size()>1){
                //We have more states!
                reschedule(statePath.poll());
                internalState=InternalState.Traversing;
                break;
            } else if(statePath.size()>0){
                //We only have one state, which means it's our goal
                reschedule(statePath.poll());
                internalState=InternalState.ApproachingGoal;
                //fallthrough to next
            } else {
                //Something triggered a re-schedule, but we've already
                //scheduled our final state. Therefore, goal.
                internalState = InternalState.ApproachingGoal;
                //fallthrough to next
            }

        case ApproachingGoal:
            checkAutoTransitions();

            if(activeState.transitionCompletionCondition.getAsBoolean()){
                internalState = InternalState.AtGoal;
                //intentional fallthrough
            }else if(activeState.goalCompletionCondition.getAsBoolean()){
                    internalState = InternalState.AtGoal;
                    //intentional fallthrough
                }else{
                    break;
            }

        case AtGoal:
            checkAutoTransitions();

            if(activeCommand.isScheduled()==true){
                //Expected case: Nothing to do
                break;
            }else{
                //Unexpected case! Fall through to handling of this scenario
                internalState=InternalState.Unscheduled;
            }
            break;

        case Unscheduled:
            //We generally shouldn't be here, and this state is ill defined.
            //An Auto Transition should have fired, or we should have a command that is still running
            //In this case, re-schedule it with a .forever()

            //Commented out for now because for whatever reason this state *does* seem to trigger constantly, 
            //causing glitches. It seems like something clunky with the scheduler and auto-transition timing.

            // System.err.printf(
            //     "State command %s unexpectedly exited without transition. Rescheduling indefinitely.\n",
            //     activeState.name.toString()
            // );
            // reschedule(activeState.name);
            // activeState.commandSupplier.get().schedule();

            internalState = InternalState.Updating;
            break;

        case Forced:
            //We go to the current state, no questions asked or other conditions considered.
            //We will exit this state when something reschedules it normally.
            activeState.commandSupplier.get().schedule();
        }
        SmartDashboard.putString("fsmInternalState", internalState.toString());
    }


    /** Return the tag for the current state */
    public T getActiveState(){
        return activeState.name;
    }

    /** Return the enum for the goal state */
    public T getGoalState(){
        return statePath.isEmpty() ? activeState.name : statePath.peekLast();
    }

    /** Returns true if the FSM is at the indicated state and the completion supplier is true. */
    public boolean isAtState(T state){
        return activeState.name==state && isGoalStateComplete.getAsBoolean();
    }

    /** Set the state and wait for completion. Good for sequencing. */
    public Command setWait(T state){
        return setRun(state).until(isGoalStateComplete);
    }

    /** Set the state and wait indefinitely; Good for buttons.*/
    public Command setRun(T state){
        return Commands.startRun(
            ()->{
                updatePath(activeState.name, state, priorState.name, enableStateBacktracking);
                reschedule(statePath.peek());
                internalState=InternalState.Updating;
                // manageStates();//ping the state machine for our next step
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
        return Commands.waitUntil(isGoalStateComplete);
    }

    /** Forcibly set a state change, bypassing state pathing. */
    public Command forceState(T state){
        return Commands.runOnce(()->{
            statePath.clear();
            this.activeState=stateMap.get(state);
            this.activeCommand = activeState.commandSupplier.get();
            this.activeCommand.schedule();
            this.internalState = InternalState.Forced;
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
     * The alternate form {@link #addState(Enum, Supplier, BooleanSupplier, BooleanSupplier)} can
     * be used to provide seperate cases when the completion goals differ when a goal state vs 
     * when part of a transition path.
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
        addState(new FSMState<T>(name,commandSupplier,completionSupplier,completionSupplier));
        return this;
    }

    /**
     * Add a new state, specifying independent goals when the state is a transition state vs a goal state
     * 
     * @param name
     * @param commandSupplier
     * @param transitionCompletionSupplier
     * @param goalCompletionSupplier
     * @return
     */
    public FSM<T> addState(
        T name, 
        Supplier<Command> commandSupplier, 
        BooleanSupplier transitionCompletionSupplier,
        BooleanSupplier goalCompletionSupplier
    ){
        addState(new FSMState<T>(name,commandSupplier,transitionCompletionSupplier,goalCompletionSupplier));
        return this;
    }


    /**
     * Connect two states together with appropriate cost and directionality.
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

    /** Connect two or more states sequences using default cost.
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
            addConnection(states[i-1],states[i],defaultCost);
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
            addConnection(states[i-1],states[i], defaultCost,false);
        }
        
        return this;
    }

    

    /** Connect one hub state to multiple other states simultaneously. Uses default cost of 1*/
    public FSM<T> addConnectionHub(T hubState, T... states){
        if(states.length<1){
            throw new Error("Need one or more connections");
        }
        for(int i=0; i<states.length; i++){
            addConnection(hubState,states[i], defaultCost);
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
    public FSM<T> addAutoTransition(T fromState, T toState, BooleanSupplier condition, boolean requiresTransitionCompletion){
        var state = stateMap.get(fromState);
        if(state==null){
            throw new Error("Initial state not present in known FSM states. Add before setting transitions.");
        }
        state.addAutoTransition(toState,condition,requiresTransitionCompletion);
        return this;
    }

    /** Automatically transition from one state to another once its completion condition has been met.
     * If multiple conditions are true simultaneously, the first one defined wins.
     * @param fromState
     * @param toState
     * @return
     */
    public FSM<T> addAutoTransition(T fromState, T toState){
        addAutoTransition(fromState,toState,()->true,true);
        return this;
    }
    
    /** Automatically transition from one state to another once once the provided condition is met. 
     * Does not require the standard state completion.
     * @param fromState
     * @param toState
     * @param condition
     * @return
     */
    public FSM<T> addAutoTransition(T fromState, T toState, BooleanSupplier condition){
        addAutoTransition(fromState,toState,condition,false);
        return this;
    }



    /** Container for state data.
     */
    public static class FSMState<T extends Enum<T>>{
        public Supplier<Command> commandSupplier = ()->new InstantCommand();
        public BooleanSupplier transitionCompletionCondition=()->false;
        public BooleanSupplier goalCompletionCondition=()->false;
        public T name;

        public class AutoTransition<T>{
            public BooleanSupplier condition;
            public T destination;
            public Boolean requiresCompletion;
            public String toString(){
                return String.format("AT(%s->%s|()->%s|()->%s && %s)",
                    name.toString(),
                    destination.toString(),
                    condition.getAsBoolean(),
                    goalCompletionCondition.getAsBoolean(),
                    requiresCompletion
                );
            }
        }
        public ArrayList<AutoTransition<T>> autotransitions = new ArrayList<>();

        /**
         * Provide a state with name, command, and exit conditions.
         * @param name
         * @param commandSupplier
         * @param transitionCompletionCondition
         */
        public FSMState(
                T name,
                Supplier<Command> commandSupplier,
                BooleanSupplier transitionCompletionCondition,
                BooleanSupplier goalCompletionCondition
            ){
            // System.out.println(name.toString());
            this.name = name;
            this.commandSupplier = commandSupplier;
            this.transitionCompletionCondition = transitionCompletionCondition;
            this.goalCompletionCondition = goalCompletionCondition;
        }

        /**
         * Configure an automatic transition from this state to another
         * @param destination
         * @param condition the boolean condition under which we transition
         * @param requiresStateCompletion Whether we also require the state's completion condition to be met.
         */
        public void addAutoTransition(T destination, BooleanSupplier condition, Boolean requiresStateCompletion){
            var t = new AutoTransition<T>();
            t.destination = destination;
            t.condition = condition;
            t.requiresCompletion=requiresStateCompletion;
            autotransitions.add(t);
        }        
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Transition Complete", isGoalStateComplete, null);
        builder.addBooleanProperty("Goal Complete", isGoalStateComplete, null);
        builder.addStringProperty("Current State", activeState.name::toString, null);
        builder.addBooleanProperty("running", ()->this.running, null);
    }


    //TODO: Missing useful items
    //"distance" function for the FSM to attempt state recovery
    //Automatically build a SendableChooser start->finish and show the paths so it's easy to proofread

    //Optional config to ignore enable/disable when scheduling

}
