package frc.robot.DijkstraV2;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CommandFSM<T extends Enum<T>> extends SubsystemBase{
    
    public static enum InnerState{
        offline,
        recoverState,
        updateStates,
        
        pathOptimization,
        traversalIncomplete,
        traversalComplete,

        forcedState,
        goalIncomplete,
        goalComplete,
    };
    private FSM2<InnerState> fsm = new FSM2<>(InnerState.offline);

    private final T initialState;

    private HashMap<T,CommandFSMState<T>> stateMap = new HashMap<>();
    private List<T> stateQueue;
    private CommandFSMState<T> offlineState;
    private CommandFSMState<T> currentState;
    private CommandFSMState<T> priorState;

    private DijkstraV2<T> stateRouter;

    /** Controls the way the robot being disabled is handled
     * <p>{@value #reset} : Stops running, and restarts/recovers the FSM on reenable
     * </p><p>{@value #retain} : Stops running, but do not alter system state.
     * </p><p>{@value #run} : Steps through the fsm regardless of the disable state. 
     * Note, that commands themselves only run when individually decorated with <code>.ignoringDisabled(true)</code>
     * </p> 
     * */
    public enum DisabledAction{
        run,
        reset,
        retain
    };
    private DisabledAction disabledAction=DisabledAction.retain;

    public CommandFSM(T initialState){
        this.initialState=initialState;

        fsm.addState(InnerState.offline,()->{})
        .addTransition(InnerState.recoverState, DriverStation::isEnabled)
        ;

        fsm.addState(InnerState.recoverState, this::recoverStates)
        .addTransition(InnerState.updateStates, ()->true)
        ;

        fsm.addState(InnerState.updateStates, this::updateStates)
        .addSelector(()->stateQueue.size()<=1 ? InnerState.pathOptimization : InnerState.goalIncomplete )
        ;
        
        fsm.addState(InnerState.pathOptimization, this::optimizeStatePath)
        .addTransition(InnerState.traversalComplete, ()->true)
        ;

        fsm.addState(InnerState.traversalIncomplete, ()->{})
        .addTransition(InnerState.traversalComplete, ()->currentState.traversalComplete.getAsBoolean())
        ;

        fsm.addState(InnerState.traversalComplete, ()->{})
        .addTransition(InnerState.updateStates, ()->true)
        .onEntry(()->{}) //Log the update?
        ;

        fsm.addState(InnerState.forcedState, ()->{})
        .addTransition(InnerState.goalIncomplete, ()->stateQueue.isEmpty()==false)
        ;

        fsm.addState(InnerState.goalIncomplete, ()->{})
        .addTransition(InnerState.goalComplete, ()->currentState.goalComplete.getAsBoolean());
        
        fsm.addState(InnerState.goalComplete, ()->{})
        .addTransition(InnerState.updateStates, ()->stateQueue.isEmpty()==false);
        ;
        
        //Initialize our operational states
        offlineState = new CommandFSMState<T>(initialState, Commands.idle(), ()->true, ()->false);
        currentState=offlineState;
        priorState=offlineState;
        stateQueue=new ArrayList<>();

        stateRouter = new DijkstraV2<T>(initialState);
    }

    //Step through the state command logic;
    public void periodic(){
        if(DriverStation.isDisabled()){
            switch(disabledAction){
            case reset:
                fsm.setState(InnerState.offline);
                return;
            case retain:
                //Do nothing, but avoid stepping through the FSM
                return;
            case run:
                //Do nothing, fall through to the updates
                break;
            }
        }

        //See if we have additional state updates from our goal states
        checkAutomatedStateTransitions();

        //Because there's several decision states, and several states that *could* 
        // transition instantly based on external factors, we need to iterate the 
        // So, simply loop until we end up in a state that doesn't transition out immediately.
        var lastState=fsm.getState();
        while(true){
            fsm.update();
            if(fsm.getState()==lastState) break;
            lastState=fsm.getState();
        }
    }

    private void checkAutomatedStateTransitions(){
        //Avoid checking transitions while a command is actively running the FSM
        //TODO: Check correctness of this behaviour, and maybe make it per-transition
        //  In general I think driver/active code intent is probably correct
        if(this.getCurrentCommand()!=null) return;

        // Chec the automated transitions, and if needed path to new states
        switch(fsm.getState()){
            case goalComplete:
                for(var t : currentState.whenGoalComplete){
                    if(t.condition.getAsBoolean()){
                        pathToState(t.dest);
                    }
                }
                //intentional fallthrough ; When complete we need to check both cases
            case goalIncomplete:
                for(var t : currentState.whenInState){
                    if(t.condition.getAsBoolean()){
                        pathToState(t.dest);
                    }
                }
            default:
                //No other cases to check
        }
    }

    private void optimizeStatePath(){
        //TODO: Avoid restarting identical states

        //TODO impliment logic for state backtracking/optimization
        // p=a, c=b, queue.first=b
    }

    private void updateStates(){
        if(stateQueue.isEmpty()){
            //Nothing to do; Operating in goal state.
            return;
        }

        //If we do have a queue, get the next command.
        var newstate=stateQueue.remove(0);
        priorState = currentState;
        currentState = stateMap.get(newstate);
        //NOTE: Commands will normally cancel automatically due to subsystem mutexes, 
        // but in case that commands aren't subsystem-locked, or have differing 
        // subsystem requirements, do it explicitly too
        priorState.command.cancel();
        currentState.command.schedule();
    }


    ///////////////////////////
    // Primary User Interfaces
    ///////////////////////////
    
    /** Set the queue to the provided destination state */
    public void pathToState(T destination){
        var path = stateRouter.findPath(currentState.id, destination);
        if(path.isEmpty()) return;
        stateQueue.clear();
        stateQueue = path;
    }

    /** Run to the destination state indefinitely. */
    public Command run(T destination){
        return this.startRun(
            ()->pathToState(destination),
            ()->{}
        );
    }

    /** Runs until the final state is completed, then exits */
    public Command runUntilCompletion(T destination){
        return run(destination).until(()->fsm.getState()==InnerState.goalComplete);
    }

    /** Set the state to the target, ending the command immediately. 
     * This lets the letting the FSM commands run in the background.
     * Intended as a companion with {@link #await()}
     */
    public Command setAsync(T destination){
        return runOnce(()->pathToState(destination));
    };

    /** Wait for the command to reach the goal state, without explicitly having set it.
     * Intended as a companion to {@link #setAsync(Enum)} in sequences.
     */
    public Command await(){
        return Commands.idle(this).until(()->fsm.getState()==InnerState.goalComplete);
    };

    /** Calculate the nearest recovery state; Does not perform any action or change states
     * @return
     */
    public T computeNearestRecoveryState(){
        var nearest=stateMap
        .values()
        .stream()
        .filter((s)->s.stateRecoveryCost.isPresent())
        .min((a,b)->Double.compare(a.stateRecoveryCost.get().getAsDouble(), a.stateRecoveryCost.get().getAsDouble()))
        .orElseGet(()->stateMap.get(initialState))
        ;
        return nearest.id;
    }

    /** Compute the nearest safe state and set it as the current active state. 
     * The initialState provided when creating the FSM will be used if no recovery states
     * are configured.
    */
    public void recoverStates(){
        var nearest=computeNearestRecoveryState();
        stateQueue.clear();
        stateQueue.add(nearest);
        fsm.setState(InnerState.recoverState);
    }

    /** Compute nearest safe state, force that state, and go directly there. Command exits
     * once the state's goal condition is met.
     */
    public Command recover(){
        return runOnce(()->fsm.setState(InnerState.recoverState)).andThen(await());
    }

    /** Cancel any active pathing, remaining in the current state.
     * Not a command, so it can be used in .finallyDo() operations if needed.
     */
    public void cancelPathing(){
        stateMap.clear();
    }
}
