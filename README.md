# FSM Testing

Testing repo for the [FSM](/src/main/java/frc/robot/FSM/FSM.java) library. 

## Design goals
- Provide a state machine manager suitable for both subsystem + superstructure/management work
- Works within (and on top of) Command oriented robots
- Provides state graph routing to streamline transition planning
- Be fully defined with Java types for easy static analysis
- Work as a self-directed/triggered FSM with out external input
- Be responsive + suitable for direct management by Triggers and driver inputs

## Core Concepts

This library uses the FRC Command as the "state action". When the state is entered, the Command is scheduled and run, performing the indicated task. 

Each state also provides a BooleanSupplier representing the "completion condition" of a task. Just like normal command `until(...)` statements, this defines what done means for the context of that state. It might be a completed motion, or the removal or addition of a game piece.


To facilitate larger, complex state machines, this impliments [Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm) to allow seamless traversal of the state graph. When switching states, nodes are evaluated for the most efficient path. Each state is then run until its completion state, and the next state queued until the goal state is reached. 

To facilitate traversal, states can optionally split their "completion conditions" into two conditions: One for traversal, and one for a goal node. This allows more seamless and responsive interactions when a goal state might rely on external factors that may not complete.


## Usage and examples

This example models a simple roller intake

```java
///Create an enum listing state names
public enum IntakeState{ //Can be named whatever! Each FSM should have a unique enum+state list.
    Unloaded,
    Loading,
    Loaded,
    Scoring,
    Ejecting,
}
///Connect your state list to the FSM manager
FSM<IntakeState> fsm = new FSM<>(IntakeStateUnloaded);

///Define your state actions
/// This is similar to just setting up commands for each intended action
fsm.addState(IntakeState.Unloaded,
    ()->rollers.set(0),
    rollers.haveGamePiece 
);
fsm.addState(IntakeState.Loading,
    ()->rollers.set(-1), //full power 
    rollers.haveGamePiece 
);
fsm.addState(IntakeState.Loaded,
    ()->rollers.set(-0.1),
    ()->true, //As a transition state, we don't want to hold things up.
    ()->false, //As a goal state, we have no specific goals
);
fsm.addState(IntakeState.Scoring,
    ()->rollers.set(0.5),
    rollers.haveGamePiece.negate(),
);
fsm.addState(IntakeState.Ejecting,
    ()->rollers.set(1),
    rollers.haveGamePiece.negate(), 
);

//Now, we connect the states: We want it to have a flow graph like this:
/*                         
    ┌──► Unloaded ◄───┐    
    │       │         │    
    │       ▼         │    
    │     Loading     │    
    │       │         │    
    │       ▼         │    
    │     Loaded      │    
    │      │ │        │    
  Scoring◄─┘ └──► Ejecting 
*/
// In this case, we have a fairly obvious nominal flow.
// We can represent this as a one-way state sequence.
fsm.addDirectionalConnection(
    IntakeState.Unloaded,
    IntakeState.Loading,
    IntakeState.Loaded,
    IntakeState.Scoring,
    IntakeState.Unloaded,
);
//We may also need to circle through the eject route, so add that
fsm.addDirectionalConnection(
    IntakeState.Loaded,
    IntakeState.Ejecting,
    IntakeState.Unloaded,
);

//Now we can turn those completion conditions into state transitions
//As each condition is met, This will cause it to traverse the state graph to the 
//destination state. In this case, they're all very close by.
fsm.addAutoTransition( IntakeState.Unloaded, IntakeState.Loading );
fsm.addAutoTransition( IntakeState.Loading, IntakeState.Loaded );
fsm.addAutoTransition( IntakeState.Scoring, IntakeState.Unloaded );
fsm.addAutoTransition( IntakeState.Ejecting, IntakeState.Unloaded );

//And lastly, we can generate commands to traverse our states as appropriate.
joystick.a().whileTrue( fsm.setWait(IntakeState.Scoring) );

//If something goes wrong, we can *also* force a specific state
//Nominally one that effectively lets the system "reset" the system.
joystick.back().whileTrue( fsm.forceState(IntakeState.Unloaded) );

```

## Useful Patterns

#### Streamlining exit conditions 
Many game piece loading/unloading states are only ever useful as as goal states, 
and not part of any routing sequence. This means you can use the split conditions to
give a "traversal condition" of `()->true`, knowing these nodes are only ever "traversed" 
when leaving them. This allows the states to exit instantly when you want to do something else,
while still having useful conditions as a goal state. 

```java
fsm.addState(Example.TryToLoadGamePiece,
    ()->rollers.intake(),
    ()->true, //will only happen when we're leaving this state
    rollers.haveGamePiece //The completion condition when we're intending to be in this state
);
```

#### Backtracking
For bidirectional state transitions, if you change states from A->B to B->A, the state router will
bypass the transition conditions for B, and immediately re-schedule state B.

It has a few benefits, but importantly allows drivers to "undo" motions where the typical
state conditon might take significant time or do something undesirable such as extending 
beyond the out the frame perimeter. 

Auto-transitions also do this in cases where the transition condition is ignored.


## Detailed Documentation

For flexibility and simplicity, many of the commands have options or variants
to streamline setting up connections, or constraining/restricting transitions. 

```java
/// Connect multiple states in a one-directional manner
/// EG, you can go from StateA->StateB, but not StateB->StateA
fsm.addDirectionalConnection(StateA,StateB,StateC,...);

/// Connect states in a bi-directional manner
/// EG, you can go from StateA->StateB, or StateB->StateA
fsm.addConnection(StateA,StateB,StateC,...);

/// Connect StateHub to multiple other states (bidirectionally).
fsm.addConnectionHub(StateHub,StateA,StateB,StateC,...);

//Note, that all states that intersect become valid routes!
//If you run
fsm.addConnection(StateA,StateB,StateC);
fsm.addConnection(StateE,StateB,StateD);
//Then you have also defined StateA->StateE as a valid route.
//Directionality Still applies though.


//Automated transitions can rely on the default exit conditon, and/or 
//provided with independent conditions.
fsm.addAutoTransition(fromState, toState, condition , requiresStateTransitionCompletion);
//An example is doing moving our over-the-bumper into and out of frame. This transition 
// immediately brings the intake back inside the moment a game piece is aquired: 
// It doesn't care what OverBumperGrab might consider "done" or if it's in the intended position yet
fsm.addAutoTransition(Intake.OverBumperGrab, Intake.InsideBumper, ()->intake.hasGamePiece() ,false);
//On the other hand, if we do require the transitionCompletion, this transition would only be valid 
// once OverBumperGrab says "I'm in the right place", at which point the transition will fire
// bringing the intake back in.
fsm.addAutoTransition(Intake.OverBumperGrab, Intake.InsideBumper, ()->intake.hasGamePiece() ,true);

//By default, the two shorthands do the Right Thing:
//This one assumes you *always* transition if and only if a state's completion has been met.
fsm.addAutoTransition(fromState, toState)
//It's equivilent to 
fsm.addAutoTransition(fromState, toState,()->true,true);
//This version, on the other hand, assumes we don't care about the completion state: 
// Instead, we just care about the provided condition and nothing else. 
fsm.addAutoTransition(fromState, toState,()->condition)
//which is equivilent to
fsm.addAutoTransition(fromState, toState,()->condition,false);
```

## Interacting with the FSM

As a command-oriented FSM manager, it has several factories. Like normal commands, these can 
be used with triggers, joysticks, combined in parallel/sequential groups, and otherwise 
decorated normally to form sequences.
```java
//Traverse to and run the goal state indefinitely.
//Similar to the normal Commands.run() factory.
fsm.setRun(state)

//Traverse to the goal state, and run it until completion condition is met.
//Effectively Commands.run().until(...), and good for sequencing actions
fsm.setWait(state)

//setAsync allows you to set the target state, exiting immediately. 
//await does nothing, but exits when the goal is reached, and its completion condition is met. 
fsm.setAsync(state)
fsm.await()
//This allows you to quickly and easily set the fsm to run in the background while you do
// something else without having to deal with parallel groups. As an example,
Commands.sequence(
    fsm.setAsync(ScorerState.ReadyToScore), //Get the system moving
    chassis.driveToScoringPosition(), //Drive, which might take a second or two
    fsm.await(), //Wait for the target state, if it's not already done
    fsm.setWait(ScorerState.Score) //and, proceed to the next step.
);



//Forcibly go to the indicated state; This bypasses normal traversal, ignoring all exit conditions.
//The state machine will ignore all transitions, completions, and other normal processes until given 
//a fsm.set___(state) or other directive for what to do next.
//This is not optimal normal use, but intended as an escape hatch, reset, or panic button.
fsm.forceState(state)
//It *is* a command that exits after a single iteration, so you can decorate it to do a reset.
fsm.forceState(state).andThen(fsm.setRun(state))
//That said, be mindful of what this state is and how you get there. 
// This command can break robots.


//And, we have several Triggers and methods that can be used to query the state machine
fsm.isAtGoalState // True if we're *at* the goal state, ignoring completion
fsm.isGoalStateComplete // True if we're at the goal state, *and* done with the state task
fsm.getActiveState() //returns the currently active state
fsm.getGoalState() //Returns the goal state
fsm.isAtState(state) // Specifically query if we're at a specific state.


// The FSM by default prints state transitions (and what state it belongs to) 
// to the STDout/Riolog. This behaviour can be configured with, and outputs made more or less noisy.
setLogLevel(Level.<level>)
```
