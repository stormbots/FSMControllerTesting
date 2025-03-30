package frc.robot.FSM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.GraphCommand;
import frc.robot.FSM.StateMint.MyBotStates;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Rollers.Rollers;
import frc.robot.subsystems.Wrist.Wrist;

public class FSM {
    Arm arm; 
    Wrist wrist; 
    Rollers rollers;

    GraphCommand command = new GraphCommand();

    public FSM(Arm arm, Wrist wrist, Rollers rollers){
        this.arm = arm;
        this.wrist = wrist; 
        this.rollers = rollers;

        //Configure the nodes
        command.addRequirements(arm,wrist,rollers);

        new RunCommand(this::periodic)
        .ignoringDisable(true)
        .withName("FSM Manager")
        .schedule();
    }

    public void periodic(){
    }

    public Command setWait(BotState state){
        return state.command.until(state.exitCondition);
    }
    public Command setRun(BotState state){
        return state.command;
    }

    /** Run the command state, ignoring when or how it completes */
    public Command setAsync(BotState state){
        state.command.until(state.exitCondition).schedule();
        return new InstantCommand();
    }

    public BotState Stow = new BotState(//MyBotStates.up,
        new ParallelCommandGroup(
            arm.setAngle(()->0),
            wrist.setAngle(()->0),
            rollers.stop()
        ),
        arm.isAtTarget.and(wrist.isAtTarget)
    );


    public BotState L1 = new BotState(//MyBotStates.up,
        new ParallelCommandGroup(
            arm.setAngle(()->45),
            wrist.setAngle(()->45),
            rollers.stop()
        ).until(arm.isAtTarget.and(wrist.isAtTarget))
        .andThen(rollers.eject())
        ,
        rollers.isHoldingCoral.negate()
    );


    public BotState IntakeStation = new BotState(//MyBotStates.up,
        new ParallelCommandGroup(
            arm.setAngle(()->80),
            wrist.setAngle(()->-10),
            rollers.intake()
        ),
        rollers.isHoldingCoral
    );

    public BotState IntakeFloor = new BotState(//MyBotStates.up,
        new ParallelCommandGroup(
            arm.setAngle(()->0),
            wrist.setAngle(()->-10),
            rollers.intake()
        ),
        rollers.isHoldingCoral
    );

    



}
