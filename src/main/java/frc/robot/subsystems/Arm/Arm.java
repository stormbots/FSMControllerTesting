// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Arm extends SubsystemBase {

  SparkFlex motor = new SparkFlex(10, MotorType.kBrushless);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(90, 180);
  TrapezoidProfile profile = new TrapezoidProfile(constraints);

  ArmFeedforward feedforward = new ArmFeedforward(0.005,0.76,0,0);

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private double startTimer=0;


  ArmSim sim = new ArmSim(motor);

  /** Creates a new Arm. */
  public Arm() {
    //Configure the encoder
    configureMotor();

    setDefaultCommand(new ConditionalCommand(
      setAngle(()->goal.position).repeatedly(),
      stop(),
      isAtTarget
    ));
  }

  public void configureMotor(){

    SparkBaseConfig config = new SparkMaxConfig()
      .smartCurrentLimit(50)
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      ;

    config.softLimit
      .reverseSoftLimit(-90)
      .forwardSoftLimit(170)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false)
      ;

    //90 degrees is 18.8 rotations 
    double rotateCoversionFactor = 1/25.0*(18.0/64.0) * 360;
    config.encoder
        .velocityConversionFactor(rotateCoversionFactor / 60.0)
        .positionConversionFactor(rotateCoversionFactor)
        ;        

    config.absoluteEncoder
      .velocityConversionFactor(360 / 60.0)
      .positionConversionFactor(360)
      .inverted(true);

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.8/60.0*2*6) //TODO: This last *6 is for my sim
      .positionWrappingEnabled(false)
      .positionWrappingInputRange(0, 360)
      ;

    Alert alert = new Alert("Arm motor failed to initialize", AlertType.kError);
    //Configure the motor, retrying if needed
    for(int i=0; i<10; i++){
      var err = motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      if(err.equals(REVLibError.kOk)) break;
      alert.set(true);
      Timer.delay(0.02);
    }
    alert.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm/angle", getAngle().in(Degree));
    SmartDashboard.putBoolean("arm/at target", isAtTarget.getAsBoolean());
    SmartDashboard.putBoolean("arm/at target (rough)", isRoughlyAtTarget.getAsBoolean());
  }


  public Angle getAngle(){
    return Degree.of(motor.getEncoder().getPosition());
  }
  public AngularVelocity getVelocity(){
    return DegreesPerSecond.of(motor.getEncoder().getVelocity());
  }
  public TrapezoidProfile.State getState(){
    return new TrapezoidProfile.State(
      motor.getEncoder().getPosition(),
      motor.getEncoder().getVelocity()
    );
  }

  public Command setAngle(DoubleSupplier position){
    return startRun(
      ()->{
        startTimer = Timer.getFPGATimestamp();
        //Seed the initial state/setpoint with the current state
        setpoint = new TrapezoidProfile.State(getAngle().in(Degrees), motor.getEncoder().getVelocity());
        //Update in the Init to prevent trigger timing misfires 
        goal = new TrapezoidProfile.State(position.getAsDouble(), 0);
      }, 
      ()->{
        //Make sure the goal is dynamically updated
        goal = new TrapezoidProfile.State(position.getAsDouble(), 0);

        //update our setpoint to be our next state
        setpoint = profile.calculate(0.02, setpoint, goal);
    
        var ff = feedforward.calculate(setpoint.position, setpoint.velocity);
        motor.getClosedLoopController()
        .setReference(
          setpoint.position,
          ControlType.kPosition, ClosedLoopSlot.kSlot0,
          ff, ArbFFUnits.kVoltage
        );
      }
    )
    // .until(isAtTarget.and(()->profile.isFinished(Timer.getFPGATimestamp()-startTimer)))
    ;
  }

  /** Apply only feedforward outputs to halt powered motion */
  public Command stop(){
    return run(()->{
      motor.setVoltage(feedforward.calculate(getAngle().in(Radians), 0));
    });
  }

  public Trigger isAtTarget = new Trigger(()->
    MathUtil.isNear(goal.position, motor.getEncoder().getPosition(), 2)
    &&MathUtil.isNear(goal.velocity, motor.getEncoder().getVelocity(), 20)
  );

  Trigger isRoughlyAtTarget = new Trigger(()->
    MathUtil.isNear(goal.position, motor.getEncoder().getPosition(), 10)
    &&MathUtil.isNear(goal.velocity, motor.getEncoder().getVelocity(), 100)
  );

}
