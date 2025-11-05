// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Extendo;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Creates a new Wrist. */
public class Extendo extends SubsystemBase {
  
  SparkFlex motor = new SparkFlex(12, MotorType.kBrushless);

  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(100, 300);
  TrapezoidProfile profile = new TrapezoidProfile(constraints);

  // ArmFeedforward feedforward = new ArmFeedforward(0.005,0.76,0,0);
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.005, 0.0);
  /** used for providing appropriate information to feedforwards and physics */
  Supplier<TrapezoidProfile.State> armStateProvider = ()->new TrapezoidProfile.State();

  ExtendoSim sim = new ExtendoSim(motor);
  

  public Extendo(Supplier<TrapezoidProfile.State> armStateProvider){
    //Configure the encoder
    configureMotor();

    this.armStateProvider = armStateProvider;

    setDefaultCommand(new ConditionalCommand(
      setDistance(()->goal.position).repeatedly(),
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
      .reverseSoftLimit(0)
      .forwardSoftLimit(36)
      .forwardSoftLimitEnabled(false)
      .reverseSoftLimitEnabled(false)
      ;

    //90 degrees is 18.8 rotations 
    double rotateCoversionFactor = 1/50.0;
    config.encoder
        .velocityConversionFactor(rotateCoversionFactor / 60.0)
        .positionConversionFactor(rotateCoversionFactor)
        ;        

    config.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .p(0.7/1.0) //TODO: This last *6 is for my sim
      .positionWrappingEnabled(false)
      ;

    Alert alert = new Alert("Extendo motor failed to initialize", AlertType.kError);
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
    SmartDashboard.putNumber("extendo/distance", getDistance().in(Inches));
    SmartDashboard.putBoolean("extendo/at target", isAtTarget.getAsBoolean());
    SmartDashboard.putBoolean("extendo/at target (rough)", isRoughlyAtTarget.getAsBoolean());
  }


  public Distance getDistance(){
    return Inches.of(motor.getEncoder().getPosition());
  }

  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  private double startTimer=0;

  public Command setDistance(double position){
    return setDistance(()->position);
  }
  public Command setDistance(Supplier<Distance> position){
    return setDistance(()->position.get().in(Inches));
  }

  public Command setDistance(DoubleSupplier position){
    return startRun(
      ()->{
        startTimer = Timer.getFPGATimestamp();
        //Seed the initial state/setpoint with the current state
        setpoint = new TrapezoidProfile.State(getDistance().in(Inches), motor.getEncoder().getVelocity());
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
    ;
  }

  public void setPID(Supplier<Distance> position){
    var ff = feedforward.calculate(position.get().in(Inch), 0);
    motor.getClosedLoopController()
    .setReference(
      position.get().in(Inch),
      ControlType.kPosition, ClosedLoopSlot.kSlot0,
      ff, ArbFFUnits.kVoltage
    );
  }

  /** Apply only feedforward outputs to halt powered motion */
  public Command stop(){
    return run(()->{
      motor.setVoltage(feedforward.calculate(getDistance().in(Inches), 0));
    });
  }

  public Trigger isAtTarget = new Trigger(()->
    MathUtil.isNear(goal.position, motor.getEncoder().getPosition(), 0.5)
    &&MathUtil.isNear(goal.velocity, motor.getEncoder().getVelocity(), 2)
  ).debounce(0.05);

  Trigger isRoughlyAtTarget = new Trigger(()->
    MathUtil.isNear(goal.position, motor.getEncoder().getPosition(), 1)
    &&MathUtil.isNear(goal.velocity, motor.getEncoder().getVelocity(), 5)
  ).debounce(0.05);
  
}