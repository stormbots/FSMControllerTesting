package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Radian;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm.*;
import frc.robot.subsystems.Extendo.*;
import frc.robot.subsystems.Wrist.*;
public class ArmIK {
    Arm arm;
    Extendo extendo;
    Wrist wrist;

    public ArmIK(Arm arm, Extendo extendo, Wrist wrist){
        this.arm = arm;
        this.extendo = extendo;
        this.wrist = wrist;

        new Trigger(DriverStation::isEnabled).onTrue(Commands.run(this::updateKinematics).ignoringDisable(true));
    }

    public class Coord{
        public double x;
        public double y;
        Coord(double x, double y){this.x=x;this.y=y;}
        Coord(){}
    }

    public class Pose{
        public Angle arm;
        public Distance extendo;
        public Angle wrist;
    }


    public Coord xy(){
        //Prep the units
        var armlen=arm.getLength().plus(extendo.getDistance()).in(Inches);
        var wristlen=wrist.getLength().in(Inches);
        var armrad=arm.getAngle().in(Radian);
        var wristrad=wrist.getAngleGround().in(Radian);
        //Do the math
        var c=new Coord();
        c.x = armlen*Math.cos(armrad) + wristlen*Math.cos(wristrad);
        c.y = armlen*Math.sin(armrad) + wristlen*Math.sin(wristrad);
        return c;
    }

    public Pose ik(DoubleSupplier xpos, DoubleSupplier ypos, DoubleSupplier degrees){
        //prep the units 
        var wtheta=Degrees.of(degrees.getAsDouble()).in(Radian);
        var wlength=wrist.getLength().in(Inches);
        var x=xpos.getAsDouble();
        var y=ypos.getAsDouble();

        // Do the math
        var p = new Pose();
        x=x-wlength*Math.cos(wtheta);
        y=y-wlength*Math.sin(wtheta);

        var atheta=Math.atan2(y, x);
        var totallength=Math.hypot(y, x);
        var extendoLength= totallength - arm.getLength().in(Inches);

        //Set up the object
        p.extendo=Inches.of(extendoLength);
        p.arm = Radian.of(atheta);
        p.wrist = Radian.of(wtheta);

        return p;
    }


    public Command runIK(Supplier<Pose> poseSupplier){
        return Commands.parallel(
            arm.setAngle(()->poseSupplier.get().arm),
            wrist.setAngleFromGround(()->poseSupplier.get().wrist),
            extendo.setDistance(()->poseSupplier.get().extendo)
        );
    }

    public Command runIK(DoubleSupplier xpos, DoubleSupplier ypos, DoubleSupplier wristDegrees){
        return runIK(()->ik(xpos,ypos,wristDegrees));
    }

    //TODO This whole process currently does not work :<
    public Translation2d startPosition;
    public Command interp(Supplier<Translation2d> poseSupplier, LinearVelocity maxVelocity, Supplier<Angle> angle){
        return Commands.startRun(()->{

        },
        ()->{
            //calculate a position in line with our max velocity to the point
            var c = xy();
            var currentposition = new Translation2d(c.x, c.y);
            var goal = poseSupplier.get();

            var delta=goal.minus(currentposition);
            var unit=delta.div(delta.getNorm());
            var offset= unit.times(maxVelocity.in(InchesPerSecond)*0.02);
            if(offset.getNorm()<delta.getNorm()){
                //We now have our offset, add it back in.
                goal = currentposition.plus(offset);
            }
            goal=currentposition;
            
            //Do some IK now
            var pose= ik(goal::getX, goal::getY, ()->angle.get().in(Degree));


            arm.setPID(()->pose.arm);
            // wrist.setPID(()->pose.wrist);
            // extendo.setPID(()->pose.extendo);

        },wrist,arm,extendo);
        
    }


    public void updateKinematics(){
        var c=xy();
        SmartDashboard.putNumber("kinematics/x", c.x );
        SmartDashboard.putNumber("kinematics/y", c.y );
        SmartDashboard.putNumber("kinematics/a", wrist.getAngleGround().in(Degree) );
    }
}
