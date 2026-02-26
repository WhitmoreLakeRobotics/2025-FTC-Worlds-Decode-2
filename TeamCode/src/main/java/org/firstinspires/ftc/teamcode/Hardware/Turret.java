package org.firstinspires.ftc.teamcode.Hardware;

import static java.lang.Math.clamp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Base class for FTC Team 8492 defined hardware
 */
@Disabled
public class Turret extends BaseHardware{

    //Robot robot = new Robot();


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    private Servo rightyTighty;
    private Servo leftyLoosy;

    // tunable pid constants
    private double kP = 0.012;
    private double kI = 0.000;
    private double kD = 0.0012;

    // angle state
    private double targetAngle = 0;
    private double currentAngle = 0;
    private double lastError = 0;
    private double integral = 0;

    // limits
    private final double MIN_DEG = -170;
    private final double MAX_DEG = 170;

    // servo limits
    private final double MIN_POS = 0.05;
    private final double MAX_POS = 0.95;

    // saftey: max angle change per loop (deg)
    private final double MAX_VELOCITY = 4.0;//smooth but responsive (ADJUST LATER)
    private ElapsedTime runtime = new ElapsedTime();
    private boolean shouldAim = false;

    /**
     * Hardware Mappings
     */

    public TrapezoidAutoAim trapezoidAutoAim;

    public HardwareMap hardwareMap = null; // will be set in Child class

    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */

    @Override
    public void init(){
        //leftyLoosy = hardwareMap.get(Servo.class, "leftyLoosy");
       // rightyTighty = hardwareMap.get(Servo.class, "rightyTighty");

        //leftyLoosy.setDirection(Servo.Direction.FORWARD);  //may need to be changed later.
        //rightyTighty.setDirection(Servo.Direction.FORWARD);   //may need to be changed later.

        currentAngle = 0;
        targetAngle = 0;
    }

    public void init_loop(){

    }

    public void start(){

    }

    @Override
    public void loop(){

        // Track the tag ?????????
        //double yaw = robot.limey.getYaw();
        //setTargetAngle(-yaw);

        // --- PID CALCULATION ---
        double error = targetAngle - currentAngle;

        // anti-windup: only integrate when error is small
        if(Math.abs(error) < 25){
            integral += error;
        }else{
            integral = 0;
        }

        double derivative = error - lastError;
        lastError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // --- APPLY VELOCITY LIMIT ---
        output = clamp(output, -MAX_VELOCITY, MAX_VELOCITY);

        // update angle
        currentAngle += output;
        currentAngle = clamp(currentAngle, MIN_DEG ,MAX_DEG);

        // ---MAP ANGLE TO SERVO POSITION---
        double pos = (currentAngle - MIN_DEG) / (MAX_DEG - MIN_DEG);
        pos = clamp(pos, MIN_POS, MAX_POS);

        //leftyLoosy.setPosition(pos);
        //rightyTighty.setPosition(pos);

        // --- TELEMETRY ---

        telemetry.addData("Turret Target", targetAngle);
        telemetry.addData("Turret Angle", currentAngle);
        telemetry.addData("Turret Pos", pos);
        telemetry.addData("Error", error);
        telemetry.addData("Output", output);





    }

    public void stop(){

    }

    public void cmdLeft(){
            rightyTighty.setPosition(0.5);
            leftyLoosy.setPosition(0.5);
            trapezoidAutoAim.runtime.reset();
    }

    public void cmdRight(){
            rightyTighty.setPosition(-0.5);
            leftyLoosy.setPosition(-0.5);
            trapezoidAutoAim.runtime.reset();
    }

    public void cmdNo(){
        rightyTighty.setPosition(0);
        leftyLoosy.setPosition(0);
        trapezoidAutoAim.runtime.reset();
    }

    // Manual driver control for turret (joystick override)
    public void manualControl(double stick) {
        double speedDegPerLoop = 3.0;   // adjust sensitivity
        double newAngle = currentAngle + (stick * speedDegPerLoop);

        // clamp to turret limits
        newAngle = clamp(newAngle, MIN_DEG, MAX_DEG);

        // update PID target
        setTargetAngle(newAngle);
    }


    public void setTargetAngle(double angle){
        targetAngle = clamp(angle, MIN_DEG, MAX_DEG);
    }

    public double getCurrentAngle(){
        return currentAngle;
    }

    private double clamp(double v, double min, double max){
        return Math.max(min,Math.min(max,v));
    }

}