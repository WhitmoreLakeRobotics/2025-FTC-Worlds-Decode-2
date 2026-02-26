package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
public class Uppies extends BaseHardware{

    public HardwareMap hardwareMap = null;// will be set in Child class

    public Mode CurrentMode;
    public CRServo USC;
    public boolean UP = false;
    public CRServo USCC;

public static final double UpUSCC = -1;
public static final double DownUSCC = 1;
    public static final double UpUSC = 1;
    public static final double DownUSC = -1;
    public static final double Stop = 0.0;

    public Telemetry telemetry = null;


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    public Uppies() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {


        USCC = hardwareMap.get(CRServo.class, "USCC");
        USC = hardwareMap.get(CRServo.class, "USC");

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start() {

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop() {

    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop() {

    }

    public void cmdUp(){
        CurrentMode = Mode.UP;
        USC.setPower(UpUSC);
        USCC.setPower(UpUSCC);
        UP = true;
    }

    public void cmdDown(){
        CurrentMode = Mode.DOWN;
        USC.setPower(DownUSC);
        USCC.setPower(DownUSCC);
        UP = false;

    }



    public void cmdStop(){
        CurrentMode = Mode.STOP;
        USC.setPower(Stop);
        USCC.setPower(Stop);

    }



    public enum Mode{
        UP,
        DOWN,
        STOP
    }
}
