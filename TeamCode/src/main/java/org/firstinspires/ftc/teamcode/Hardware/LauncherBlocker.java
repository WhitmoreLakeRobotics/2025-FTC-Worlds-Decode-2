package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;

import java.lang.annotation.Target;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class LauncherBlocker extends BaseHardware{

    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */

    public Servo LBS01;

    public static final double Blocked = 0.515;     //make have constant power while blocking.
    public static final double UnBlocked = 0.40;

    public boolean AtUnBlocked = false;

    public Mode CurrentMode = Mode.Stop;

    private ElapsedTime runtime = new ElapsedTime();
    private double waitTime = 50;

    public Telemetry telemetry = null;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    public LauncherBlocker() {

    }

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){

        LBS01 = hardwareMap.get(Servo.class,"LBS01");
        cmdBlock();

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop(){

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
/*
        if(AtUnBlocked == false){
            if(runtime.milliseconds() >= waitTime){
                AtUnBlocked = true;
            }
        }
*/




    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
     void stop(){

     }

     public void cmdBlock(){
         LBS01.setPosition(Blocked);
         AtUnBlocked = false;
         runtime.reset();
     }

     public void cmdUnBlock(){
         LBS01.setPosition(UnBlocked);
         AtUnBlocked = true;
         runtime.reset();
     }

     public enum Mode{
         Stop
     }



}
