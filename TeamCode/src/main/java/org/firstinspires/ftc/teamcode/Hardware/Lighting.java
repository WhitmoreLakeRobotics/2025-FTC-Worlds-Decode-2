package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Lighting extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;


    public int DefaultColor;
    public int TempColor ;
    public static int TempColorTimeout = 500;

   // private Servo AllianceState;
    private Servo IntakeState;
    //private Servo LiftOffLight;

    private final static int GAMEPAD_LOCKOUT = 500;
    //public ColorAlliance CurrentColorA = ColorAlliance.OFF;
    public ColorIntake CurrentColorI = ColorIntake.OFF;
  //  public ColorLiftOff CurrentColorL = ColorLiftOff.OFF;
    public Team CurrentTeam = Team.UNKNOWN;

    public static final double Green = 0.5;
    public static final double Red = 0.28;
    public static final double Yellow = 0.388;
    public static final double Purple = 0.722;  //favwit
    public static final double Blue = 0.6111;
    public static final double Orange = 0.333;
    public static final double White = 0.9;
    public static final double Off = 0;

    private ElapsedTime initLightTime = new ElapsedTime();

    public boolean initLight1 = false;
    public boolean initLight2 = false;

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private RevBlinkinLedDriver.BlinkinPattern baseColor = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    private Telemetry.Item patternName;
    private Telemetry.Item display;
    //private RevBlinkinLedDriver.BlinkinPattern displayKind;
    private Deadline ledCycleDeadline;
    private Deadline gamepadRateLimit;

    private enum DisplayKind {
        MANUAL,
        AUTO
    }

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
    /*public Swing_Arm_And_Lift() {

    }*/

    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init(){
      /*  blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "LEDC");
        pattern = RevBlinkinLedDriver.BlinkinPattern.WHITE;
        blinkinLedDriver.setPattern(pattern);
*/
        //display = telemetry.addData("Display Kind: ", displayKind.toString());
        patternName = telemetry.addData("Pattern: ", pattern.toString());

       // AllianceState = hardwareMap.get(Servo.class, "AllianceState");
        IntakeState = hardwareMap.get(Servo.class, "IntakeState");
       // LiftOffLight = hardwareMap.get(Servo.class,"LiftOffLight");


        initLightTime.reset();

        initLight1 = true;
        initLight2 = false;

        CurrentColorI = ColorIntake.OFF;
      //  CurrentColorA = ColorAlliance.OFF;
      //  CurrentColorL = ColorLiftOff.OFF;
        cmdOFFi();
       // cmdOFFa();
      //  cmdOFFl();


    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop(){

        if (initLight1 && initLightTime.milliseconds() >= 750) {
            if(CurrentTeam == Team.RED){
       //         cmdREDa();
            }else if(CurrentTeam == Team.BLUE){
        //        cmdBLUEa();
            }else{
       //         cmdWHITEa();
            }
            initLight1 = false;
            initLight2 = true;
            initLightTime.reset();
        }

        if (initLight2 && initLightTime.milliseconds() >= 750) {
           // cmdOFFa();
            initLight2 = false;
            initLight1 = true;
            initLightTime.reset();
        }

    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){

        initLight1 = false;
        initLight2 = false;

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){
        //ReturnToBaseColor();
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

  /*  public void UpdateBaseColor (RevBlinkinLedDriver.BlinkinPattern newColor){
        // pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
        baseColor = newColor;
        blinkinLedDriver.setPattern(baseColor);

    }
    public void SetTempColor (RevBlinkinLedDriver.BlinkinPattern tempColor){
        blinkinLedDriver.setPattern(tempColor);
        runtime.reset();
    }
    private void ReturnToBaseColor () {
        if (runtime.milliseconds() > TempColorTimeout) {
            blinkinLedDriver.setPattern(baseColor);
        }

    }
    */

/* not in use
    public void cmdREDa()    { AllianceState.setPosition(Red);    CurrentColorA = ColorAlliance.RED; }
    public void cmdGREENa()  { AllianceState.setPosition(Green);  CurrentColorA = ColorAlliance.GREEN; }
    public void cmdYELLOWa() { AllianceState.setPosition(Yellow); CurrentColorA = ColorAlliance.YELLOW; }
    public void cmdPURPLEa() { AllianceState.setPosition(Purple); CurrentColorA = ColorAlliance.PURPLE; }
    public void cmdBLUEa()   { AllianceState.setPosition(Blue);   CurrentColorA = ColorAlliance.BLUE; }
    public void cmdORANGEa() { AllianceState.setPosition(Orange); CurrentColorA = ColorAlliance.ORANGE; }
    public void cmdWHITEa()  { AllianceState.setPosition(White);  CurrentColorA = ColorAlliance.WHITE; }
    public void cmdOFFa()    { AllianceState.setPosition(Off);    CurrentColorA = ColorAlliance.OFF; }
*/
    public void cmdREDi()    { IntakeState.setPosition(Red);      CurrentColorI = ColorIntake.RED; }
    public void cmdGREENi()  { IntakeState.setPosition(Green);    CurrentColorI = ColorIntake.GREEN; }
    public void cmdYELLOWi() { IntakeState.setPosition(Yellow);   CurrentColorI = ColorIntake.YELLOW; }
    public void cmdPURPLEi() { IntakeState.setPosition(Purple);   CurrentColorI = ColorIntake.PURPLE; }
    public void cmdBLUEi()   { IntakeState.setPosition(Blue);     CurrentColorI = ColorIntake.BLUE; }
    public void cmdORANGEi() { IntakeState.setPosition(Orange);   CurrentColorI = ColorIntake.ORANGE; }
    public void cmdWHITEi()  { IntakeState.setPosition(White);    CurrentColorI = ColorIntake.WHITE; }
    public void cmdOFFi()    { IntakeState.setPosition(Off);      CurrentColorI = ColorIntake.OFF; }
/*
    public void cmdREDl()    { LiftOffLight.setPosition(Red);     CurrentColorL = ColorLiftOff.RED; }
    public void cmdGREENl()  { LiftOffLight.setPosition(Green);   CurrentColorL = ColorLiftOff.GREEN; }
    public void cmdYELLOWl() { LiftOffLight.setPosition(Yellow);  CurrentColorL = ColorLiftOff.YELLOW; }
    public void cmdPURPLEl() { LiftOffLight.setPosition(Purple);  CurrentColorL= ColorLiftOff.PURPLE; }
    public void cmdBLUEl()   { LiftOffLight.setPosition(Blue);    CurrentColorL = ColorLiftOff.BLUE; }
    public void cmdORANGEl() { LiftOffLight.setPosition(Orange);  CurrentColorL = ColorLiftOff.ORANGE; }
    public void cmdWHITEl()  { LiftOffLight.setPosition(White);   CurrentColorL = ColorLiftOff.WHITE; }
    public void cmdOFFl()    { LiftOffLight.setPosition(Off);     CurrentColorL = ColorLiftOff.OFF; }

    public enum ColorAlliance { GREEN, RED, YELLOW, PURPLE, BLUE, ORANGE, WHITE, OFF }
*/
    public enum ColorIntake { GREEN, RED, YELLOW, PURPLE, BLUE, ORANGE, WHITE, OFF }

  //  public enum ColorLiftOff{ GREEN, RED, YELLOW, PURPLE, BLUE, ORANGE, WHITE, OFF }

    public enum Team { RED, BLUE, UNKNOWN}

}
