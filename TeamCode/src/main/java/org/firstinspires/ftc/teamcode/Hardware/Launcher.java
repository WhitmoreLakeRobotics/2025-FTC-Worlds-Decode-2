package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;


/**
 * Base class for FTC Team 8492 defined hardware
 */
@Configurable
public class Launcher extends BaseHardware{


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    private TelemetryManager telemetryMU;
    public HardwareMap hardwareMap = null; // will be set in Child class


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */

    private DcMotorEx LaunchM01 ;
    private DcMotorEx LaunchM02 ;

    public Mode CurrentMode;
    public Position CurrentPosition;
    private double LaunchM01Power;
    private double LaunchM02Power;
    private VoltageSensor Pikachu;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    /* naj commenting these unused items out to clear the configurables in pp Panels
        public static  double stopSpeed = 0;
        public static  double topSpeednear =  0.5;
        public static  double topSpeedfar =  1;
        public static  double bottomSpeednear = 0.5;
        public static  double bottomSpeedfar = 1;
    */

    // ---------------- PID CONSTANTS ----------------
    public static double LkP = 0.00020;   // increased for faster recovery
    public static double LkI = 0.0;       // still unused
    public static double LkD = 0.0000015; // small D for damping
    public static double kF = 1.0 / 6000.0; // feedforward per RPM

    // ---------------- RPM TARGETS ----------------
    public static double topMotorRPMnear = 2900;
    public static double bottomMotornear = 3600;
    public static double topMotorRPMfar = 2850;
    public static double bottomMotorfar = 4650; //was4600
    public static double topMotorRPMtouch = 2200; //was 2300
    public static double bottomMotortouch = 3930; //was 3800
    public static double topMotorRPMTelletouch = 2300;
    public static double bottomMotorRPMTelletouch = 4000;
    public static double topMotorRPMlaser = 6000;
    public static double bottomMotorRPMlaser = 6000;

    private double targetRPM1 = 0;
    private double targetRPM2 = 0;

    private double targetRPM1Tol = 50;
    private double targetRPM2Tol = 50;
    public boolean bAtSpeed = false;

    //  NEW PER-MOTOR PID STATE
    private double lastError1 = 0;
    private double lastError2 = 0;

    private double integral1 = 0;
    private double integral2 = 0;

    private ElapsedTime timer1 = new ElapsedTime();
    private ElapsedTime timer2 = new ElapsedTime();

    //  public static final double snailoutSpeed = -0.25;
    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    @Override
    public void init() {

        Pikachu = hardwareMap.get(VoltageSensor.class, "Expansion Hub 3");

        LaunchM02 = hardwareMap.get(DcMotorEx.class, "LaunchM02");
        LaunchM01 = hardwareMap.get(DcMotorEx.class, "LaunchM01");

        LaunchM01.setDirection(DcMotorSimple.Direction.REVERSE);
        LaunchM02.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();
        PanelsConfigurables.INSTANCE.refreshClass(this);
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
        runPID();

        if ((CommonLogic.inRange(getMotorRPM(LaunchM01),targetRPM1,targetRPM1Tol)) &&
                (CommonLogic.inRange(getMotorRPM(LaunchM02),targetRPM2,targetRPM2Tol))
        ){
            bAtSpeed = true;
        } else {
            bAtSpeed = false;
        }

        // double voltage = hardwareMap.voltageSensor.get("Expansion Hub 3").getVoltage();
        // {
        // telemetry.addData("Battery Voltage", voltage);
        // telemetry.update();
        // }
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

    public void setTargetRPMs(double top, double bottom) {
        targetRPM1 = top;
        targetRPM2 = bottom;
    }

    public void cmdOuttouch(){
        CurrentMode = Mode.LaunchMout;
        CurrentPosition = Position.LaunchNear;
        targetRPM1 = topMotorRPMtouch;
        targetRPM2 = bottomMotortouch;
    }

    public void cmdOuttelletouch(){
        CurrentMode = Mode.LaunchMout;
        CurrentPosition = Position.LaunchNear;
        targetRPM1 = topMotorRPMTelletouch;
        targetRPM2 = bottomMotorRPMTelletouch;
    }

    public void cmdoutlaser(){
        CurrentMode = Mode.LaunchMout;
        CurrentPosition = Position.LaunchFar;
        targetRPM1 = topMotorRPMlaser;
        targetRPM2 = bottomMotorRPMlaser;
    }

    public void cmdOutnear(){
        CurrentMode = Mode.LaunchMout;
        CurrentPosition = Position.LaunchNear;
        targetRPM1 = topMotorRPMnear;
        targetRPM2 = bottomMotornear;
    }

    public void cmdOutfar(){
        CurrentMode = Mode.LaunchMout;
        CurrentPosition = Position.LaunchFar;
        targetRPM1 = topMotorRPMfar;
        targetRPM2 = bottomMotorfar;
    }

/*
    public void cmdAutoRPM(){
        CurrentMode = Mode.LaunchMout;
        CurrentPosition = Position.LaunchCalc;

        targetRPM1 = robot.autoRPM

    }
 */

    public void cmdStop(){
        CurrentMode = Mode.LaunchMstop;

        targetRPM1 = 0;
        targetRPM2 = 0;
    }

    public double getMotorRPM(DcMotorEx motor){
        double ticksPerRevolution = 28; //update and double check
        double gearRatio = 1.0; //update and double check
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60 * gearRatio;
    }

    // NEW FIXED PID FOR MOTOR 1
    private double pidMotor1(double currentRPM) {
        double error = targetRPM1 - currentRPM;
        double dt = timer1.seconds();
        timer1.reset();

        if (dt <= 0) dt = 0.001;

        double p = LkP * error;

        integral1 += error * dt;
        double i = LkI * integral1;

        double derivative = (error - lastError1) / dt;
        lastError1 = error;
        double d = LkD * derivative;

        double voltage = Pikachu.getVoltage();
        double ff = (targetRPM1 * kF) * (12.0 / voltage);

        return ff + p + i + d;
    }

    // NEW FIXED PID FOR MOTOR 2
    private double pidMotor2(double currentRPM) {
        double error = targetRPM2 - currentRPM;
        double dt = timer2.seconds();
        timer2.reset();

        if (dt <= 0) dt = 0.001;

        double p = LkP * error;

        integral2 += error * dt;
        double i = LkI * integral2;

        double derivative = (error - lastError2) / dt;
        lastError2 = error;
        double d = LkD * derivative;

        double voltage = Pikachu.getVoltage();
        double ff = (targetRPM2 * kF) * (12.0 / voltage);

        return ff + p + i + d;
    }
    public void runPID(){
        double currentRPM1 = getMotorRPM(LaunchM01);
        double power1 = pidMotor1(currentRPM1);

        double currentRPM2 = getMotorRPM(LaunchM02);
        double power2 = pidMotor2(currentRPM2);

        LaunchM01.setPower(power1);
        LaunchM02.setPower(power2);

        /*
        telemetryMU.addData("Target targetRPM1",targetRPM1);
        telemetryMU.addData("Current currentRPM1",currentRPM1);
        telemetryMU.addData("Motor 1",power1);
        telemetryMU.addData("launch motor 1 velocity", LaunchM01.getVelocity());
        //telemetry.update();

        telemetryMU.addData("Target targetRPM2",targetRPM2);
        telemetryMU.addData("Current currentRPM2",currentRPM2);
        telemetryMU.addData("Motor Power2",power2);
        telemetryMU.update();
        */
    }

    public enum Mode {
        LaunchMout,
        LaunchMstop;
    }

    public enum Position {
        LaunchFar,
        LaunchNear,
        LaunchCalc;
    }

}
