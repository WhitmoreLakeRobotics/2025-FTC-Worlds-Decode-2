package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;

public class Intake extends BaseHardware {

    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;

    private DcMotorEx NTKM01;
    private Servo PeaLight;
    public ColorRangeSensor NTKAP2;
    public ColorRangeSensor NTKAP3;

    public Mode CurrentMode = Mode.NTKstop;
    public Distance2 CurrentDistance2 = Distance2.MISSING2;
    public Distance3 CurrentDistance3 = Distance3.MISSING3;
    public Color CurrentColor = Color.OFF;

    public boolean AtIntakeStop = true;

    public static final double stopSpeed = 0;
    public static final double inSpeed = -1;
    public static final double outSpeed = 0.65;
    public static final double autoSpeed = -1.0;

    public static final double Green = 0.5;
    public static final double Red = 0.28;
    public static final double Yellow = 0.388;
    public static final double Purple = 0.722;
    public static final double Blue = 0.6111;
    public static final double Orange = 0.333;
    public static final double Off = 0;

    private double NTKAP2distance = 999;
    private double NTKAP3distance = 999;

    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime sensorTime = new ElapsedTime();
    private ElapsedTime initLightTime = new ElapsedTime();

    public boolean initLight1 = false;
    public boolean initLight2 = false;

    private final double targRange = 6;

    public void init() {

        NTKAP3 = hardwareMap.get(ColorRangeSensor.class, "NTKAP3");
        NTKAP2 = hardwareMap.get(ColorRangeSensor.class, "NTKAP2");
        NTKM01 = hardwareMap.get(DcMotorEx.class, "NTKM01");
        PeaLight = hardwareMap.get(Servo.class, "PeaLight");

        NTKM01.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        NTKM01.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        NTKM01.setPower(0);

        sensorTime.reset();
        initLightTime.reset();

        initLight1 = true;
        initLight2 = false;

        CurrentMode = Mode.NTKstop;
        CurrentColor = Color.OFF;
        cmdOFF();
    }

    public void init_loop() {

        if (initLight1 && initLightTime.milliseconds() >= 750) {
            cmdORANGE();
            initLight1 = false;
            initLight2 = true;
            initLightTime.reset();
        }

        if (initLight2 && initLightTime.milliseconds() >= 750) {
            cmdOFF();
            initLight2 = false;
            initLight1 = true;
            initLightTime.reset();
        }
    }

    public void start() {
        initLight1 = false;
        initLight2 = false;
        cmdRED();
        loopTime.reset();
    }

    public void loop() {

        // SENSOR UPDATE
        if (loopTime.milliseconds() >= 50) {

            getDistNTKAP2();
            getDistNTKAP3();

            boolean sensorStable = sensorTime.milliseconds() >= 1000;

            if (NTKAP2distance <= targRange && sensorStable) {
                CurrentDistance2 = Distance2.FILLED2;
                cmdBLUE(); // MJD: LED feedback for sensor 2 seeing object
            } else {
                CurrentDistance2 = Distance2.MISSING2;
            }

            if (NTKAP3distance <= targRange && sensorStable) {
                CurrentDistance3 = Distance3.FILLED3;
                cmdPURPLE(); // MJD: LED feedback for sensor 3 seeing object
            } else {
                CurrentDistance3 = Distance3.MISSING3;
            }

            loopTime.reset();
        }

        // FIXED DEBOUNCE — old logic prevented sensors from stabilizing
        // if (CurrentMode == Mode.NTKforward && NTKM01.getPower() == inSpeed) {
        //     if (sensorTime.milliseconds() > 2000) {
        //         sensorTime.reset();
        //     }
        // }
        // MJD: Removed broken debounce

        if (CurrentMode == Mode.NTKforward) {  // MJD
            // allow sensors to stabilize
        }

        // STOP CONDITION
        if (CurrentMode == Mode.NTKforward) {

            boolean bothFilled =
                    CurrentDistance2 == Distance2.FILLED2 &&
                            CurrentDistance3 == Distance3.FILLED3;

            // boolean rpmLoaded =
            //         CommonLogic.inRange(getMotorRPM(NTKM01), 550, 650);   // MJD: impossible RPM range

            // if (bothFilled && rpmLoaded) {   // MJD
            //     cmdStop();
            // }

            if (bothFilled) {   // MJD
                cmdStop();      // MJD
                cmdYELLOW();    // MJD: LED feedback for auto-stop
            }
        }
    }

    @Override
    public void stop() {
        cmdStop();
    }

    // COMMANDS

    public void cmdBackward() {
        CurrentMode = Mode.NTKbackward;
        NTKM01.setPower(outSpeed);
        cmdGREEN();
        loopTime.reset();
    }

    public void cmdFoward() {
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower(inSpeed);
        sensorTime.reset();
        loopTime.reset();
        cmdGREEN(); // MJD: LED feedback for intake running
    }

    public void cmdStop() {
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower(stopSpeed);
        cmdRED(); // MJD: LED feedback for intake stopped
        loopTime.reset();
    }

    public void cmdAutoFoward() {
        CurrentMode = Mode.NTKautoIn;
        NTKM01.setPower(autoSpeed);
    }

    // LED COMMANDS

    public void cmdRED()    { PeaLight.setPosition(Red);    CurrentColor = Color.RED; }
    public void cmdGREEN()  { PeaLight.setPosition(Green);  CurrentColor = Color.GREEN; }
    public void cmdYELLOW() { PeaLight.setPosition(Yellow); CurrentColor = Color.YELLOW; }
    public void cmdPURPLE() { PeaLight.setPosition(Purple); CurrentColor = Color.PURPLE; }
    public void cmdBLUE()   { PeaLight.setPosition(Blue);   CurrentColor = Color.BLUE; }
    public void cmdORANGE() { PeaLight.setPosition(Orange); CurrentColor = Color.ORANGE; }
    public void cmdOFF()    { PeaLight.setPosition(Off);    CurrentColor = Color.OFF; }

    // SENSOR READS

    private void getDistNTKAP2() {
        NTKAP2distance = NTKAP2.getDistance(DistanceUnit.CM);
    }

    private void getDistNTKAP3() {
        NTKAP3distance = NTKAP3.getDistance(DistanceUnit.CM);
    }

    public enum Distance3 { FILLED3, MISSING3 }
    public enum Distance2 { FILLED2, MISSING2 }
    public enum Mode { NTKstop, NTKforward, NTKautoIn, NTKbackward }
    public enum Color { GREEN, RED, YELLOW, PURPLE, BLUE, ORANGE, OFF }

    public double getMotorRPM(DcMotorEx motor) {
        double ticksPerRevolution = 28;
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60;
    }
}
