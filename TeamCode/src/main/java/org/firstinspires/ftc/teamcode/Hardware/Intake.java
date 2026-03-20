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
    public Lighting lighting;

    private DcMotorEx NTKM01;
    public ColorRangeSensor NTKAP2;
    public ColorRangeSensor NTKAP3;

    public Mode CurrentMode = Mode.NTKstop;
    public Distance2 CurrentDistance2 = Distance2.MISSING2;
    public Distance3 CurrentDistance3 = Distance3.MISSING3;

    public boolean AtIntakeStop = true;

    public static final double stopSpeed = 0;
    public static final double inSpeed = -1;
    public static final double outSpeed = 0.65;
    public static final double autoSpeed = -1.0;

    private double NTKAP2distance = 999;
    private double NTKAP3distance = 999;

    private ElapsedTime loopTime = new ElapsedTime();
    private ElapsedTime sensorTime = new ElapsedTime();

    private final double targRange = 6;

    public void init() {

        NTKAP3 = hardwareMap.get(ColorRangeSensor.class, "NTKAP3");
        NTKAP2 = hardwareMap.get(ColorRangeSensor.class, "NTKAP2");
        NTKM01 = hardwareMap.get(DcMotorEx.class, "NTKM01");

        NTKM01.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        NTKM01.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        NTKM01.setPower(0);

        sensorTime.reset();

        CurrentMode = Mode.NTKstop;
    }

    public void init_loop() {

    }

    public void start() {
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
                //cmdBLUE(); // MJD: LED feedback for sensor 2 seeing object
            } else {
                CurrentDistance2 = Distance2.MISSING2;
            }

            if (NTKAP3distance <= targRange && sensorStable) {
                CurrentDistance3 = Distance3.FILLED3;
                //cmdPURPLE(); // MJD: LED feedback for sensor 3 seeing object
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
                //cmdYELLOW();    // MJD: LED feedback for auto-stop
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
        lighting.cmdGREENi();
        loopTime.reset();
    }

    public void cmdFoward() {
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower(inSpeed);
        sensorTime.reset();
        loopTime.reset();
        lighting.cmdGREENi(); // MJD: LED feedback for intake running
    }

    public void cmdStop() {
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower(stopSpeed);
        lighting.cmdREDi(); // MJD: LED feedback for intake stopped
        loopTime.reset();
    }

    public void cmdAutoFoward() {
        CurrentMode = Mode.NTKautoIn;
        NTKM01.setPower(autoSpeed);
    }

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

    public double getMotorRPM(DcMotorEx motor) {
        double ticksPerRevolution = 28;
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60;
    }
}
