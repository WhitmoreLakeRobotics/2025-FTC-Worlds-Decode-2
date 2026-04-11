package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends BaseHardware {

    private boolean IntakeFull = false;
    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;
    private Lighting lighting;
    private Sensors sensors;
    private LauncherBlocker launcherBlocker;
    private Robot robot;

    private DcMotorEx NTKM01;

    public Mode CurrentMode = Mode.NTKstop;

    public boolean AtIntakeStop = true;
    public boolean AtUnblocked = false;

    public static final double stopSpeed = 0;
    public static final double inSpeed = -1;
    public static final double outSpeed = 0.65;
    public static final double autoSpeed = -1.0;

    //private ElapsedTime loopTime = new ElapsedTime();
    private final double targRange = 6;


    public void init() {

        NTKM01 = hardwareMap.get(DcMotorEx.class, "NTKM01");

        NTKM01.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        NTKM01.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        NTKM01.setPower(0);


        CurrentMode = Mode.NTKstop;
    }

    public void init_loop() {

    }

    public void start() {
        //loopTime.reset();
    }

    public void loop() {

      // if(IntakeFull && !AtUnblocked){
     //      cmdStop();
           //robot.cmdTRStop();
       //}

    }


    @Override
    public void stop() {
        cmdStop();
    }

    // COMMANDS

    public void cmdBackward() {
        CurrentMode = Mode.NTKbackward;
        NTKM01.setPower(outSpeed);
        //robot.cmdSensorsReset();
     //   lighting.cmdGREENi();
     //   sensors.cmdResetSensor();
        //loopTime.reset();
    }

    public void cmdFoward() {
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower(inSpeed);
        //robot.sensors.sensorTime.reset();
        //sensorTime.reset();
        //loopTime.reset();
         // MJD: LED feedback for intake running
      //  sensors.cmdResetSensor();
      //  lighting.cmdGREENi();
    }

    public void cmdStop() {
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower(stopSpeed);
    //    lighting.cmdREDi(); // MJD: LED feedback for intake stopped
        //loopTime.reset();
    }

    public void cmdAutoFoward() {
        CurrentMode = Mode.NTKautoIn;
        NTKM01.setPower(autoSpeed);
    }

    public enum Mode { NTKstop, NTKforward, NTKautoIn, NTKbackward }

    public double getMotorRPM(DcMotorEx motor) {
        double ticksPerRevolution = 28;
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60;
    }

    public void setIntakeFull(boolean bStatus){
        IntakeFull = bStatus;
    }
    public void setAtUnblocked(boolean bStatus){AtUnblocked = bStatus;}
}
