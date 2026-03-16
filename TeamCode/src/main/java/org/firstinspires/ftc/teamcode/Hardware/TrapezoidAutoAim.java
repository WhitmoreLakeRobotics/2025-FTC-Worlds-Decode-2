package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autons.TestAuton;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Tele_Op;

import java.util.MissingFormatWidthException;
import java.util.Objects;


public class TrapezoidAutoAim {

    private Limey limey;
    // private Turret turret;
    private DriveTrain driveTrain;

    public TurretColor CurrentTurretColor;
    public Mode CurrentMode;

    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();
    public boolean PrimitiveDriver = false;
    public double YawDif = 0;
    public boolean JackHappy = false;
    public static double heading = 0;
    /*
    public TrapezoidAutoAim(Limey limey,DriveTrain driveTrain, Telemetry telemetry,HardwareMap hardwareMap){
        this.limey = limey;
        this.driveTrain = driveTrain;
        this.telemetry = telemetry;
        this.hardwareMap =hardwareMap;
        this.CurrentMode = Mode.NotTrying;
        //this.CurrentTurretColor = TurretColor.Unknown;
    }

     */

    public void init(){

        //this.limey = limey;
        //this.driveTrain = driveTrain;
        //this.telemetry = telemetry;
        // this.hardwareMap =hardwareMap;
        this.CurrentMode = Mode.NotTrying;


    }

    public void init_loop(){


    }

    public void start(){

    }

    public void loop(){
        //runtime.log("Position");
        //limey.getTx();

        if(limey == null) return;
        if(driveTrain == null) return;

        if(limey.getTagID() > -1) {
            YawDif = limey.getTagAngle() * 0.250;
        }

        if(!PrimitiveDriver) {
            if (CurrentTurretColor == TurretColor.Red) {
                if (limey.getTagID() == 24) {
                    if (limey.getTx() > 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        //maybe change to ty
                        if(limey.getTx() <= 36 || limey.getTx() >= 108){
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 4),0.35);
                            heading = heading - 4;
                            // turret.cmdRight();
                        }else{
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1), 0.35);
                            heading = heading - 1;
                        }
                    } else if (limey.getTx() < 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        if(limey.getTx() <= 36 || limey.getTx() >= 108){
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 4),0.35);
                            heading = heading + 4;
                            // turret.cmdLeft();
                        }else{
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1), 0.35);
                            heading = heading + 1;
                        }
                    } else {
                        // turret.cmdNo();
                        if(limey.getTx() == 72 + YawDif) {
                            JackHappy = true;
                            CurrentMode = Mode.Target_Acquired;
                        }

                    }
                } else {
                    // turret.cmdNo();   >:3
                    CurrentMode = Mode.Target_NotFound;



                }
            }
            if (CurrentTurretColor == TurretColor.Blue) {
                if (limey.getTagID() == 20) {
                    if (limey.getTx() > 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        //turret.cmdRight();
                        if(limey.getTx() <= 36 || limey.getTx() >= 108){
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 4),0.35);
                            heading = heading - 4;
                            // turret.cmdRight();
                        }else{
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1), 0.35);
                            heading = heading - 1;
                        }
                    } else if (limey.getTx() < 72 + YawDif) {
                        JackHappy = false;
                        CurrentMode = Mode.Targeting;
                        //turret.cmdLeft();
                        if(limey.getTx() <= 36 || limey.getTx() >= 108){
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 4),0.35);
                            heading = heading + 4;
                            // turret.cmdRight();
                        }else{
                            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1), 0.35);
                            heading = heading + 1;
                        }
                    } else {
                        // turret.cmdNo();
                        if(limey.getTx() == 72 + YawDif){
                            JackHappy = true;
                            CurrentMode = Mode.Target_Acquired;
                        }

                    }
                } else {
                    // turret.cmdNo();
                    CurrentMode = Mode.Target_NotFound;
                }
            }
        }

        // if(CurrentTurretColor == TurretColor.NoAuto || CurrentTurretColor == TurretColor.Unknown){
        //    PrimitiveDriver = true;
        // }


        if(CurrentMode == Mode.Targeting && limey.getTagID() == -1){
            CurrentMode = Mode.Target_NotFound;
        }
/*
        if(heading > 170){
            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 340),0.70);
            heading = heading + 340;
        } else if (heading < -170){
            driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 340),0.70);
            heading = heading - 340;
        }else {

        }

 */







    }

    public void stop(){

    }



    public enum Mode{
        Targeting,
        Target_Acquired,
        Target_NotFound,
        NotTrying

    }

    public enum TurretColor{
        Red,
        Blue,
        NoAuto,
        Unknown
    }

}



