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

@Disabled
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

    public void init(){

    }

    public void init_loop(){


    }

    public void start(){

    }

    public void loop(){
        //runtime.log("Position");
        //limey.getTx();


       if(limey == null) return;
       if(limey.getTagID() > -1) {
           YawDif = limey.getTagAngle() * 0.125;
       }
/*
        if(CommonLogic.inRange(limey.getTagAngle(), 12.25,11.25)){
            YawDif = 7;
        }else
        if(CommonLogic.inRange(limey.getTagAngle(), -12.25,11.25)){
            YawDif = -7;
        }else
        if(CommonLogic.inRange(limey.getTagAngle(), 33.75,10.25)){
            YawDif = 14;
        }else
        if(CommonLogic.inRange(limey.getTagAngle(), -33.75,10.25)){
            YawDif = -14;
        }else if(CommonLogic.inRange(limey.getTagAngle(), 0,0.99)){
            YawDif = 0;
        }else {

        }

 */



        if(PrimitiveDriver == false) {
            if (CurrentTurretColor == TurretColor.Red) {
                if (limey.getTagID() == 24) {
                    if (limey.getTx() >= 72 + YawDif) { //maybe change to ty
                       // turret.cmdRight();
                       driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1),0.35);
                    } else if (limey.getTx() <= 72 + YawDif) {
                       // turret.cmdLeft();
                        driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1),0.35);
                    } else {
                       // turret.cmdNo();

                    }
                } else {
                   // turret.cmdNo();
                }
            }
            if (CurrentTurretColor == TurretColor.Blue) {
                if (limey.getTagID() == 20) {
                    if (limey.getTx() >= 72 + YawDif) {
                        //turret.cmdRight();
                        driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() + 1),0.35);
                    } else if (limey.getTx() <= 72 + YawDif) {
                        //turret.cmdLeft();
                        driveTrain.cmdTurn(Math.abs(driveTrain.getCurrentHeading() - 1),0.35);
                    } else {
                       // turret.cmdNo();
                    }
                } else {
                   // turret.cmdNo();
                }
            }
        }

       // if(CurrentTurretColor == TurretColor.NoAuto || CurrentTurretColor == TurretColor.Unknown){
        //    PrimitiveDriver = true;
       // }


        if(CurrentMode == Mode.Targeting && limey.getTagID() == -1){
            CurrentMode = Mode.Target_NotFound;
        }





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



