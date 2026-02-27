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
            YawDif = limey.getTagAngle() * 0.125;
        }

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



