package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;

/**
 * Base class for FTC Team 8492 defined hardware
 */
@Disabled
public class Sensors extends BaseHardware {

    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;
    //private ColorRangeSensor IntakeSensor;
    //private DistanceSensor RearLeftSensor

    public ColorRangeSensor NTKAP2;
    public ColorRangeSensor NTKAP3;

    private boolean cmdComplete = true;
    public boolean bothFilled = false;
    private boolean sensorStable = false;
    //private Mode CurrentMode = Mode.STOP;

    private int SensorBlue;
    private int SensorRed;
    private int SensorGreen;

    public Distance2 CurrentDistance2 = Distance2.MISSING2;
    public Distance3 CurrentDistance3 = Distance3.MISSING3;

    private double NTKAP2distance = 999;
    private double NTKAP3distance = 999;

    private ElapsedTime sensorTime = new ElapsedTime();
    private final double targRange = 6;

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
        //DeliverySensor = hardwareMap.get(ColorSensor.class, "DeliveryS");

        NTKAP3 = hardwareMap.get(ColorRangeSensor.class, "NTKAP3");
        NTKAP2 = hardwareMap.get(ColorRangeSensor.class, "NTKAP2");

        sensorTime.reset();

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop() {

        //int red1 = SDC01.red();
        /*//int green1 = SDC01.green();
        int blue1 = SDC01.blue();

        int red2 = SDC02.red();
        int green2 = SDC02.green();
        int blue2 = SDC02.blue();

        int red3 = SDC03.red();
        int green3 = SDC03.green();
        int blue3 = SDC03.blue();

        int red4 = NTKC01.red();
        int green4 = NTKC01.green();
        int blue4 = NTKC01.blue();
*/




        /**
         * User defined init_loop method
         * <p>
         * This method will be called repeatedly when the INIT button is pressed.
         * This method is optional. By default this method takes no action.
         */

//         telemetry.addData("FLDS1 Pos " , FLDS1.getDistance(DistanceUnit.INCH)) ;
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

        if(sensorTime.milliseconds() >= 1000){
            sensorStable = true;
        }else{
            sensorStable = false;
        }

        getDistNTKAP2();
        getDistNTKAP3();

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

        //if (CurrentMode == Intake.Mode.NTKforward) {  // MJD
            // allow sensors to stabilize
       // }

        // STOP CONDITION
        //if (CurrentMode == Intake.Mode.NTKforward) {

                    if(CurrentDistance2 == Distance2.FILLED2 &&
                            CurrentDistance3 == Distance3.FILLED3){
                        bothFilled = true;
                    }


            // boolean rpmLoaded =
            //         CommonLogic.inRange(getMotorRPM(NTKM01), 550, 650);   // MJD: impossible RPM range

            // if (bothFilled && rpmLoaded) {   // MJD
            //     cmdStop();
            // }


        //}

    }






    public void doStop(){
        //CurrentMode = Mode.STOP;
        cmdComplete = true;
    }

    public void cmdResetSensor(){
        sensorTime.reset();
        sensorStable = false;
        CurrentDistance2 = Distance2.MISSING2;
        CurrentDistance3 = Distance3.MISSING3;
        bothFilled = false;
    }



    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */



public void stop(){

}

public TargetType getSlotArtifact(ColorSensor v3) {
    int red1 = v3.red();
    int green1 = v3.green();
    int blue1 = v3.blue();

    if ((CommonLogic.inRange(red1,TargetType.GREENT.red,TargetType.GREENT.redTol ))
    &&(CommonLogic.inRange(blue1,TargetType.GREENT.blue,TargetType.GREENT.blueTol ))
    &&(CommonLogic.inRange(green1,TargetType.GREENT.green,TargetType.GREENT.greenTol))
    ){
        return TargetType.GREENT;
    }else if ((CommonLogic.inRange(red1,TargetType.PURPLET.red,TargetType.PURPLET.redTol ))
            &&(CommonLogic.inRange(blue1,TargetType.PURPLET.blue,TargetType.PURPLET.blueTol ))
            &&(CommonLogic.inRange(green1,TargetType.PURPLET.green,TargetType.PURPLET.greenTol))
    ){
        return TargetType.PURPLET;
    }else {
        return TargetType.UNKNOWNT;
    }

}


    private void getDistNTKAP2() {
        NTKAP2distance = NTKAP2.getDistance(DistanceUnit.CM);
    }

    private void getDistNTKAP3() {
        NTKAP3distance = NTKAP3.getDistance(DistanceUnit.CM);
    }

    public enum Distance3 { FILLED3, MISSING3 }
    public enum Distance2 { FILLED2, MISSING2 }







//public enum Mode{ STOP }


    public enum TargetType {
        GREENT(25, 5, 75,25,1,3 ),
        PURPLET(6, 1, 1, 2, 7,1 ),
        UNKNOWNT(1, 1, 1,1,1,1);



        private int red;
        private int  redTol;
        private int blue;
        private int blueTol;
        private int green;
        private int greenTol;


        TargetType(int red,int redTol,int blue,int blueTol,int green,int greenTol) {
            this.red = red;
            this.redTol = redTol;
            this.blue = blue;
            this.blueTol = blueTol;
            this.green = green;
            this.greenTol = greenTol;
        }

        }



        private void updateColorSensor() {
        }




}


