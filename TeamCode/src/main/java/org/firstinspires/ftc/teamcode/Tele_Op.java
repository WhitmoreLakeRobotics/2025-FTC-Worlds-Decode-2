//package org.firstinspires.ftc.robotcontroller.external.samples;
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Autons.TestAuton;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;
import org.firstinspires.ftc.teamcode.Common.Settings;

import org.firstinspires.ftc.teamcode.Hardware.AutoRPM;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.AutoAim;
import org.firstinspires.ftc.teamcode.Hardware.TrapezoidAutoAim;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele_Op", group = "TeleOp")
//@Disabled
public class Tele_Op extends OpMode {
    private static final String TAGTeleop = "8492-Teleop";
    //RobotTest robot = new RobotTest();
    Robot robot = new Robot();
    //    // Declare OpMode members.
    private boolean gp1_prev_a = false;
    private boolean gp1_prev_b = false;
    private boolean gp1_prev_x = false;
    private boolean gp1_prev_y = false;
    private boolean gp1_prev_right_bumper = false;
    private boolean gp1_prev_left_bumper = false;
    private boolean gp1_prev_dpad_up = false;
    private boolean gp1_prev_dpad_down = false;
    private boolean gp1_prev_dpad_left = false;
    private boolean gp1_prev_dpad_right = false;
    private boolean gp1_prev_back = false;
    private boolean gp1_prev_start = false;

    private boolean gp2_prev_a = false;
    private boolean gp2_prev_b = false;
    private boolean gp2_prev_x = false;
    private boolean gp2_prev_y = false;
    private boolean gp2_prev_right_bumper = false;
    private boolean gp2_prev_left_bumper = false;
    private boolean gp2_prev_dpad_up = false;
    private boolean gp2_prev_dpad_down = false;
    private boolean gp2_prev_dpad_left = false;
    private boolean gp2_prev_dpad_right = false;
    private boolean gp2_prev_back = false;
    private double LeftMotorPower = 0;
    private double RightMotorPower = 0;
    private boolean gp2_prev_start = false;
    private int tHeading = 0;
    private boolean bAutoTurn = false;
    private boolean EndGame = false;
    private boolean EndGame2 = false;
    private boolean EndGame3 = false;
    private boolean EndGame4 = false;
    private boolean EndGameb = false;
    private boolean EndGame2b = false;
    private boolean EndGame3b = false;
    private boolean EndGame4b = false;
    private boolean UppiesOverrideEnabled = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime Gameruntime = new ElapsedTime();
    private ElapsedTime EndGameTime = new ElapsedTime();
    private ElapsedTime Gameruntime2 = new ElapsedTime();
    private ElapsedTime EndGameTime2= new ElapsedTime();
    private ElapsedTime uppiesInhibitor = new ElapsedTime();
    private double HLIW = 500;
    public Alliance CurrentAlliance;
    //HowLongItWork

    private AutoRPM visionController;


    //*********************************************************************************************

    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // Safety Management
        //
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        // msStuckDetectInit = Settings.msStuckDetectInit;
        // msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        //msStuckDetectStart = Settings.msStuckDetectStart;
        //msStuckDetectLoop = Settings.msStuckDetectLoop;
        // msStuckDetectStop = Settings.msStuckDetectStop;

        telemetry.addData("Tele_Op", "Initialized");

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        //robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_NORMALSPEED);
        robot.init();
        visionController = new AutoRPM(robot.limey, robot.launcher);  //adding auto RPM control to launcher
        //robot.driveTrain.ResetGyro();
        //Gameruntime.reset();
        //Gameruntime2.reset();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        if(TestAuton.Alliance == "Red"){
            CurrentAlliance = Alliance.Red;
        } else if(TestAuton.Alliance == "Blue"){
            CurrentAlliance = Alliance.Blue;
        }else if(TestAuton.Alliance == "Unknown"){
            CurrentAlliance = Alliance.Unknown;
        }else{
            CurrentAlliance = Alliance.NoAuto;
        }

    }

    //*********************************************************************************************

    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        robot.init_loop();
    }

    //*********************************************************************************************

    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        Runtime.getRuntime();
       // Gameruntime.reset();
        //Gameruntime2.reset();

        if(CurrentAlliance == Alliance.Red){
            robot.intake.cmdRED();
        }else if(CurrentAlliance == Alliance.Blue){
            robot.intake.cmdBLUE();
        }else if(CurrentAlliance == Alliance.Unknown){
            robot.intake.cmdPURPLE();
        }else if(CurrentAlliance == Alliance.NoAuto){
            robot.intake.cmdYELLOW();
        }else{

        }

        // robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        //robot.signalSign.doUP();
        //robot.swing_arm_and_lift.SetPOS(Swing_Arm_And_Lift.Mode.PICKUP);
    }

    //*********************************************************************************************

    // Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {
        robot.loop();

        write2Log();
       // tHeading = getTurnDirection();                                        put back
        if (Math.abs(gamepad1.right_stick_x) > 0.04) {
            bAutoTurn = false;
        }
        if (gamepad1.right_trigger > 0.4) {
           tHeading = (int)Math.round(robot.targetAngleCalc());
            bAutoTurn = true;
        }
        //AutoAim tied to Left Trigger hold
        if (gamepad1.left_trigger > 0.2) {

            // MJD — allow auto aim to run
            robot.autoAim.setDriverOverride(false);

            double angle = robot.autoAim.computeAimAngle();

            if (!Double.isNaN(angle)) {
                int heading = (int)Math.round(angle);

                // MJD — auto-turn robot chassis toward tag
                robot.driveTrain.cmdTeleOp(
                        CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(heading),
                        DriveTrain.DTrain_NORMALSPEED
                );
            }

        } else {

            // MJD — driver regains full control when trigger released
            robot.autoAim.setDriverOverride(true);

        }





        double turretStick = gamepad2.right_stick_x;
/*
        if (Math.abs(turretStick) > 0.1) {
            robot.autoAim.setDriverOverride(true);
            robot.turret.manualControl(turretStick);
        } else {
            robot.autoAim.setDriverOverride(false);
        }

 */



        /*if(CurrentAlliance == Alliance.Red){
            robot.turret.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Red;
        } else if(CurrentAlliance == Alliance.Blue) {
            robot.turret.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Blue;
        } else if(CurrentAlliance == Alliance.NoAuto) {
            robot.turret.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.NoAuto;
        } else if(CurrentAlliance == Alliance.Unknown) {
            robot.turret.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Unknown;
        }else{
            robot.turret.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Unknown;
        }
    */



        /*
        if(Gameruntime.seconds() >= 85){
            //robot.intake.cmdYELLOW();
            Gameruntime.reset();
            //EndGameTime.reset();
           EndGame = true;
        }

        if(EndGame) {
            EndGameTime.reset();
            robot.intake.cmdYELLOW();
            EndGame2 = true;
            EndGame = false;
        }

         if (EndGameTime.milliseconds() >= 500 && EndGame2) {
        if (robot.intake.CurrentMode == Intake.Mode.NTKstop) {
            robot.intake.cmdRED();
            EndGame3 = true;
            EndGame2 = false;
        } else {
            robot.intake.cmdGREEN();
            EndGame3 = true;
            EndGame2 = false;
        }
         }

        if (EndGameTime.milliseconds() >= 1000 && EndGame3) {
            robot.intake.cmdYELLOW();
            EndGame4 = true;
            EndGame3 = false;
        }

        if (EndGameTime.milliseconds() >= 1500 && EndGame4) {
            if (robot.intake.CurrentMode == Intake.Mode.NTKstop) {
                robot.intake.cmdRED();
                EndGame4 = false;
            } else {
                robot.intake.cmdGREEN();
                EndGame4 = false;
            }
        }
//------------------------------------------------------------------------------------------------
        if(Gameruntime2.seconds() >= 100){
            //robot.intake.cmdYELLOW();
            Gameruntime2.reset();
            //EndGameTime.reset();
            EndGameb = true;
        }

        if(EndGameb) {
            EndGameTime2.reset();
            robot.intake.cmdPURPLE();
            EndGame2b = true;
            EndGameb = false;
        }

        if (EndGameTime2.milliseconds() >= 500 && EndGame2b) {
            if (robot.intake.CurrentMode == Intake.Mode.NTKstop) {
                robot.intake.cmdRED();
                EndGame3b = true;
                EndGame2b = false;
            } else {
                robot.intake.cmdGREEN();
                EndGame3b = true;
                EndGame2b = false;
            }
        }

        if (EndGameTime2.milliseconds() >= 1000 && EndGame3b) {
            robot.intake.cmdPURPLE();
            EndGame4b = true;
            EndGame3b = false;
        }

        if (EndGameTime2.milliseconds() >= 1500 && EndGame4b) {
            if (robot.intake.CurrentMode == Intake.Mode.NTKstop) {
                robot.intake.cmdRED();
                EndGame4b = false;
            } else {
                robot.intake.cmdGREEN();
                EndGame4b = false;
            }

        }
         */

        //***********   Gamepad 1 controls ********
        if (bAutoTurn) {
            if (gamepad1.right_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(tHeading), robot.driveTrain.DTrain_FASTSPEED);
            } else if (gamepad1.left_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(tHeading), robot.driveTrain.DTrain_SLOWSPEED);

            } else {

                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        robot.driveTrain.autoTurn(tHeading), robot.driveTrain.DTrain_NORMALSPEED);
            }
        } else {
            if (gamepad1.right_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x), robot.driveTrain.DTrain_FASTSPEED);
            } else if (gamepad1.left_bumper) {
                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x), robot.driveTrain.DTrain_SLOWSPEED);

            } else {

                robot.driveTrain.cmdTeleOp(CommonLogic.joyStickMath(gamepad1.left_stick_y * -1),
                        CommonLogic.joyStickMath(gamepad1.left_stick_x),
                        CommonLogic.joyStickMath(gamepad1.right_stick_x), robot.driveTrain.DTrain_NORMALSPEED);
            }

        }

        if (Math.abs(gamepad1.right_stick_y) > Settings.JOYSTICK_DEADBAND_STICK) {

        }
        //***********   Pushers
        //if (CommonLogic.oneShot(gamepad1.a, gp1_prev_a)) {
        if (gamepad1.a) {
            //robot.subPushers.cmdMoveAllDown();
            //      robot.cmdStrafeIntake();
            //    robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
        }

        if (gamepad1.b) {
            //robot.subPushers.cmdMoveAllUp();
            //  robot.cmdStrafeDelivery();
            //robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_PARTY_PALETTE);
        }
        if (CommonLogic.oneShot(gamepad1.back, gp1_prev_back)) {
            //Initialize Gyro
            robot.driveTrain.ResetGyro();
            tHeading = 0;
        }

        if (CommonLogic.oneShot(gamepad1.start, gp1_prev_start)) {
            UppiesOverrideEnabled = true;
        }

        if (gamepad1.right_trigger > 0.8) {
            robot.trapezoidAutoAim.PrimitiveDriver = false;

        }else{
            robot.trapezoidAutoAim.PrimitiveDriver = true;
        }


        if ((gamepad1.right_trigger <= 0.79) && (gamepad1.right_trigger > 0.10)) {

        }
//            robot.sweeper.setCurrentMode(Sweeper.Mode.STOP);

        // Bumpers high and lower Powers for the wheels,
        //if (CommonLogic.oneShot(gamepad1.left_bumper, gp1_prev_left_bumper)) {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_SLOWSPEED);
        //}
        if ((gamepad1.left_trigger > .6) && (gamepad1.right_trigger < .6)) {

        } else if ((gamepad1.left_trigger < .6) && (gamepad1.right_trigger > .6)) {

        } else {
            //robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_NORMALSPEED);
        }

        //if (CommonLogic.oneShot(gamepad1.right_bumper, gp1_prev_right_bumper)) {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_SLOWSPEED);
        // }
        // if (gamepad1.right_bumper) {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_TURBOSPEED);
        // RobotLog.aa(TAGTeleop,"GamepadRB: " + gamepad1.right_bumper);
        //     telemetry.addData (TAGTeleop, "GamepadRB: " + gamepad1.right_bumper);
        //  } else if(gamepad1.right_bumper == false)
        // {
        //  robot.driveTrain.setMaxPower(DriveTrain.DRIVETRAIN_NORMALSPEED);
        //}

        //***********  Grabbers
        if (CommonLogic.oneShot(gamepad1.dpad_right, gp1_prev_dpad_right)) {

        }

        if (CommonLogic.oneShot(gamepad1.dpad_up, gp1_prev_dpad_up)) {
            if(uppiesInhibitor.seconds() >= 100 || UppiesOverrideEnabled){
                robot.uppies.cmdUp();
            }

        }
        if(CommonLogic.oneShotRelease(gamepad1.dpad_up, gp1_prev_dpad_up)){
            robot.uppies.cmdStop();
        }

        if (CommonLogic.oneShot(gamepad1.dpad_left, gp1_prev_dpad_left)) {

        }

        if (CommonLogic.oneShot(gamepad1.dpad_down, gp1_prev_dpad_down)) {
            if(uppiesInhibitor.seconds() >= 100 || UppiesOverrideEnabled){
                robot.uppies.cmdDown();
            }

        }
        if(CommonLogic.oneShotRelease(gamepad1.dpad_down, gp1_prev_dpad_down)){
            robot.uppies.cmdStop();
        }

        //***********   Gamepad 2 controls ********

        // Bumpers close and open the gripper
        if (( gamepad2.left_bumper == true)) {

            LaunchTelleTouch();

        }

        if (CommonLogic.oneShot(gamepad2.right_bumper, gp2_prev_right_bumper)) {
            robot.bCkSenors = false;

        }
        if (gamepad2.right_bumper) {
            LaunchNear();
        }

        if (CommonLogic.oneShot(gamepad2.back, gp2_prev_back)) {
            robot.transitionRoller.cmdBack();

        }

        if (CommonLogic.oneShotRelease(gamepad2.back, gp2_prev_back)) {
            robot.transitionRoller.cmdStop();
        }

        if (CommonLogic.oneShot(gamepad2.start, gp2_prev_start)) {
            //robot.arm.setWristUp();

        }
        if (gamepad2.start) {
//            robot.cmdExcecuteBumpStack();   // this was SetPOS() not setting the mode
            //          robot.lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.AQUA);

        }

        if (CommonLogic.oneShot(gamepad2.a, gp2_prev_a)) {
            robot.intake.cmdFoward();
            robot.bCkSenors = true;
            robot.transitionRoller.cmdSpin();
        }

        if (CommonLogic.oneShot(gamepad2.b, gp2_prev_b)) {
            robot.intake.cmdStop();
            robot.transitionRoller.cmdStop();
        }

        if (CommonLogic.oneShot(gamepad2.y, gp2_prev_y)) {
            NoLaunch();
            robot.autoRPM.Measure = false;
        }

        if (CommonLogic.oneShot(gamepad2.x, gp2_prev_x)) {

            if (robot.intake.AtIntakeStop == false) {
                robot.intake.cmdStop();
                robot.intake.AtIntakeStop = true;
            }
            else {
                robot.intake.cmdBackward();
                robot.intake.AtIntakeStop = false;
            }
        }


        //robot.swing_arm_and_lift.SwingPos(robot.swing_arm_and_lift.LASTSWINGPOSITION + (int)(gamepad2.left_stick_x) * 5);

        if (Math.abs(gamepad2.left_stick_x) > 0.8) {
            //robot.subLifter.stickControl(-gamepad2.left_stick_y);
        }

        if (Math.abs(gamepad2.left_stick_y) > Settings.JOYSTICK_DEADBAND_STICK) {
            //robot.subLifter.stickControl(-gamepad2.left_stick_y);
            //robot.capper.cmdTeleOp((gamepad2.left_stick_y * 0.5) + (gamepad2.right_stick_y * 0.5));
        }
        else{
        }

        //robot.swing_arm_and_lift.LiftPos(robot.swing_arm_and_lift.LASTLIFTPOSITION + (int)(gamepad2.right_stick_y) * 5);

        if (Math.abs(gamepad2.right_stick_y) > Settings.JOYSTICK_DEADBAND_STICK) {
            //robot.subLifter.stickControl(-gamepad2.left_stick_y);
            //robot.capper.cmdTeleOp(gamepad2.right_stick_y * .5);
        }

        if (CommonLogic.oneShot(gamepad2.dpad_up, gp2_prev_dpad_up)) {
            LaunchLaser();
        }

        if (CommonLogic.oneShot(gamepad2.dpad_down, gp2_prev_dpad_down)) {
            if(robot.autoRPM.Measure == false){
                robot.autoRPM.Measure = true;
                LaunchAutoRPM();
            }else{
                robot.autoRPM.Measure = false;
                robot.launcher.cmdStop();
            }

        }

        if (CommonLogic.oneShot(gamepad2.dpad_right, gp2_prev_dpad_right)) {
        }

        if (CommonLogic.oneShot(gamepad2.dpad_left, gp2_prev_dpad_left)) {
        }

        if (gamepad2.right_trigger > 0.8){
            LaunchFar();
            robot.bCkSenors = false;
            }

        if ((gamepad2.right_trigger <= 0.79) && (gamepad2.right_trigger > 0.10)){
        }

        if (gamepad2.left_trigger > 0.7) { //0.8
            robot.launcherBlocker.cmdUnBlock();
        }else{
            robot.launcherBlocker.cmdBlock();  //experiment
        }

        // Update the previous status for gamepad1
        gp1_prev_a = gamepad1.a;
        gp1_prev_b = gamepad1.b;
        gp1_prev_x = gamepad1.x;
        gp1_prev_y = gamepad1.y;
        gp1_prev_left_bumper = gamepad1.left_bumper;
        gp1_prev_right_bumper = gamepad1.right_bumper;
        gp1_prev_dpad_down = gamepad1.dpad_down;
        gp1_prev_dpad_left = gamepad1.dpad_left;
        gp1_prev_dpad_up = gamepad1.dpad_up;
        gp1_prev_dpad_right = gamepad1.dpad_right;
        gp1_prev_back = gamepad1.back;
        gp1_prev_start = gamepad1.start;

        // Update the previous status for gamepad 2
        gp2_prev_a = gamepad2.a;
        gp2_prev_b = gamepad2.b;
        gp2_prev_x = gamepad2.x;
        gp2_prev_y = gamepad2.y;
        gp2_prev_left_bumper = gamepad2.left_bumper;
        gp2_prev_right_bumper = gamepad2.right_bumper;
        gp2_prev_dpad_down = gamepad2.dpad_down;
        gp2_prev_dpad_left = gamepad2.dpad_left;
        gp2_prev_dpad_up = gamepad2.dpad_up;
        gp2_prev_dpad_right = gamepad2.dpad_right;
        gp2_prev_back = gamepad2.back;
        gp2_prev_start = gamepad2.start;
    }

    //*********************************************************************************************

      //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }
/*
    private int getTurnDirection(){
        boolean a = gamepad1.a;
        boolean b = gamepad1.b;
        boolean x = gamepad1.x;
        boolean y = gamepad1.y;
        boolean RDP = gamepad1.dpad_right;
        boolean LDP = gamepad1.dpad_left;

    if(a){
        bAutoTurn = true;

            return 59;

    }
    else if (b){
        bAutoTurn = true;
        if (y){
            return -135;
        }else {
            return -45;
        }
    }
    else if (y){
        bAutoTurn = true;
        if(x){
            return 135;
        }else{
            return -59;
        }
    }
    else if(x){
        bAutoTurn = true;
       return 45;
    }

    /*else if(RDP){ //comment
        bAutoTurn = true;
        return -6;
    }
    else if(LDP){
        bAutoTurn = true;           put back
        return 6;
    }//comment
    else {
        return tHeading;
    }
   }
    */

    //*********************************************************************************************
    private void  write2Log() {

//
//    RobotLog.aa(TAGTeleop, " gp1_prev_a : " + gp1_prev_a);
//    RobotLog.aa(TAGTeleop, " gp1_prev_b : " + gp1_prev_b);
//    RobotLog.aa(TAGTeleop, " gp1_prev_x : " + gp1_prev_x);
//    RobotLog.aa(TAGTeleop, " gp1_prev_y : " + gp1_prev_y);
//    RobotLog.aa(TAGTeleop, " gp1_prev_right_bumper : " + gp1_prev_right_bumper);
//   RobotLog.aa(TAGTeleop, " gp1_prev_left_bumper : " + gp1_prev_left_bumper);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_up : " + gp1_prev_dpad_up);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_down : " + gp1_prev_dpad_down);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_left : " + gp1_prev_dpad_left);
//    RobotLog.aa(TAGTeleop, " gp1_prev_dpad_right : " + gp1_prev_dpad_right);
//
//    RobotLog.aa(TAGTeleop, " gp2_prev_a : " + gp2_prev_a);
//    RobotLog.aa(TAGTeleop, " gp2_prev_b : " + gp2_prev_b);
//    RobotLog.aa(TAGTeleop, " gp2_prev_x : " + gp2_prev_x);
//    RobotLog.aa(TAGTeleop, " gp2_prev_y : " + gp2_prev_y);
//    RobotLog.aa(TAGTeleop, " gp2_prev_right_bumper : " + gp2_prev_right_bumper);
//    RobotLog.aa(TAGTeleop, " gp2_prev_left_bumper : " + gp2_prev_left_bumper);
        //RobotLog.aa(TAGTeleop, " gp2_prev_dpad_up : " + gp2_prev_dpad_up);
        //RobotLog.aa(TAGTeleop, " gp2_prev_dpad_down : " + gp2_prev_dpad_down);
//    RobotLog.aa(TAGTeleop, " gp2_prev_dpad_left : " + gp2_prev_dpad_left);
//    RobotLog.aa(TAGTeleop, " gp2_prev_dpad_right : " + gp2_prev_dpad_right);
//
//
    }

    public void LaunchLaser() {         //wait for launcher to spin up to speed.
        robot.launcher.cmdoutlaser();
        if (robot.launcher.bAtSpeed) {
            if(robot.launcherBlocker.AtUnBlocked == true){
                robot.transitionRoller.cmdSpin();
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }
        }
    }

    public void LaunchTelleTouch() {         //wait for launcher to spin up to speed.
        robot.launcher.cmdOuttelletouch();
        if (robot.launcher.bAtSpeed) {
            if(robot.launcherBlocker.AtUnBlocked == true){
                robot.transitionRoller.cmdSpin();
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }
        }
    }

    public void LaunchTouch() {         //wait for launcher to spin up to speed.
        robot.launcher.cmdOuttouch();
        if (robot.launcher.bAtSpeed) {
            if(robot.launcherBlocker.AtUnBlocked == true){
                robot.transitionRoller.cmdSpin();
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }
        }
    }

    public void LaunchNear(){         //wait for launcher to spin up to speed.
        robot.launcher.cmdOutnear();
        if (robot.launcher.bAtSpeed) {
           if(robot.launcherBlocker.AtUnBlocked == true){
                        robot.transitionRoller.cmdSpin();
            }
            if(robot.launcherBlocker.AtUnBlocked == false) {
                robot.transitionRoller.cmdStop();
            }
        }
    }

    public void LaunchFar(){          //wait for launcher to spin up to speed.
        robot.launcher.cmdOutfar();
        if (robot.launcher.bAtSpeed){
            if(robot.launcherBlocker.AtUnBlocked == true) {
                robot.transitionRoller.cmdSpin();
            }
            else{
                robot.transitionRoller.cmdStop();
            }
            }
        }

    public void LaunchAutoRPM() {
        double[] rpms = visionController.calculateRPMs(
                robot.limey.getTx(),
                robot.limey.getTy(),
                robot.limey.getTagAngle()
        );

        robot.launcher.setTargetRPMs(rpms[0], rpms[1]);

        if (robot.launcher.bAtSpeed) {
            if (robot.launcherBlocker.AtUnBlocked) {
                robot.transitionRoller.cmdSpin();
            } else {
                robot.transitionRoller.cmdStop();
            }
        }
    }

    public void TurretAutoAim() {

    }



    public void NoLaunch(){
        robot.transitionRoller.cmdStop();
        robot.launcherBlocker.cmdBlock();
        robot.launcher.cmdStop();
    }


public enum Alliance{
        Red,
        Blue,
        Unknown,
        NoAuto
}












}