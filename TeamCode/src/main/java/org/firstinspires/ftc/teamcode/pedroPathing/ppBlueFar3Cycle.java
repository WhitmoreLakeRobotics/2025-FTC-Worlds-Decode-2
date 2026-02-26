package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


@Configurable
@Autonomous(name = "ppBlueFar3Cycle", group = "PP")
// @Autonomous(...) is the other common choice

public class ppBlueFar3Cycle extends OpMode {

    //RobotComp robot = new RobotComp();
    Robot robot = new Robot();
    private stage currentStage = stage._unknown;
    // declare auton power variables
    //private double AUTO_DRIVE_TURBO_SPEED = DriveTrain.DRIVETRAIN_TURBOSPEED;
    //private double AUTO_DRIVE_SLOW_SPEED = DriveTrain.DRIVETRAIN_SLOWSPEED;
    // private double AUTO_DRIVE_NORMAL_SPEED = DriveTrain.DRIVETRAIN_NORMALSPEED;
    // private double AUTO_TURN_SPEED = DriveTrain.DRIVETRAIN_TURNSPEED;

    private String RTAG = "8492-Auton";
// Set up stuff for pedro path

    private String thisUpdate = "11";
    private TelemetryManager telemetryMU;
    //Private Follower follower;
    public static Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime pTimer;// this is for pausing at the end of a path
    //configurables for pedro
    public static double powerCreeper = 0.15;
    public  static  double powerSlow = 0.3;
    public static double powerNormal = 0.65;
    public static double powerFast = 0.8;
    // poses for pedropath
    // poses for pedropath
    public static Pose startPose = new Pose(57, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(57, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP =new Pose(52,18,Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(20, 20, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(12, 15, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(23, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));

    public static Pose pickup2aPose = new Pose(9, 35.5, Math.toRadians(190)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup2aPoseC = new Pose(71, 39, Math.toRadians(190)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    //public static Pose pickReturn2 =new Pose(20,75,(180));
    //public static Pose pickup3aPose = new Pose(47, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    //public static Pose pickup3bPose = new Pose(15, 35, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static Pose endPose = new Pose(11,15,Math.toRadians(180));
    public static Pose endPose2 = new Pose(11,15,Math.toRadians(180));
    private Pose currentTargetPose = startPose;
    private Pose lastPose = startPose;
    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a,grabPickup2b, scorePickup2 ,goEndPose, goEndPose2, endPath;

    // private Path grabPickup1a;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
     /*   scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        scorePreload.setHeadingConstraint(0.1);
        scorePreload.setVelocityConstraint(2.0);*/

        scorePreload = follower.pathBuilder()
                .addPath (new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1cPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1cPose.getHeading())
                .build();
        grabPickup1a = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())
                .build();
        grabPickup1b = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1aPose,pickup1bPoseC,pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())
                .build();

        grabPickup1c = follower.pathBuilder()
                .addPath(new BezierLine(pickup1bPose,pickup1cPose))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), pickup1cPose.getHeading())
                .build();


        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1cPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1cPose.getHeading(), scorePose.getHeading()).setHeadingConstraint(0.1)
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2a = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose,pickup2aPoseC,pickup2aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2aPose.getHeading())
                .build();

        //grabPickup2b = follower.pathBuilder()
        //        .addPath(new BezierLine(pickup2aPose, pickup2aPoseC))
         //       .setLinearHeadingInterpolation(pickup2aPose.getHeading(), pickup2aPoseC.getHeading())
         //       .build();
/*
        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
      /*  scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2bPose, pickup2aPose))
                .setLinearHeadingInterpolation(pickup2bPose.getHeading(), pickup1aPose.getHeading())
                .addPath(new BezierLine(pickup2bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePoseAP.getHeading())
                .build();*/
        //tring a curve
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2aPose,scorePoseAP))
                .setLinearHeadingInterpolation(pickup2aPose.getHeading(), scorePose.getHeading())
                // .addPath(new BezierLine(pickup2bPose, scorePoseAP))
                //.setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePoseAP.getHeading())
                .build();

        // This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. *//*
        goEndPose= follower.pathBuilder()
                .addPath(new BezierLine(scorePose,endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();

        // This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. *//*
         goEndPose2= follower.pathBuilder()
                .addPath(new BezierLine(endPose,endPose2))
                .setLinearHeadingInterpolation(endPose.getHeading(), endPose2.getHeading())
                .build();
        //endPath = follower.pathBuilder()
          //      .addPath(new BezierCurve(scorePose, pickup2aPose, endPose))
                //.setLinearHeadingInterpolation(scorePoseAP.getHeading(), pickup2aPose.getHeading())
                //.addPath(new BezierLine(pickup2aPose, pickup2bPose))
         //       .setLinearHeadingInterpolation(scorePose.getHeading(),endPose.getHeading())
          //      .build();
    }




    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    //Code to run ONCE when the driver hits INIT

    @Override
    public void init() {
        //----------------------------------------------------------------------------------------------
        // These constants manage the duration we allow for callbacks to user code to run for before
        // such code is considered to be stuck (in an infinite loop, or wherever) and consequently
        // the robot controller application is restarted. They SHOULD NOT be modified except as absolutely
        // necessary as poorly chosen values might inadvertently compromise safety.
        //----------------------------------------------------------------------------------------------
        msStuckDetectInit = Settings.msStuckDetectInit;
        msStuckDetectInitLoop = Settings.msStuckDetectInitLoop;
        msStuckDetectStart = Settings.msStuckDetectStart;
        msStuckDetectLoop = Settings.msStuckDetectLoop;
        msStuckDetectStop = Settings.msStuckDetectStop;

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
        telemetry.addData("Test Auton", "Initialized");

        //Initialize Gyro
        robot.driveTrain.ResetGyro();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        pTimer = new ElapsedTime();

        follower =  CompBotConstants.createFollower(hardwareMap);
        buildPaths();
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(startPose);
        follower.update();
        //  pedroPanelsTelemetry.init();
        Drawing.init();
        telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();

        // disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
        // Feedback to Driver Hub for debugging
        updateTelemetry();

    }


    //Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY

    @Override
    public void init_loop() {
        // initialize robot
        robot.init_loop();

    }


    //Code to run ONCE when the driver hits PLAY

    @Override
    public void start() {
        // start robot
        runtime.reset();
        robot.start();
        opmodeTimer.resetTimer();


    }


    //Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP

    @Override
    public void loop() {

        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _unknown:
                currentStage = stage._00_preStart;
                break;

            case _00_preStart:
                currentStage = stage._20_DriveToScore;
                break;

            case _20_DriveToScore:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, powerNormal, true);
                    lastPose = startPose;
                    currentTargetPose = scorePose;
                    // follower.update();
                    robot.launcher.cmdOutfar();
                    currentStage = stage._25_checkDrivetoscore;
                }
            case _25_checkDrivetoscore:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    currentStage = stage._30_Shoot1; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;

            case _30_Shoot1:
                if (!follower.isBusy()) {
                    if (runtime.milliseconds() >= 500) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        // if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                        //         CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        runtime.reset();
                        currentStage = stage._40_LauncherStop;
                    }}
                break;

            case _40_LauncherStop:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                   // currentStage = stage._50_Pickup1;
                    currentStage = stage._50_Pickup1;
                }
                break;

            case _50_Pickup1:
                if (!follower.isBusy()) {
                  // follower.followPath(grabPickup1a, powerNormal, true);
                  //  follower.followPath(grabPickup1,powerNormal,true);
                    follower.followPath(grabPickup1,powerNormal,true);
                    robot.intake.cmdFoward();
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup1cPose;
                   // currentStage = stage._55_Pickup1_Startintake;
                    currentStage = stage._65_Pickup1b;
                }
                break;

            case _55_Pickup1_Startintake:
                if (!follower.isBusy()) {
                  //  follower.followPath(grabPickup1a, true);
                   // currentTargetPose = pickup1aPose;
                    robot.intake.cmdFoward();
                    runtime.reset();
                    currentStage = stage._60_Pickup1a;
                }
                break;

            case _60_Pickup1a:
                if (!follower.isBusy()) {
                  follower.followPath(grabPickup1b,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup1bPose;
                    currentStage = stage._65_Pickup1b;
                    runtime.reset();
                }

                break;

            case _65_Pickup1b:
                if (!follower.isBusy() || runtime.milliseconds() > 3500) {
                   // follower.followPath(grabPickup1c,powerSlow, true);
                    //if we have 3 artifacts stop the path and go to next stage
                    if (robot.intake.CurrentColor == Intake.Color.RED){
                        follower.breakFollowing();
                        currentStage = stage._70_ToScorePoseAP;
                        runtime.reset();

                    }
                  /*  if (runtime.milliseconds() < 300 ){
                        follower.turnToDegrees(185);
                    }
                    else{
                        follower.turnToDegrees(175); //wiggle to pick up more
                    }*/
                   // lastPose = currentTargetPose;
                    //currentTargetPose = pickup1cPose;
                    currentStage = stage._66_PickupWiggle;
                    runtime.reset();
                }

                break;
            case _66_PickupWiggle:
                if (!follower.isBusy()) {
                   // follower.followPath(grabPickup1c, powerSlow, true);
                    if (runtime.milliseconds() < 100) {
                        follower.turnToDegrees(185);
                    } else {
                        follower.turnToDegrees(175); //wiggle to pick up more
                    }

                    currentStage = stage._70_ToScorePoseAP;
                    runtime.reset();
                }
                break;

            case _70_ToScorePoseAP:
                if(!follower.isBusy() || runtime.milliseconds() > 1000){
                    follower.followPath(scorePickup1,powerNormal,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = scorePose;
                    robot.launcher.cmdOutfar();
                    currentStage = stage._75_chkDrive_to_score_P1;
                }
                break;
            case _75_chkDrive_to_score_P1:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    currentStage = stage._80_ScorePickup1; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }
                break;

            case _80_ScorePickup1:
                if (!follower.isBusy()) {
                    //                   if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //                           CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    if (runtime.milliseconds() >= 1000) {
                        telemetryMU.addLine("wqiting to shoot 2");
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        currentStage = stage._90_LauncherStop;
                        runtime.reset();
                    }}

            case _90_LauncherStop:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    currentStage = stage._100_Pickup2;
                }
                break;

            case _100_Pickup2:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2a, powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = pickup2aPose;
                    currentStage = stage._110_Pickup2_Startintake;
                }
                break;

            case _110_Pickup2_Startintake:
                if (!follower.isBusy()) {
                    // follower.followPath(grabPickup1a, true);
                    currentTargetPose = pickup2aPose;
                    robot.intake.cmdFoward();
                    currentStage = stage._130_ToScorePoseAP;
                }
                /*
                break;

            case _120_Pickupa2:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2 ,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose= pickup2aPose;
                    currentStage = stage._130_ToScorePoseAP;
                }

                 */
                break;
            case _130_ToScorePoseAP:
                if(!follower.isBusy()){
                    follower.followPath(scorePickup2,powerNormal,true);
                    currentTargetPose = scorePose;
                    robot.launcher.cmdOutfar();
                    currentStage = stage._140_chkDrive_to_scorePoseAP;
                }
                break;
            case _140_chkDrive_to_scorePoseAP:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    currentStage = stage._150_ScorePickup2; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }


                /*
                break;

            case _142_Pickup3:
                if (!follower.isBusy()) {
                    follower.followPath(goEndPose, powerNormal, true);
                    lastPose = currentTargetPose;
                    currentTargetPose = endPose;
                    currentStage = stage._143_Pickup3_Startintake;
                }
                break;

            case _143_Pickup3_Startintake:
                if (!follower.isBusy()) {
                    // follower.followPath(grabPickup1a, true);
                    currentTargetPose = endPose;
                    robot.intake.cmdFoward();
                    currentStage = stage._144_Pickupa2;
                }
                break;

            case _144_Pickupa2:
                if (!follower.isBusy()) {
                    follower.followPath(goEndPose ,powerSlow, true);
                    lastPose = currentTargetPose;
                    currentTargetPose= endPose2;
                    currentStage = stage._146_ToScorePoseAP;
                }
                break;
            case _146_ToScorePoseAP:
                if(!follower.isBusy()){
                    follower.followPath(goEndPose2,powerNormal,true);
                    currentTargetPose = scorePose;
                    robot.launcher.cmdOuttouch();
                    currentStage = stage._148_chkDrive_to_scorePoseAP;
                }
                break;
            case _148_chkDrive_to_scorePoseAP:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    currentStage = stage._150_ScorePickup2; // we don't need to do the turn since heading is adjusted in path
                    runtime.reset();
                }

                 */

                break;

            case _150_ScorePickup2:
                if (!follower.isBusy()) {
                    //                   if (CommonLogic.inRange(follower.getPose().getX(), wallScoreX, xTol) &&
                    //                           CommonLogic.inRange(follower.getPose().getY(), wallScoreY, yTol)) {
                    if (runtime.milliseconds() >= 1000) {
                        telemetryMU.addLine("wqiting to shoot 1");
                        robot.intake.cmdFoward();
                        robot.transitionRoller.cmdSpin();
                        robot.launcherBlocker.cmdUnBlock();
                        currentStage = stage._450_Park;
                        runtime.reset();
                    }
                }
                break;

            case _450_Park:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    follower.followPath(goEndPose, powerNormal,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = endPose;
                    currentStage = stage._500_End; //   75_ParkToBeContinued;
                }

                break;

            case _475_ParkToBeContinued:
                if (runtime.milliseconds() >= 1500) {
                    // robot.driveTrain.CmdDrive(0, 0, 0.0, 0);
                    robot.launcherBlocker.cmdBlock();
                    follower.followPath(goEndPose2, powerNormal,true);
                    lastPose = currentTargetPose;
                    currentTargetPose = endPose2;
                    currentStage = stage._500_End;
                }

                break;
            case _500_End:
            { //do nothing let the time run out
                if (runtime.milliseconds() > 2500){
                    follower.breakFollowing();
                }
            }


            break;
        }

        updateTelemetry();
    }  //  loop

    private void updateTelemetry() {
        telemetryMU.addData("Follower Busy?", follower.isBusy());
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetryMU.addData("LAST Pose", lastPose);
        telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P );
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D );
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F );
        telemetryMU.addData("CONSTRAINTS", "");
        telemetryMU.addData("Tvalue (% complete)", follower.pathConstraints.getTValueConstraint());
        telemetryMU.addData("Current tValue", follower.getCurrentTValue());
        telemetryMU.addData("Velocity Constraint", follower.pathConstraints.getVelocityConstraint());
        telemetryMU.addData("Current Velocity", follower.getVelocity());
        telemetryMU.addData("Trans constraint", follower.pathConstraints.getTranslationalConstraint());
        // telemetryMU.addData("current Trans", follower.getTranslationalError());
        telemetryMU.addData("Heading Constraint", follower.pathConstraints.getHeadingConstraint());

        telemetryMU.update();
        Drawing.drawDebug(follower);
    }

    //Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
        robot.stop();
    }

    private enum stage {
        _unknown,
        _00_preStart,
        _20_DriveToScore,
        _25_checkDrivetoscore,
        _30_Shoot1,
        _40_LauncherStop,
        _50_Pickup1,
        _55_Pickup1_Startintake,
        _60_Pickup1a,
        _65_Pickup1b,
        _66_PickupWiggle,
        _70_ToScorePoseAP,
        _75_chkDrive_to_score_P1,
        _80_ScorePickup1,
        _90_LauncherStop,
        _100_Pickup2,
        _110_Pickup2_Startintake,
        _120_Pickupa2,
        _130_ToScorePoseAP,
        _140_chkDrive_to_scorePoseAP,
        _142_Pickup3,
        _143_Pickup3_Startintake,
        _144_Pickupa2,
        _146_ToScorePoseAP,
        _148_chkDrive_to_scorePoseAP,
        _150_ScorePickup2,
        _450_Park,
        _475_ParkToBeContinued,
        _500_End

    }

}


