package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Robot;

@Autonomous(name = "pp6CycleBlueFar", group = "PP")
public class pp6CycleBlueFar extends OpMode {

    Robot robot = new Robot();

//tristan and wyatt's fail of an auton (with assistance)

    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private stage currentStage = stage._00_unknown;
    private ElapsedTime runtime = new ElapsedTime();
    public boolean End = false;



    public static Follower follower;
    public static Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(15, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(20, 20, Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(25, 25, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(20, 20, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(1, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1;


    public void buildPaths() {
        cyclePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1aPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1aPose.getHeading())

                .addPath(new BezierLine(pickup1aPose, pickup1bPose))
                .setLinearHeadingInterpolation(pickup1aPose.getHeading(), pickup1bPose.getHeading())

                .addPath(new BezierCurve(pickup1bPose, scorePoseAP))
                .setLinearHeadingInterpolation(pickup1bPose.getHeading(), scorePose.getHeading())

                .build();
        scorePreload = follower.pathBuilder()
                .addPath (new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
    }

    @Override
    public void init() {

// NAJ CompBotConstants is a file/class that contains the definition of the gryo and drive motors among other things
        //super.init();
        follower = CompBotConstants.createFollower(hardwareMap);
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

        robot.hardwareMap = hardwareMap;
        robot.telemetry = telemetry;
        robot.init();
    }


    @Override
    public void init_loop() {
        //super.init_loop();


        robot.init_loop();

    }

    @Override
    public void start () {
        //super.start();
        robot.start();
    }

    @Override
    public void loop() {

        updateTelemetry();
        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                currentStage = stage._10_preStart;
                break;

            case _10_preStart:
                currentStage = stage._20_PreLaunch;
                break;

            case _20_PreLaunch:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload,  true);
                    //lastPose = startPose;
                    /* currentTargetPose = scorePose;*/
                    // follower.update();
                 robot.autoRPM.Measure = true;
                    currentStage = stage._30_ScorePreload;
                }

                break;

            case _30_ScorePreload:
                if (!follower.isBusy()) {
                    dolaunch_process();
                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._40_PickupSpike1;
                }
                break;
            case _40_PickupSpike1:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._45_PreLaunch2;
                }
                break;
            case _45_PreLaunch2:
                if (follower.isBusy()) {
                    robot.autoRPM.Measure = true;
                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._50_Launch2;
                }
                break;
            case _50_Launch2:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._60_PickupConer1;
                }
                break;
            case _60_PickupConer1:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._70_PreLaunch3;
                }
                break;
            case _70_PreLaunch3:
                if (follower.isBusy()) {
                    robot.autoRPM.Measure = true;
                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._75_Launch3;
                }
                break;
            case _75_Launch3:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._80_PickupSpike2;
                }
                break;
            case _80_PickupSpike2:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._90_PreLaunch4;
                }
                break;
            case _90_PreLaunch4:
                if (follower.isBusy()) {
                    robot.autoRPM.Measure = true;
                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._100_Launch4;

                }
                break;
            case _100_Launch4:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._110_PickupCorner2;
                }
                break;
            case _110_PickupCorner2:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._120_Prelaunch5;
                }
                break;
            case _120_Prelaunch5:
                if (follower.isBusy()) {
                    robot.autoRPM.Measure = true;
                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._130_Launch5;
                }
                break;
            case _130_Launch5:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._140_PickupCorner6;
                }
                break;
            case _140_PickupCorner6:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._150_PreLaunch6;
                }
            case _150_PreLaunch6:
                if (follower.isBusy()) {
                    robot.autoRPM.Measure = true;
                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._160_Launch6;
                }
                break;
            case _160_Launch6:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._170_ParkToBeContinued;
                }
                break;
            case _170_ParkToBeContinued:
                if (follower.isBusy()) {

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }else {
                    currentStage = stage._200_end;
                }
                break;
            case _200_end:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
                    stop();
                    runtime.reset();
                    End = true;
                }
                break;



        }

    }



    @Override
    public void stop () {
        //super.stop();
        robot.stop();
    }
    private enum stage {

        _00_unknown,
        _10_preStart,
        _20_PreLaunch,
        _30_ScorePreload,
        _40_PickupSpike1,
        _45_PreLaunch2,
        _50_Launch2,
        _60_PickupConer1,
        _70_PreLaunch3,
        _75_Launch3,
        _80_PickupSpike2,
        _90_PreLaunch4,
        _100_Launch4,
        _110_PickupCorner2,
        _120_Prelaunch5,
        _130_Launch5,
        _140_PickupCorner6,
        _150_PreLaunch6,
        _160_Launch6,
        _170_ParkToBeContinued,
        _200_end;


    }

    private void dolaunch_process(){

        robot.launcherBlocker.cmdUnBlock();
        robot.transitionRoller.cmdSpin();
        robot.intake.cmdFoward();
        runtime.reset();

    }

    private void updateTelemetry () {
        telemetryMU.addData("Follower Busy?", follower.isBusy());
        telemetryMU.addData("Current Stage", currentStage);
        telemetryMU.addData("x", follower.getPose().getX());
        telemetryMU.addData("y", follower.getPose().getY());
        telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        //  telemetryMU.addData("LAST Pose", lastPose);
        //  telemetryMU.addData("Current Target Pose", currentTargetPose);
        telemetryMU.addData("breakingStrength", pathConstraints.getBrakingStrength());
        telemetryMU.addData("breakstart ", pathConstraints.getBrakingStart());
        telemetryMU.addData("drivepid P", follower.constants.coefficientsDrivePIDF.P);
        telemetryMU.addData("drivepid D", follower.constants.coefficientsDrivePIDF.D);
        telemetryMU.addData("drivepid F", follower.constants.coefficientsDrivePIDF.F);
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


}
