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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.TrapezoidAutoAim;
import org.firstinspires.ftc.teamcode.Tele_Op;

@Disabled
@Autonomous(name = "TheInnocentSystemThatDoesAbsolutelyNothing", group = "PP")
public class SystemX extends OpMode {

    Robot robot = new Robot();


    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private stage currentStage = stage._00_unknown;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean Auton = true;
    public boolean intakeFull = false;


    public  static  double powerSlow = 0.3;
    public static double powerNormal = 0.65;
    public static double powerFast = 0.8;


    public static Follower follower;
    public Pose currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(follower.getPose().getHeading()));
    public static Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(15, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePose2 = new Pose(72, 96, Math.toRadians(10));

    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(20, 20, Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(25, 25, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(20, 20, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(1, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1, interruptedPickup,scorePreload2;


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
                .addPath (new BezierLine(currentPose, scorePose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose.getHeading())
                .build();

        scorePreload2 = follower.pathBuilder()
                .addPath (new BezierLine(currentPose, scorePose2))
                .setLinearHeadingInterpolation(currentPose.getHeading(), scorePose2.getHeading())
                .build();

    }

    @Override
    public void init() {

        if(robot.TeleOpRunning){
            Auton = false;
        }else{
            Auton = true;
        }

        if(Auton){
            intakeFull = true;
        }

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
                if(intakeFull) {
                    if (robot.trapezoidAutoAim.CurrentTurretColor == TrapezoidAutoAim.TurretColor.Red) {
                        if (follower.getPose().getY() <= 36) {

                            currentStage = stage._20_prelaunchRN;
                        } else {
                            currentStage = stage._20_prelaunchRF;
                        }
                    } else {
                        if (follower.getPose().getY() <= 36) {

                            currentStage = stage._20_prelaunchBN;
                        } else {
                            currentStage = stage._20_prelaunchBF;
                        }
                    }
                }else{
                    if (robot.trapezoidAutoAim.CurrentTurretColor == TrapezoidAutoAim.TurretColor.Red) {
                        if (follower.getPose().getY() <= 36) {

                            currentStage = stage._20_pickUp1RN;
                        } else {
                            currentStage = stage._20_pickUp1RF;
                        }
                    } else {
                        if (follower.getPose().getY() <= 36) {

                            currentStage = stage._20_pickUp1BN;
                        } else {
                            currentStage = stage._20_pickUp1BF;
                        }
                    }
                }


                break;

            case _20_prelaunchRN:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload, true);
                  //  robot.launcher.cmdOutfar();
                    robot.autoRPM.Measure = true;
                    runtime.reset();
                    currentStage = stage._30_LaunchN;

            }
             break;

            case _20_pickUp1BF:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload2, true);  // go pickup if effort + auto
                    //  robot.launcher.cmdOutfar();
                    robot.autoRPM.Measure = true;
                    runtime.reset();
                    currentStage = stage._30_LaunchF;

                }
                break;

            case _20_prelaunchRF:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload2, true);
                    //  robot.launcher.cmdOutfar();
                    robot.autoRPM.Measure = true;
                    runtime.reset();
                    currentStage = stage._30_LaunchF;

                }
                break;

            case _20_prelaunchBN:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload, true);
                    //  robot.launcher.cmdOutfar();
                    robot.autoRPM.Measure = true;
                    runtime.reset();
                    currentStage = stage._30_LaunchN;

                }
                break;

            case _20_prelaunchBF:
                if(!follower.isBusy()){
                    follower.followPath(scorePreload2, true);
                    //  robot.launcher.cmdOutfar();
                    robot.autoRPM.Measure = true;
                    runtime.reset();
                    currentStage = stage._30_LaunchF;

                }
                break;


            case _30_LaunchN:
                if (!follower.isBusy() || runtime.milliseconds() > 1000) {
                    dolaunch_process();
                    currentStage = stage._40_RunningCornerPickup;
                    runtime.reset();
                }
                break;


            case _40_RunningCornerPickup:
                if (runtime.milliseconds() > 500) {
                    robot.launcherBlocker.cmdBlock();
                    robot.autoRPM.Measure = false;
                    robot.launcher.cmdStop();


                if (!follower.isBusy()) {
                    follower.followPath(cyclePickup1, true);
                    //if we have 3 artifacts stop the path and go to next stage
                    runtime.reset();
                    currentStage = stage._46_RunningCheck;
                }

                    telemetryMU.addData("Corner pickup", follower.getPose());
                }
                    break;

            case _46_RunningCheck:
                if (robot.intake.CurrentColor == Intake.Color.RED || !follower.isBusy()) {
                    follower.breakFollowing();
                    newPath();
                    currentStage = stage._50_Launch1;
                    runtime.reset();

                }


            case _50_Launch1:
                if (!follower.isBusy()){

                  robot.autoRPM.Measure = true;
                 dolaunch_process();
                      runtime.reset();
                    currentStage = stage._100_end;
                }


            case _100_end:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());
stop();
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
            _20_prelaunchRN,
            _20_prelaunchRF,
            _20_prelaunchBN,
            _20_prelaunchBF,
            _20_pickUp1RN,
            _20_pickUp1RF,
            _20_pickUp1BN,
            _20_pickUp1BF,
            _30_LaunchN,
            _30_LaunchF,
            _40_RunningCornerPickup,
            _46_RunningCheck,
            _50_Launch1,
            _100_end;


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
        private  void newPath(){
            interruptedPickup = follower.pathBuilder()
                    .addPath (new BezierLine(follower.getPose(), scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
                    follower.followPath(interruptedPickup,true);

        }


}