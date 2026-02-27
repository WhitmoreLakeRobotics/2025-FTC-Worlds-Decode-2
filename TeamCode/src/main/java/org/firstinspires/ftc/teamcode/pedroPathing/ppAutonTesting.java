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

@Autonomous(name = "ppAutonTesting", group = "PP")
public class ppAutonTesting extends OpMode {

    Robot robot = new Robot();


    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;
    private stage currentStage;
    private ElapsedTime runtime = new ElapsedTime();



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
        updateTelemetry(telemetry);//this might break the code  :)


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


        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                currentStage = stage._10_preStart;
                break;

            case _10_preStart:
                currentStage = stage._20_cyclePickup1;
                break;

            case _20_cyclePickup1:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload,  true);
                    //lastPose = startPose;
                   /* currentTargetPose = scorePose;*/
                    // follower.update();
                    robot.launcher.cmdOutfar();
                    currentStage = stage._40_end;
                }
            case _40_end:
                if (!follower.isBusy()) {
                    telemetryMU.addData("Drive Complete?", follower.isBusy());

                    runtime.reset();
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
            _20_cyclePickup1,
            _40_end;


        }


        private void updateTelemetry () {
            telemetryMU.addData("Follower Busy?", follower.isBusy());
            // telemetryMU.addData("Current Stage", currentStage);
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