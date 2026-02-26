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

@Autonomous(name = "ppAutonTesting", group = "PP")
public class ppAutonTesting extends OpMode{


    private String thisUpdate = "0";
    private TelemetryManager telemetryMU;


    public static Follower follower;
    public static Pose startPose = new Pose(57, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(57, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP =new Pose(52,18,Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(20, 20, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(12, 15, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(23, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a,grabPickup2b, scorePickup2 ,goEndPose, goEndPose2, endPath;
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






    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void stop() {
        super.stop();
    }

private void updateTelemetry() {
    telemetryMU.addData("Follower Busy?", follower.isBusy());
   // telemetryMU.addData("Current Stage", currentStage);
    telemetryMU.addData("x", follower.getPose().getX());
    telemetryMU.addData("y", follower.getPose().getY());
    telemetryMU.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
  //  telemetryMU.addData("LAST Pose", lastPose);
  //  telemetryMU.addData("Current Target Pose", currentTargetPose);
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
}
