package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.CompBotConstants.pathConstraints;

import com.bylazar.configurables.PanelsConfigurables;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Intake;
import org.firstinspires.ftc.teamcode.Hardware.Robot;
import org.firstinspires.ftc.teamcode.Hardware.TrapezoidAutoAim;
import org.firstinspires.ftc.teamcode.Tele_Op;

@Disabled
@Autonomous(name = "TheInnocentSystemThatDoesAbsolutelyNothing", group = "PP")
public class SystemX extends OpMode {

    Robot robot = new Robot();
    ppNearRed6 ppNearRed6 = new ppNearRed6();
    pp6CycleBlueFar pp6CycleBlueFar = new pp6CycleBlueFar();


    private String thisUpdate = "0";
    public String choosenAuton = "None";
    private TelemetryManager telemetryMU;
    public stage currentStage = stage._00_unknown;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean Auton = false;
    public boolean intakeFull = false;
    //public boolean did1First = false;
    public boolean completed2 = false;
    public boolean phaseCompleted = false;
    public boolean goToFarLaunch = false;
    public boolean goToPickTunnel = false;
    public boolean main = false;

    public static String Alliance;

    public  static  double powerSlow = 0.3;
    public static double powerNormal = 0.65;
    public static double powerFast = 0.8;

    public static Follower follower;
    public Pose currentPose = new Pose(follower.getPose().getX(), follower.getPose().getY(), Math.toRadians(follower.getPose().getHeading()));
    public static Pose startPose = new Pose(10, 10, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose scorePose = new Pose(15, 15, Math.toRadians(114)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose launchFarRed = new Pose(48, 96, Math.toRadians(135));
    public static Pose launchFarBlue = new Pose(96, 96, Math.toRadians(45));
    public static Pose pickTunnelRed = new Pose(19, 35, Math.toRadians(190));
    public static Pose pickTunnelBlue = new Pose(125, 35, Math.toRadians(-10));
    public static Pose scorePose2 = new Pose(72, 96, Math.toRadians(10));
    public static Pose scoreCheck = new Pose(90,135,(Math.toRadians(90)));   //check
    public static Pose startPose2 = new Pose(110, 135, Math.toRadians(90));
    public static Pose scoreCheckCorrect = new Pose (54,135, Math.toRadians(-90));//check
    public static Pose spikeB1start = new Pose (35,84,Math.toRadians(90));
    public static Pose spikeB1end = new Pose (15,84,Math.toRadians(90));
    public static Pose spikeB2start = new Pose (35,60,Math.toRadians(90));
    public static Pose spikeB2end = new Pose (15,60,Math.toRadians(90));
    public static Pose spikeB3start = new Pose (35,36,Math.toRadians(90));
    public static Pose spikeB3end = new Pose (15,36,Math.toRadians(90));
    public static Pose spikeR1start = new Pose (110,84,Math.toRadians(-90));
    public static Pose spikeR1end = new Pose (130,84,Math.toRadians(-90));
    public static Pose spikeR2start = new Pose (110,60,Math.toRadians(-90));
    public static Pose spikeR2end = new Pose (130,60,Math.toRadians(-90));
    public static Pose spikeR3start = new Pose (110,36,Math.toRadians(-90));
    public static Pose spikeR3end = new Pose (130,36,Math.toRadians(-90));
    public static Pose LaunchRN = new Pose(60,84,Math.toRadians(-45)); // -45 is placeholder
    public static Pose LaunchBN = new Pose(84,84,Math.toRadians(45)); // 45 is placeholder


    //private final Pose scorePose = new Pose(wallScoreX, wallScoreY, wallScoreH); // seeing if configurables work for this. Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAP = new Pose(20, 20, Math.toRadians(10));
    public static Pose pickup1aPose = new Pose(25, 25, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup1bPose = new Pose(20, 20, Math.toRadians(190)); // (First Set) of Artifacts picked up.
    public static Pose pickup1bPoseC = new Pose(1, 27, Math.toRadians(200));
    public static Pose pickup1cPose = new Pose(4, 13.5, Math.toRadians(180));

    private PathChain scorePreload;
    private PathChain grabPickup1, grabPickup1a, grabPickup1b, grabPickup1c, scorePickup1, grabPickup2a, grabPickup2b, scorePickup2, goEndPose, goEndPose2, endPath;
    private PathChain cyclePickup1, interruptedPickup,scorePreload2,checkColor, correctPos, correctPos2, spikeB1, spikeB2, spikeB3, spikeR1, spikeR2, spikeR3, doLaunchRN, doLaunchBN, doLaunchRF, doLaunchBF, doPickupBT ,doPickupRT;

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

        checkColor = follower.pathBuilder()
                .addPath (new BezierLine(startPose2, scoreCheck))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreCheck.getHeading())
                .build();

        correctPos = follower.pathBuilder()
                .addPath (new BezierLine(scoreCheckCorrect, scoreCheckCorrect))
                .setLinearHeadingInterpolation(scoreCheckCorrect.getHeading(), scoreCheckCorrect.getHeading())
                .build();

        correctPos = follower.pathBuilder()
                .addPath (new BezierLine(scoreCheckCorrect,scoreCheckCorrect))
                .setLinearHeadingInterpolation(scoreCheckCorrect.getHeading(), scoreCheckCorrect.getHeading())
                .build();

        correctPos2 = follower.pathBuilder()
                .addPath (new BezierPoint(scoreCheckCorrect))   //  vtest2v       ^test1^
                .setLinearHeadingInterpolation(scoreCheckCorrect.getHeading(), scoreCheckCorrect.getHeading())
                .build();

        spikeB1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeB1start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeB1start.getHeading())

                .addPath (new BezierLine(spikeB1start,spikeB1end))
                .setLinearHeadingInterpolation(spikeB1start.getHeading(), spikeB1end.getHeading())
                .build();

        spikeB2 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeB2start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeB2start.getHeading())

                .addPath (new BezierLine(spikeB2start,spikeB2end))
                .setLinearHeadingInterpolation(spikeB2start.getHeading(), spikeB2end.getHeading())
                .build();

        spikeB3 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeB3start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeB3start.getHeading())

                .addPath (new BezierLine(spikeB3start,spikeB3end))
                .setLinearHeadingInterpolation(spikeB3start.getHeading(), spikeB3end.getHeading())
                .build();

        spikeR1 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR1start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR1start.getHeading())

                .addPath (new BezierLine(spikeR1start,spikeR1end))
                .setLinearHeadingInterpolation(spikeR1start.getHeading(), spikeR1end.getHeading())
                .build();

        spikeR2 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR2start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR2start.getHeading())

                .addPath (new BezierLine(spikeR2start,spikeR2end))
                .setLinearHeadingInterpolation(spikeR2start.getHeading(), spikeR2end.getHeading())
                .build();

        spikeR3 = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, spikeR3start))
                .setLinearHeadingInterpolation(currentPose.getHeading(), spikeR3start.getHeading())

                .addPath (new BezierLine(spikeR3start,spikeR3end))
                .setLinearHeadingInterpolation(spikeR3start.getHeading(), spikeR3end.getHeading())
                .build();

        doLaunchRN = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,LaunchRN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), LaunchRN.getHeading())
                .build();

        doLaunchBN = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,LaunchBN))
                .setLinearHeadingInterpolation(currentPose.getHeading(), LaunchBN.getHeading())
                .build();

        doLaunchRF = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,launchFarRed))
                .setLinearHeadingInterpolation(currentPose.getHeading(), launchFarRed.getHeading())
                .build();

        doLaunchBF = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,launchFarBlue))
                .setLinearHeadingInterpolation(currentPose.getHeading(), launchFarBlue.getHeading())
                .build();

        doPickupBT = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,pickTunnelBlue))
                .setLinearHeadingInterpolation(currentPose.getHeading(), pickTunnelBlue.getHeading())
                .build();

        doPickupRT = follower.pathBuilder()
                .addPath (new BezierLine(currentPose,pickTunnelRed))
                .setLinearHeadingInterpolation(currentPose.getHeading(), pickTunnelRed.getHeading())
                .build();


    }



    @Override
    public void init() {

        if(robot.TeleOpRunning){
            Auton = false;
        }else{
            Auton = true;
        }

        main = true;

// NAJ CompBotConstants is a file/class that contains the definition of the gryo and drive motors among other things
        //super.init();
       // follower = CompBotConstants.createFollower(hardwareMap);     p
   //     buildPaths();                                  maybe p
      //  PanelsConfigurables.INSTANCE.refreshClass(this);                 p
      //  follower.setStartingPose(startPose);                   p
      //  follower.update();                                        p
//  pedroPanelsTelemetry.init();
        //Drawing.init();                                                  p
        //telemetryMU = PanelsTelemetry.INSTANCE.getTelemetry();                  p

// disp[lay starting postition
        telemetryMU.addData("initialized postition - Update ", thisUpdate);
// Feedback to Driver Hub for debugging
        //updateTelemetry();                         p

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
        
        telemetry.addData("Auton Running",choosenAuton);

//updateTelemetry();                                                                   p
        telemetry.addData("Auton_Current_Stage ", currentStage);
        robot.autonLoop();
        follower.update();
        switch (currentStage) {
            case _00_unknown:
                    currentStage = stage._07_AutoPos;
                break;

            case _07_AutoPos:
                if(robot.limey.getTagID() == 24){  //if red
                    robot.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Red;
                    findAlliance();
                    currentStage = stage._RedFar;

                }else if(robot.limey.getTagID() == 20){//if blue
                    robot.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Blue;
                    findAlliance();
                    currentStage = stage._BlueFar;
                }else {
                    //follower.followPath(checkColor,true);                   p
                    runtime.reset();
                    currentStage = stage._08_AutoPos2;
                }
                break;

            case _08_AutoPos2:
                //if(!follower.isBusy() || runtime.milliseconds() >= 1500) {                   p
                   if(robot.limey.getTagID() == 24){  // use color senser
                       robot.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Red;
                       findAlliance();
                      // follower.setStartingPose(scoreCheckCorrect);                         p
                       //follower.followPath(correctPos,true);    // theory, NOT TESTED!!!
                   runtime.reset();                                     //if not working try correctPos2(less likely to work)
                    currentStage = stage._RedNear;                  // based on logical conclusion
                }else if(robot.limey.getTagID() == 20) { // use color senser
                       robot.trapezoidAutoAim.CurrentTurretColor = TrapezoidAutoAim.TurretColor.Blue;
                       findAlliance();
                       runtime.reset();
                       currentStage = stage._BlueNear;
                   }
              //  }                                 *w*           p

                break;
                
            case _RedFar:
                choosenAuton = "Red Far";
                //currentStage = stage._100_end;
                

                break;

            case _RedNear:
                ppNearRed6.init();
                choosenAuton = "Red Near";
                currentStage = stage._RedNear2;
                //currentStage = stage._100_end;

                break;

            case _BlueFar:
                pp6CycleBlueFar.init();
                choosenAuton = "Blue Far";
                currentStage = stage._BlueFar2;
                //currentStage = stage._100_end;

                break;

            case _BlueNear:
                choosenAuton = "Blue Near";
                //currentStage = stage._100_end;

                break;

            case _RedFar2:
                //choosenAuton = "Red Far";
                //currentStage = stage._100_end;


                break;

            case _RedNear2:
                ppNearRed6.init_loop();
                currentStage = stage._RedNear3;
                //choosenAuton = "Red Near";
                //currentStage = stage._100_end;

                break;

            case _BlueFar2:
                pp6CycleBlueFar.init_loop();
                currentStage = stage._BlueFar3;
                //choosenAuton = "Blue Far";
                //currentStage = stage._100_end;

                break;

            case _BlueNear2:
                //choosenAuton = "Blue Near";
                //currentStage = stage._100_end;

                break;

            case _RedFar3:
                //choosenAuton = "Red Far";
                //currentStage = stage._100_end;

                break;
                
            case _RedNear3:
                ppNearRed6.start();
                currentStage = stage._RedNear4;
                //choosenAuton = "Red Near";
                //currentStage = stage._100_end;

                break;

            case _BlueFar3:
                pp6CycleBlueFar.start();
                currentStage = stage._BlueFar4;
                //choosenAuton = "Blue Far";
                //currentStage = stage._100_end;

                break;

            case _BlueNear3:
                //choosenAuton = "Blue Near";
                //currentStage = stage._100_end;
                
                break;

            case _RedFar4:
                //choosenAuton = "Red Far";
                //currentStage = stage._100_end;

                break;

            case _RedNear4:
                if(ppNearRed6.End){
                    ppNearRed6.stop();
                    currentStage = stage._100_end;
                }else {
                    ppNearRed6.loop();
                }
                //choosenAuton = "Red Near";
                //currentStage = stage._100_end;

                break;

            case _BlueFar4:
                if(pp6CycleBlueFar.End){
                    pp6CycleBlueFar.stop();
                    currentStage = stage._100_end;
                }else {
                    pp6CycleBlueFar.loop();
                }
                //choosenAuton = "Blue Far";
                //currentStage = stage._100_end;

                break;

            case _BlueNear4:
                //choosenAuton = "Blue Near";
                //currentStage = stage._100_end;

                break;

            case _100_end:
                //if (!follower.isBusy()) {        p
                   // telemetryMU.addData("Drive Complete?", follower.isBusy());   p
                //}           p
                break;
}
if(!Auton) {
    if (goToFarLaunch) {
        if (robot.trapezoidAutoAim.CurrentTurretColor == TrapezoidAutoAim.TurretColor.Red) {
            follower.followPath(doLaunchRF, false);

        } else {
            follower.followPath(doLaunchBF, false);

        }
        goToFarLaunch = false;
    }
}

        if(!Auton) {
            if (goToPickTunnel) {
                if (robot.trapezoidAutoAim.CurrentTurretColor == TrapezoidAutoAim.TurretColor.Red) {
                    follower.followPath(doPickupBT, false);

                } else {
                    follower.followPath(doPickupRT, false);

                }
                goToPickTunnel = false;
            }
        }



        }



        @Override
        public void stop () {
            //super.stop();
            robot.stop();
        }
        private enum stage {

            _00_unknown,
            _05_findTorA,
            _07_AutoPos,
            _08_AutoPos2,
            _10_preStart,
            //_20_prelaunchRN,
            //_20_prelaunchRF,
            //_20_prelaunchBN,
            //_20_prelaunchBF,
            //_20_pickUp1RN,
            //_20_pickUp1RF,
            //_20_pickUp1BN,
            //_20_pickUp1BF,
            _30_Launch1,
            _30_LaunchF,
            _40_RunningCornerPickup,
            _46_RunningCheck,
            _50_Launch1,
            _RedNear,
            _RedFar,
            _BlueNear,
            _BlueFar,
            _RedNear2,
            _RedFar2,
            _BlueNear2,
            _BlueFar2,
            _RedNear3,
            _RedFar3,
            _BlueNear3,
            _BlueFar3,
            _RedNear4,
            _RedFar4,
            _BlueNear4,
            _BlueFar4,
            _100_end;

        }

        private void dolaunch_process(){
            robot.launcherBlocker.cmdUnBlock();
        robot.transitionRoller.cmdSpin();
            robot.intake.cmdFoward();
        runtime.reset();
    }

/*
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

 */
    /*
        private  void newPath(){
            interruptedPickup = follower.pathBuilder()
                    .addPath (new BezierLine(follower.getPose(), scorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                    .build();
                    follower.followPath(interruptedPickup,true);

        }

     */

    public void findAlliance(){   // use color senser
        double currentTagId = robot.limey.getTagID();
        if(currentTagId == 20){
            Alliance = "Blue";
        }else if(currentTagId == 24){
            Alliance = "Red";
        }else{
            Alliance = "Unknown";
        }
    }


}