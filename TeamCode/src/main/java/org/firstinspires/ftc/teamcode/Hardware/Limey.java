package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Limey extends BaseHardware {

    public Telemetry telemetry;
    public HardwareMap hardwareMap;

    private Limelight3A limelight;
    private LLResult result;

    private double tx = 0;
    private double ty = 0;
    private double tagDistance = 0;
    private double tagAngle = 0;
    private int tagID = -1;

    // MJD — store tag pose in camera space for AutoAim
    private double tagXCam = 0;   // MJD
    private double tagZCam = 0;   // MJD

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        limelight.setPollRateHz(100);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        result = limelight.getLatestResult();

        if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {

            var tag = result.getFiducialResults().get(0);

            tagID = tag.getFiducialId();
            tx = tag.getTargetXDegrees();
            ty = tag.getTargetYDegrees();

            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose != null) {
                double x = pose.getPosition().x;
                double y = pose.getPosition().y;
                double z = pose.getPosition().z;

                tagDistance = pose.getPosition().z;
                tagAngle = pose.getOrientation().getYaw();

                // MJD — store camera‑space tag position for AutoAim
                tagXCam = x;   // MJD
                tagZCam = z;   // MJD

                double fullDistance = Math.sqrt(x*x + y*y + z*z);

                telemetry.addData("Full 3D Distance", "%.2f", fullDistance);
            }

            double[] tp = getBotposeTargetSpace();
            if(tp != null) {
                double x = tp[0];
                double y = tp[1];
                double yaw = tp[5];
            }

            telemetry.addData("Limelight", "VALID TARGET");
            telemetry.addData("Tag ID", tagID);
            telemetry.addData("tx", "%.2f°", tx);
            telemetry.addData("ty", "%.2f°", ty);
            telemetry.addData("Distance", "%.2f", tagDistance);
            telemetry.addData("Yaw", "%.2f°", tagAngle);
            telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());

            Pose3D botPose = result.getBotpose();
            if (botPose != null) {
                telemetry.addData("Bot Pose (X,Y,Z)", "%.2f, %.2f, %.2f",
                        botPose.getPosition().x,
                        botPose.getPosition().y,
                        botPose.getPosition().z);
            }

        } else {
            tagID = -1;
            telemetry.addData("Limelight", "NO TARGET");

            if (result != null) {
                telemetry.addData("Valid", result.isValid());
                telemetry.addData("Latency", "%.1f ms", result.getTargetingLatency());
            }
        }
    }

    @Override
    void stop() {

    }

    public void method(){

    }





    // Returns: [x, y, z, roll, pitch, yaw]
    public double[] getBotPose() {
        if (result == null) return null;

        Pose3D bot = result.getBotpose();
        if (bot == null) return null;

        return new double[]{
                bot.getPosition().x,
                bot.getPosition().y,
                bot.getPosition().z,
                bot.getOrientation().getRoll(),
                bot.getOrientation().getPitch(),
                bot.getOrientation().getYaw()
        };
    }

    public double [] getBotposeTargetSpace() {
        if(result == null) return null;
        if (tagID == -1) return null;

        Pose3D bot = result.getBotpose();
        if (bot == null) return null;

        double robotX = bot.getPosition().x;
        double robotY = bot.getPosition().y;
        double robotHeadingDeg = bot.getOrientation().getYaw();

        double tagX = tagXCam;
        double tagZ = tagZCam;

        double headingRad = Math.toRadians(robotHeadingDeg);

        double dx = tagZ * Math.cos(headingRad) - tagX * Math.sin(headingRad);
        double dy = tagZ * Math.cos(headingRad) - tagX * Math.sin(headingRad);

        double tagFieldX = robotX + dx;
        double tagFieldY = robotY + dy;

        double relX = robotX - tagFieldX;
        double relY = robotY - tagFieldY;

        double Yaw = Math.toDegrees(Math.atan2(relY, relX));

        return new double[]{
                relX,
                relY,
                0,
                0, 0,
                Yaw
        };




    }

    // Getters
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public int getTagID() { return tagID; }
    public double getTagDistance() { return tagDistance; }
    public double getTagAngle() {

        if(result == null) return Double.NaN;
        if(!result.isValid()) return Double.NaN;
        if(tagID == -1) return Double.NaN;

        return tagAngle;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void setHardwareMap(HardwareMap hw) {
        this.hardwareMap = hw;
    }
}
