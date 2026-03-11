package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.teamcode.Field.DecodeField;
import org.firstinspires.ftc.teamcode.Field.DecodeField.TagPose;

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

    // store tag pose in camera space for AutoAim
    private double tagXCam = 0;   // MJD
    private double tagYCam = 0;   // MJD
    private double tagZCam = 0;   // MJD

    // camera pose in robot space (in meters)
    private static final double CAM_X_ROBOT = 0.0;
    private static final double CAM_Y_ROBOT = 0.0;
    private static final double CAM_Z_ROBOT = 13.5 * 0.0254;
    private static final double CAM_YAW_DEG = 0.0;
    private static final double CAM_PITCH_DEG = 0.0;
    private static final double CAM_ROLL_DEG = 0.0;

    private double robotFieldX = Double.NaN;        // MJD
    private double robotFieldY = Double.NaN;        // MJD
    private double robotFieldHeadingDeg = Double.NaN; // MJD

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();
        limelight.setPollRateHz(100);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {}

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

                tagXCam = x;   // MJD
                tagYCam = y;   // MJD
                tagZCam = z;   // MJD

                double fullDistance = Math.sqrt(x*x + y*y + z*z);

                telemetry.addData("Full 3D Distance", "%.2f", fullDistance);
            }

            double[] tagRobot = getTagPoseRobotSpace3D();   // MJD
            if (tagRobot != null) {

                double tagRobotX = tagRobot[0];  // MJD
                double tagRobotY = tagRobot[1];  // MJD

                TagPose tagField = DecodeField.getTagPose(tagID);  // MJD

                if (tagField != null) {

                    // Robot position = tagField - tagRobot
                    robotFieldX = tagField.x - tagRobotX;  // MJD
                    robotFieldY = tagField.y - tagRobotY;  // MJD

                    robotFieldHeadingDeg = tagField.headingDeg - tagAngle;   // MJD
                    robotFieldHeadingDeg = ((robotFieldHeadingDeg % 360) + 360) % 360; // MJD
                }
            }

            telemetry.addData("Robot Field X", robotFieldX);        // MJD
            telemetry.addData("Robot Field Y", robotFieldY);        // MJD
            telemetry.addData("Robot Field Heading", robotFieldHeadingDeg); // MJD

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
            robotFieldX = Double.NaN;        // MJD
            robotFieldY = Double.NaN;        // MJD
            robotFieldHeadingDeg = Double.NaN; // MJD
            telemetry.addData("Limelight", "NO TARGET");
        }
    }

    @Override
    void stop() {}

    public void method(){}

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

    // 2D tag pose in ROBOT SPACE (yaw-only)
    public double[] getTagPoseRobotSpace() {
        if (result == null) return null;
        if (!result.isValid()) return null;
        if (result.getFiducialResults().isEmpty()) return null;

        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
        if (pose == null) return null;

        double camX = pose.getPosition().x;
        double camZ = pose.getPosition().z;

        double robotX = -camX;
        double robotY = camZ;

        return new double[]{
                robotX,
                robotY,
                pose.getPosition().z,
                pose.getOrientation().getRoll(),
                pose.getOrientation().getPitch(),
                pose.getOrientation().getYaw()
        };
    }

    // 3D tag pose in ROBOT SPACE
    public double[] getTagPoseRobotSpace3D() {
        if (result == null) return null;
        if (!result.isValid()) return null;
        if (result.getFiducialResults().isEmpty()) return null;

        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace();
        if (pose == null) return null;

        double cx = pose.getPosition().x;
        double cy = pose.getPosition().y;
        double cz = pose.getPosition().z;

        double rx = -cx;
        double ry = cz;
        double rz = CAM_Z_ROBOT - cy;

        double tagRobotX = CAM_X_ROBOT + rx;
        double tagRobotY = CAM_Y_ROBOT + ry;
        double tagRobotZ = rz;

        return new double[]{
                tagRobotX,
                tagRobotY,
                tagRobotZ
        };
    }

    // robot pose in field space getter
    public double[] getRobotPoseFieldSpace() {  // MJD
        return new double[]{ robotFieldX, robotFieldY, robotFieldHeadingDeg }; // MJD
    }

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
