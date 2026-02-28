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

    // store tag pose in camera space for AutoAim
    private double tagXCam = 0;   // MJD
    private double tagYCam = 0;   // MJD
    private double tagZCam = 0;   // MJD

    // camera pose in robot space (in meters)
    // X=0, Y=0, Z=13.5 in → convert to meters
    private static final double CAM_X_ROBOT = 0.0;                 // MJD
    private static final double CAM_Y_ROBOT = 0.0;                 // MJD
    private static final double CAM_Z_ROBOT = 13.5 * 0.0254;       // MJD
    private static final double CAM_YAW_DEG = 0.0;                 // MJD
    private static final double CAM_PITCH_DEG = 0.0;               // MJD
    private static final double CAM_ROLL_DEG = 0.0;                // MJD

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

                //store camera‑space tag position for 3D AutoAim
                tagXCam = x;   // MJD
                tagYCam = y;   // MJD
                tagZCam = z;   // MJD

                double fullDistance = Math.sqrt(x*x + y*y + z*z);

                telemetry.addData("Full 3D Distance", "%.2f", fullDistance);
            }

            // double[] tp = getBotposeTargetSpace();  // MJD
            // if(tp != null) {
            //     double x = tp[0];
            //     double y = tp[1];
            //     double yaw = tp[5];
            // }

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

    /*
    public double [] getBotposeTargetSpace() {
        if(result == null) return null;        // MJD
        if (tagID == -1) return null;          // MJD

        Pose3D bot = result.getBotpose();
        if (bot == null) return null;          // MJD

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
    */

    //2D tag pose in ROBOT SPACE (for yaw-only) — MJD
    public double[] getTagPoseRobotSpace() {   // MJD
        if (result == null) return null;       // MJD
        if (!result.isValid()) return null;    // MJD
        if (result.getFiducialResults().isEmpty()) return null;  // MJD

        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace(); // MJD
        if (pose == null) return null;                                             // MJD

        double camX = pose.getPosition().x;   // camera +X = right   // MJD
        double camZ = pose.getPosition().z;   // camera +Z = forward // MJD

        // Robot frame: +X = left, +Y = forward
        double robotX = -camX;                // right(+) → left(-) → negate  // MJD
        double robotY = camZ;                 // forward is forward           // MJD

        return new double[]{                  // MJD
                robotX,
                robotY,
                pose.getPosition().z,
                pose.getOrientation().getRoll(),
                pose.getOrientation().getPitch(),
                pose.getOrientation().getYaw()
        };
    }

    // 3D tag pose in ROBOT SPACE (for full 3D aiming)
    // Returns [x, y, z] in robot coordinates (meters)
    public double[] getTagPoseRobotSpace3D() {          // MJD
        if (result == null) return null;                // MJD
        if (!result.isValid()) return null;             // MJD
        if (result.getFiducialResults().isEmpty()) return null;  // MJD

        Pose3D pose = result.getFiducialResults().get(0).getTargetPoseCameraSpace(); // MJD
        if (pose == null) return null;                                              // MJD

        // Camera-space position (meters) — Limelight convention:
        // X: right, Y: down, Z: forward
        double cx = pose.getPosition().x;   // MJD
        double cy = pose.getPosition().y;   // MJD
        double cz = pose.getPosition().z;   // MJD

        // Camera orientation relative to robot (we assume yaw=0, pitch=0, roll=0)
        double yawRad   = Math.toRadians(CAM_YAW_DEG);   // MJD
        double pitchRad = Math.toRadians(CAM_PITCH_DEG); // MJD
        double rollRad  = Math.toRadians(CAM_ROLL_DEG);  // MJD

        // For now, with yaw=pitch=roll=0, rotation is identity.
        // If you later tilt/rotate the camera, apply full R = Rz(yaw)*Ry(pitch)*Rx(roll).

        // Map camera-space to robot-space:
        // Robot: +X = left, +Y = forward, +Z = up
        // Camera: +X = right, +Y = down, +Z = forward
        double rx = -cx;   // right(+) → left(-) → negate           // MJD
        double ry = cz;    // forward is forward                    // MJD
        double rz = CAM_Z_ROBOT - cy; // camera is at Z=CAM_Z, +Y down → subtract to go up // MJD

        // Add camera translation in robot space (here X=0, Y=0, Z=CAM_Z_ROBOT)
        double tagRobotX = CAM_X_ROBOT + rx;   // MJD
        double tagRobotY = CAM_Y_ROBOT + ry;   // MJD
        double tagRobotZ = rz;                 // MJD

        return new double[]{                   // MJD
                tagRobotX,
                tagRobotY,
                tagRobotZ
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
