package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Field.DecodeField;
import org.firstinspires.ftc.teamcode.Field.DecodeField.TagPose;

public class AutoAim {

    private final Limey limey;
    private final Turret turret;
    private final DriveTrain driveTrain;

    private boolean driverOverride = true;

    // Distance behind the tag to aim at
    private static final double OFFSET_INCHES = 8.0;
    private static final double OFFSET = OFFSET_INCHES * 0.0254;  // meters

    //launcher position in robot space (meters)
    // If launcher is at robot center, leave these 0.
    private static final double LAUNCHER_X = 0.0;   // MJD
    private static final double LAUNCHER_Y = 0.0;   // MJD
    private static final double LAUNCHER_Z = 0.0;   // MJD

    public AutoAim(Limey limey, Turret turret, DriveTrain driveTrain) {
        this.limey = limey;
        this.turret = turret;
        this.driveTrain = driveTrain;
    }

    public void setDriverOverride(boolean override) {
        this.driverOverride = override;
    }

    // FIELD-BASED AUTO AIM — MJD
    public double computeAimAngle() {   // MJD

        // Must see a tag
        if (limey.getTagID() == -1) return Double.NaN;   // MJD

        //  Robot pose from Limey
        double[] robot = limey.getRobotPoseFieldSpace();   // MJD
        if (robot == null || robot.length < 3) return Double.NaN;   // MJD

        double robotX = robot[0];   // MJD
        double robotY = robot[1];   // MJD
        double robotHeading = robot[2];   // MJD

        //  Tag pose from field map — MJD
        TagPose tag = DecodeField.getTagPose(limey.getTagID());   // MJD
        if (tag == null) return Double.NaN;   // MJD

        double tagX = tag.x;   // MJD
        double tagY = tag.y;   // MJD

        //  Angle from robot → tag in field space — MJD
        double angleToTag = Math.toDegrees(Math.atan2(tagY - robotY, tagX - robotX));   // MJD

        //  Convert field angle → robot-relative heading — MJD
        double desiredHeading = angleToTag - robotHeading;   // MJD

        // Normalize to [-180,180] — MJD
        desiredHeading = ((desiredHeading + 540) % 360) - 180;   // MJD

        return desiredHeading;   // MJD
    }

    // OLD CAMERA-ANGLE AUTO AIM — MJD

    /*
    public double computeAimAngle() {

        if (limey.getTagID() == -1) {
            return Double.NaN;
        }

        double tx = limey.getTx();
        if (Double.isNaN(tx)) {
            return Double.NaN;
        }

        double ty = limey.getTy();
        if (Double.isNaN(ty)) {
            return Double.NaN;
        }

        double cameraHeight = 11.0;
        double targetHeight = 14.375;
        double cameraAngle = 25.0;

        double cameraAngleRad = Math.toRadians(cameraAngle + ty);
        double distanceInches = (targetHeight - cameraHeight) / Math.tan(cameraAngleRad);

        double offsetAngleDeg = Math.toDegrees(Math.atan(OFFSET_INCHES / distanceInches));

        double correctedTx = tx - offsetAngleDeg;

        double robotHeading = driveTrain.getCurrentHeading();
        double desiredHeading = robotHeading + correctedTx;

        desiredHeading = ((desiredHeading + 540) % 360) - 180;

        return desiredHeading;
    }
    */

    // OLD BOTPOSE TARGET SPACE AUTO AIM — MJD

    /*
    public double computeAimAngle() {

        double[] tp = limey.getBotposeTargetSpace();
        if (tp == null || tp.length < 6) return Double.NaN;

        double x = tp[0];
        double y = tp[1];
        double tagYaw = tp[5];

        double angleToTag = Math.toDegrees(Math.atan2(y, x));

        double offsetX = OFFSET * Math.cos(Math.toRadians(tagYaw));
        double offsetY = OFFSET * Math.sin(Math.toRadians(tagYaw));

        double aimX = x - offsetX;
        double aimY = y - offsetY;

        double angleToAimPoint = Math.toDegrees(Math.atan2(aimY, aimX));

        double robotHeading = driveTrain.getCurrentHeading();

        double desiredHeading = robotHeading + angleToAimPoint;

        desiredHeading = ((desiredHeading + 540) % 360) - 180;

        return desiredHeading;
    }
    */

    // OLD 3D AUTO AIM — MJD

    /*
    public double computeAimAngle() {

        if (limey.getTagID() == -1) return Double.NaN;

        double[] tag = limey.getTagPoseRobotSpace3D();
        if (tag == null || tag.length < 3) return Double.NaN;

        double tagX = tag[0];
        double tagY = tag[1];
        double tagZ = tag[2];

        double dx = tagX - LAUNCHER_X;
        double dy = tagY - LAUNCHER_Y;
        double dz = tagZ - LAUNCHER_Z;

        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < 1e-6) return Double.NaN;

        double ux = dx / dist;
        double uy = dy / dist;
        double uz = dz / dist;

        double aimX = tagX - ux * OFFSET;
        double aimY = tagY - uy * OFFSET;
        double aimZ = tagZ - uz * OFFSET;

        double ax = aimX - LAUNCHER_X;
        double ay = aimY - LAUNCHER_Y;
        double az = aimZ - LAUNCHER_Z;

        double yawDeg = Math.toDegrees(Math.atan2(ax, ay));

        yawDeg = ((yawDeg + 540) % 360) - 180;

        return yawDeg;
    }
    */

    // OLD PITCH CALC — MJD
    /*

    public double computePitchAngle() {
        if (limey.getTagID() == -1) return Double.NaN;

        double[] tag = limey.getTagPoseRobotSpace3D();
        if (tag == null || tag.length < 3) return Double.NaN;

        double tagX = tag[0];
        double tagY = tag[1];
        double tagZ = tag[2];

        double dx = tagX - LAUNCHER_X;
        double dy = tagY - LAUNCHER_Y;
        double dz = tagZ - LAUNCHER_Z;

        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);
        if (dist < 1e-6) return Double.NaN;

        double ux = dx / dist;
        double uy = dy / dist;
        double uz = dz / dist;

        double aimX = tagX - ux * OFFSET;
        double aimY = tagY - uy * OFFSET;
        double aimZ = tagZ - uz * OFFSET;

        double ax = aimX - LAUNCHER_X;
        double ay = aimY - LAUNCHER_Y;
        double az = aimZ - LAUNCHER_Z;

        double horizDist = Math.sqrt(ax*ax + ay*ay);
        return Math.toDegrees(Math.atan2(az, horizDist));
    }
    */

    public void update() {   // MJD

        if (driverOverride || driveTrain == null) return;   // MJD

        double yaw = computeAimAngle();   // MJD
        if (Double.isNaN(yaw)) return;    // MJD

        // Use drivetrain IMU heading to compute absolute target — MJD
        double robotHeading = driveTrain.getCurrentHeading();   // MJD

        double targetHeading = robotHeading + yaw;   // MJD

        // Normalize to [0,360) — MJD
        targetHeading = ((targetHeading % 360) + 360) % 360;   // MJD

        // Rotate robot smoothly while driver keeps driving — MJD
        driveTrain.turnToHeading((int) targetHeading);   // MJD
    }
}
