package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

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

    public double computeAimAngle() {

        /*
        2D CAMERA-ANGLE
        */

        /*
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
        */

        /*
        OLD BOTPOSE TARGET SPACE CODE
        PRESERVED BUT DISABLED — MJD
        */

        /*
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
        */

        //NEW 3D AUTO AIM
        if (limey.getTagID() == -1) return Double.NaN;   // MJD

        double[] tag = limey.getTagPoseRobotSpace3D();   // MJD
        if (tag == null || tag.length < 3) return Double.NaN;  // MJD

        double tagX = tag[0];   // left/right — MJD
        double tagY = tag[1];   // forward/back — MJD
        double tagZ = tag[2];   // height — MJD

        // Compute distance from launcher → tag
        double dx = tagX - LAUNCHER_X;   // MJD
        double dy = tagY - LAUNCHER_Y;   // MJD
        double dz = tagZ - LAUNCHER_Z;   // MJD

        double dist = Math.sqrt(dx*dx + dy*dy + dz*dz);  // MJD
        if (dist < 1e-6) return Double.NaN;              // MJD

        // Unit direction vector from launcher → tag
        double ux = dx / dist;   // MJD
        double uy = dy / dist;   // MJD
        double uz = dz / dist;   // MJD

        // Move aim point 8 inches behind tag
        double aimX = tagX - ux * OFFSET;   // MJD
        double aimY = tagY - uy * OFFSET;   // MJD
        double aimZ = tagZ - uz * OFFSET;   // MJD

        // Vector from launcher → aim point
        double ax = aimX - LAUNCHER_X;   // MJD
        double ay = aimY - LAUNCHER_Y;   // MJD
        double az = aimZ - LAUNCHER_Z;   // MJD

        // Compute yaw (left/right)
        double yawDeg = Math.toDegrees(Math.atan2(ax, ay));   // MJD

        // Compute pitch (up/down)
        double horizDist = Math.sqrt(ax*ax + ay*ay);          // MJD
        double pitchDeg = Math.toDegrees(Math.atan2(az, horizDist));  // MJD

        // Normalize yaw
        yawDeg = ((yawDeg + 540) % 360) - 180;   // MJD

        // If turret only uses yaw, return yaw
        return yawDeg;   // MJD
    }

    public double computePitchAngle() {   // MJD
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
        return Math.toDegrees(Math.atan2(az, horizDist));   // MJD
    }

    public void update() {

        if (driverOverride || turret == null) {
            return;
        }

        double yaw = computeAimAngle();
        if (Double.isNaN(yaw)) return;

        turret.setTargetAngle(yaw);   // MJD — yaw only for now
    }

}
