package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

public class AutoAim {

    private final Limey limey;
    private final Turret turret;
    private final DriveTrain driveTrain;   // MJD — added so we can read robot heading

    private boolean driverOverride = true;

    // Distance behind the tag to aim at
    private static final double OFFSET_INCHES = 8.0;
    private static final double OFFSET = OFFSET_INCHES * 0.0254;  // convert to meters for botpose

    public AutoAim(Limey limey, Turret turret, DriveTrain driveTrain) {
        this.limey = limey;
        this.turret = turret;
        this.driveTrain = driveTrain;   // MJD — store drivetrain reference
    }

    public void setDriverOverride(boolean override) {
        this.driverOverride = override;
    }

    public double computeAimAngle() {

        /*
        // No tag detected
        if (limey.getTagID() == -1) {
            return Double.NaN;
        }

        // Horizontal angle to tag (degrees)
        double tx = limey.getTx();
        if (Double.isNaN(tx)) {
            return Double.NaN;
        }

        // Vertical angle to tag (degrees)
        double ty = limey.getTy();
        if (Double.isNaN(ty)) {
            return Double.NaN;
        }

        double cameraHeight = 11.0;      // inches — adjust for your robot
        double targetHeight = 14.375;    // inches — FTC backdrop tag height
        double cameraAngle = 25.0;       // degrees — adjust for your mount

        double cameraAngleRad = Math.toRadians(cameraAngle + ty);
        double distanceInches = (targetHeight - cameraHeight) / Math.tan(cameraAngleRad);

        double offsetAngleDeg = Math.toDegrees(Math.atan(OFFSET_INCHES / distanceInches));

        double correctedTx = tx - offsetAngleDeg;

        double robotHeading = driveTrain.getCurrentHeading();
        double desiredHeading = robotHeading + correctedTx;

        desiredHeading = ((desiredHeading + 540) % 360) - 180;

        return desiredHeading;
        */

        double[] tp = limey.getBotposeTargetSpace();  // MJD
        if (tp == null || tp.length < 6) return Double.NaN;  // MJD

        double x = tp[0];   // robot X relative to tag (meters) — MJD
        double y = tp[1];   // robot Y relative to tag (meters) — MJD
        double tagYaw = tp[5]; // robot yaw relative to tag (degrees) — MJD

        double angleToTag = Math.toDegrees(Math.atan2(y, x));  // MJD

        double offsetX = OFFSET * Math.cos(Math.toRadians(tagYaw));  // MJD
        double offsetY = OFFSET * Math.sin(Math.toRadians(tagYaw));  // MJD

        double aimX = x - offsetX;  // MJD
        double aimY = y - offsetY;  // MJD

        double angleToAimPoint = Math.toDegrees(Math.atan2(aimY, aimX));  // MJD

        double robotHeading = driveTrain.getCurrentHeading();  // MJD

        double desiredHeading = robotHeading + angleToAimPoint;  // MJD

        desiredHeading = ((desiredHeading + 540) % 360) - 180;  // MJD

        return desiredHeading;  // MJD
    }

    // AutoAim drives the turret ONLY when override is off.
    public void update() {

        // If driver override OR turret is missing, do nothing
        if (driverOverride || turret == null) {
            return;
        }

        double angle = computeAimAngle();
        if (Double.isNaN(angle)) return;

        turret.setTargetAngle(angle);
    }

}
