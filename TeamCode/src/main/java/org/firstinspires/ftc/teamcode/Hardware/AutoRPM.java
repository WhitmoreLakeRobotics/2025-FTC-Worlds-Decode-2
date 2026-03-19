package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// Auto calculates RPM for launcher
public class AutoRPM {

    public Telemetry telemetry = null;
    public HardwareMap hardwareMap = null;

    public boolean Measure = false;

    public Limey limey;
    public Launcher launcher;

    private double lastTop = 0;      // MJD
    private double lastBottom = 0;   // MJD

    public AutoRPM(Limey limey, Launcher launcher) {
        this.limey = limey;
        this.launcher = launcher;
    }

    public void init() {}
    public void init_loop() {}
    public void start() {}
    public void stop() {}

    public void loop() {
        update();
    }

    public void update() {

        if (!Measure) return;

        // Null safety — MJD
        if (limey == null || launcher == null) {
            return;
        }

        double tx = limey.getTx();
        double ty = limey.getTy();
        double yaw = limey.getTagAngle();

        double[] rpms = calculateRPMs(tx, ty, yaw);

        launcher.setTargetRPMs(rpms[0], rpms[1]);
    }

    public double[] calculateRPMs(double tx, double ty, double yaw) {

        double distance = limey.getTagDistance();

        // Clamp distance to calibrated range — MJD FIX
        distance = Math.max(0.5, Math.min(distance, 2.9));   // MJD

        // Top motor calibration
        double d1 = 0.5;       // meters
        double r1top = 1900;   // test value

        double d2 = 2.9;       // meters
        double r2top = 3600; // r=range

        double m_top = (r2top - r1top) / (d2 - d1); // m=meters
        double b_top = r1top - m_top * d1; // b=base

        double targetTopRPM = m_top * distance + b_top;

        // Bottom motor calibration
        double r1bottom = 4000;
        double r2bottom = 5500;

        double m_bottom = (r2bottom - r1bottom) / (d2 - d1); // y = mx + b
        double b_bottom = r1bottom - m_bottom * d1;

        double targetBottomRPM = m_bottom * distance + b_bottom;

        // Apply smoothing
        targetTopRPM = smooth(targetTopRPM, lastTop);         // MJD
        targetBottomRPM = smooth(targetBottomRPM, lastBottom); // MJD

        lastTop = targetTopRPM;       // MJD
        lastBottom = targetBottomRPM; // MJD

        return new double[]{targetTopRPM, targetBottomRPM};
    }

    // Low-pass filter
    private double smooth(double current, double last) {
        return 0.7 * last + 0.3 * current;
    }
}
