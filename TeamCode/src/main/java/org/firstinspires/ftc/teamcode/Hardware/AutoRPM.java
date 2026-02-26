package org.firstinspires.ftc.teamcode.Hardware;

import android.icu.util.Measure;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
//Auto calculates RPM for launcher

public class AutoRPM {

    public Telemetry telemetry = null;

    public HardwareMap hardwareMap = null;

    public boolean Measure = false;

    public Limey limey;
    public Launcher launcher;

    public void init() {

    }

    public void init_loop() {

    }

    public void start() {

    }

    public void loop() {

        /*
        if (Measure == true){
            double tx = limey.getTx();
            double ty = limey.getTy();
            double yaw = limey.getTagAngle();

            double[] rpms = calculateRPMs(tx, ty, yaw);

            launcher.setTargetRPMs(rpms[0], rpms[1]);
        }
        */

        update();

    }

    public void stop() {

    }

    public AutoRPM(Limey limey, Launcher launcher) {
        this.limey = limey;
        this.launcher = launcher;
    }

    public void update() {

        if (Measure == true) {

            // Null safety
            if (limey == null || launcher == null) {
                return;
            }

            double tx = limey.getTx();
            double ty = limey.getTy();
            double yaw = limey.getTagAngle();

            double[] rpms = calculateRPMs(tx, ty, yaw);

            launcher.setTargetRPMs(rpms[0], rpms[1]);
        }
    }

    public double[] calculateRPMs(double tx, double ty, double yaw) {

        double distance = limey.getTagDistance();

        // Top motor
        double d1 = 0.5;    //in meters
        double r1top = 1900;    //need to update test

        double d2 = 2.9;    //in meters
        double r2top = 3600;

        double m_top = (r2top - r1top) / (d2 - d1);
        double b_top = r1top - m_top * d1;

        double targetTopRPM = m_top * distance + b_top;

        // bottom motor
        // double d1b = 18;
        double r1bottom = 4000;

        // double d2b = 180;
        double r2bottom = 5500;

        double m_bottom = (r2bottom - r1bottom) / (d2 - d1);
        double b_bottom = r1bottom - m_bottom * d1;

        double targetBottomRPM = m_bottom * distance + b_bottom;

        return new double[]{targetTopRPM, targetBottomRPM};
    }
}
