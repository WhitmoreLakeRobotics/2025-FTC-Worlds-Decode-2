package org.firstinspires.ftc.teamcode.Hardware;

import org.firstinspires.ftc.teamcode.Field.DecodeField;

public class DecodeSubsystem extends BaseHardware {

    private Limey limey;
    private DriveTrain driveTrain;

    public int currentTagID = -1;
    public double tagFieldX = Double.NaN;
    public double tagFieldY = Double.NaN;
    public double tagFieldHeadingDegree = Double.NaN;

    public double robotHeadingDeg = 0;

    @Override
    public void init() {
        // nothing needed
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    public void setDependencies(Limey limey, DriveTrain driveTrain) {
        this.limey = limey;
        this.driveTrain = driveTrain;
    }

    @Override
    public void loop() {

        if (limey == null || driveTrain == null) return;

        currentTagID = limey.getTagID();
        robotHeadingDeg = driveTrain.getCurrentHeading();

        if (currentTagID == -1) {
            tagFieldX = Double.NaN;
            tagFieldY = Double.NaN;
            tagFieldHeadingDegree = Double.NaN;
            return;
        }

        tagFieldX = DecodeField.getTAGSx(currentTagID);
        tagFieldY = DecodeField.getTAGSy(currentTagID);
        tagFieldHeadingDegree = DecodeField.getTAGSHeadingdegree(currentTagID);

        telemetry.addData("DECODE Tag ID", currentTagID);
        telemetry.addData("DECODE Tag X", tagFieldX);
        telemetry.addData("DECODE Tag Y", tagFieldY);
        telemetry.addData("DECODE Tag Heading", tagFieldHeadingDegree);
        telemetry.addData("Robot Heading", robotHeadingDeg);
    }

    @Override
    void stop() {

    }
}
