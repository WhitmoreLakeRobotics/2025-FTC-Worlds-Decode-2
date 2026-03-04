package org.firstinspires.ftc.teamcode.Field;

import java.util.Map;

public class DecodeField {

    // Simple pose container
    public static class TagPose {
        public final double x;          // meters
        public final double y;          // meters
        public final double z;          // meters (height)
        public final double headingDeg; // yaw in degrees

        public TagPose(double x, double y, double z, double headingDeg) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.headingDeg = headingDeg;
        }
    }

    // Tag IDs
    public static final int TAG_20 = 20;   // Blue
    public static final int TAG_24 = 24;   // Red

    /*
       AprilTag positions for FTC DECODE (2025–2026)
       Coordinate system: FTC standard center-origin
       +X = toward Red Alliance
       +Y = toward audience
       Units: meters
       Source: Measured field coordinates published by FTC teams
    */

    private static final Map<Integer, TagPose> TAG_POSES = Map.of(
            TAG_20, new TagPose(
                    -1.482,   // x
                    -1.413,   // y
                    0.749,    // z
                    54        // heading
            ),
            TAG_24, new TagPose(
                    -1.482,
                    +1.413,
                    0.749,
                    -54
            )
    );

    // Lookup
    public static TagPose getTagPose(int id) {
        return TAG_POSES.get(id);
    }
}
