package org.firstinspires.ftc.teamcode.pathing;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.lang.reflect.Field;

/**
 * Everything you may need to edit lives here. The tuner changes the TUNABLE fields live and can
 * save them to a file on the robot controller; {@link #load()} reads that file back. Saved values
 * override the defaults below, so once you are happy, copy them in here and delete the file.
 */
public final class PathingConfig {
    // ---- Hardware (fill these in) ----
    public static String pinpointName = "pinpoint";
    public static String frontLeftName = "leftFrontMotor";
    public static String frontRightName = "rightFrontMotor";
    public static String backLeftName = "leftBackMotor";
    public static String backRightName = "rightBackMotor";

    /** Pod offsets from the robot's tracking point in mm. X pod: left is +. Y pod: forward is +. */
    public static double xPodOffsetMm = 0;
    public static double yPodOffsetMm = 0;
    /** true = goBILDA swingarm pod, false = goBILDA 4-bar pod. */
    public static boolean swingarmPod = false;
    /** Flip if the X (forward) pod counts down when pushing forward / Y pod down when pushing left. */
    public static boolean xPodReversed = false;
    public static boolean yPodReversed = false;

    // ---- Tunables (editable live in the tuner, saved to file) ----
    public static double transP = 0.05;
    public static double transI = 0.0;
    public static double transD = 0.005;
    public static double headP = 1.2;
    public static double headI = 0.0;
    public static double headD = 0.08;
    /** Minimum power added while still outside tolerance, to beat static friction. */
    public static double kStatic = 0.08;
    public static double maxPower = 0.6;
    public static double posTolIn = 0.75;
    public static double headTolDeg = 2.0;
    /** Time inside tolerance before the final waypoint counts as reached. */
    public static double settleSec = 0.15;

    public static final String[] TUNABLE = {
            "transP", "transI", "transD", "headP", "headI", "headD",
            "kStatic", "maxPower", "posTolIn", "headTolDeg", "settleSec"};

    /** Step size per TUNABLE entry for gamepad adjustment. */
    public static final double[] STEP = {
            0.005, 0.0001, 0.0005, 0.05, 0.001, 0.005,
            0.005, 0.05, 0.05, 0.25, 0.05};

    public static final int VIEWER_PORT = 8765;
    private static final String FILE_NAME = "pathing_config.txt";

    private PathingConfig() {}

    public static double get(String name) {
        try {
            return PathingConfig.class.getField(name).getDouble(null);
        } catch (ReflectiveOperationException e) {
            throw new IllegalArgumentException(name, e);
        }
    }

    public static void set(String name, double value) {
        try {
            PathingConfig.class.getField(name).setDouble(null, value);
        } catch (ReflectiveOperationException e) {
            throw new IllegalArgumentException(name, e);
        }
    }

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static void save() {
        StringBuilder sb = new StringBuilder();
        for (String name : TUNABLE) sb.append(name).append('=').append(get(name)).append('\n');
        ReadWriteFile.writeFile(file(), sb.toString());
    }

    /** Applies saved tunables, if a save file exists. Returns true if one was applied. */
    public static boolean load() {
        File f = file();
        if (!f.exists()) return false;
        for (String line : ReadWriteFile.readFile(f).split("\n")) {
            String[] kv = line.trim().split("=");
            if (kv.length != 2) continue;
            try {
                Field field = PathingConfig.class.getField(kv[0]);
                if (field.getType() == double.class) field.setDouble(null, Double.parseDouble(kv[1]));
            } catch (ReflectiveOperationException | NumberFormatException ignored) {
                // Unknown or malformed line: keep the default.
            }
        }
        return true;
    }

    /** Current tunables as text, ready to paste into this file. */
    public static String dump() {
        StringBuilder sb = new StringBuilder();
        for (String name : TUNABLE) sb.append("public static double ").append(name).append(" = ").append(get(name)).append(";\n");
        return sb.toString();
    }
}
