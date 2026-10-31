package org.firstinspires.ftc.teamcode.pathing;

/**
 * The field grid. Origin is the field center, +x is the Pinpoint's forward axis, +y is left,
 * heading is CCW positive. Everything is stored in inches; {@link #unit} only controls how the
 * numbers you type into {@link #at} (and the numbers shown back to you) are interpreted.
 */
public final class Grid {
    public enum Unit {
        INCHES(1.0), TILES(24.0);

        public final double inches;

        Unit(double inches) {
            this.inches = inches;
        }
    }

    public static final double FIELD_SIZE_IN = 144.0;
    public static final double TILE_IN = 24.0;

    /** Active authoring unit. Switch to TILES to write waypoints like Grid.at(1.5, -2, 90). */
    public static Unit unit = Unit.INCHES;

    private Grid() {}

    /** Pose from x/y in the active unit and heading in degrees. */
    public static Pose at(double x, double y, double headingDeg) {
        return new Pose(x * unit.inches, y * unit.inches, Math.toRadians(headingDeg));
    }

    /**
     * Center of a field tile. col/row are 0..5, (0,0) is the tile in the -x/-y corner,
     * col increases along +x and row along +y.
     */
    public static Pose tile(int col, int row, double headingDeg) {
        double half = FIELD_SIZE_IN / 2.0;
        return new Pose(-half + TILE_IN / 2 + col * TILE_IN,
                -half + TILE_IN / 2 + row * TILE_IN,
                Math.toRadians(headingDeg));
    }

    /** Converts inches to the active unit, for display. */
    public static double toUnit(double inches) {
        return inches / unit.inches;
    }

    public static void toggleUnit() {
        unit = (unit == Unit.INCHES) ? Unit.TILES : Unit.INCHES;
    }
}
