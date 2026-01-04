package org.firstinspires.ftc.teamcode.util;

/**
 * Simple utility class to handle alliance color (RED or BLUE).
 * Usage in OpMode:
 *   AllianceColor alliance = new AllianceColor(AllianceColor.Selection.RED);
 *   if (alliance.isRed()) { ... }
 */
public class AllianceColor {

    public enum Selection {
        RED,
        BLUE
    }

    private final Selection selection;

    /**
     * Constructor
     * @param selection RED or BLUE
     */
    public AllianceColor(Selection selection) {
        this.selection = selection;
    }

    /**
     * @return true if this alliance is RED
     */
    public boolean isRed() {
        return selection == Selection.RED;
    }

    /**
     * @return true if this alliance is BLUE
     */
    public boolean isBlue() {
        return selection == Selection.BLUE;
    }

    /**
     * @return the current Selection (RED or BLUE)
     */
    public Selection getSelection() {
        return selection;
    }

    /**
     * Convenience method to get the opposite alliance
     * @return BLUE if current is RED, RED if current is BLUE
     */
    public AllianceColor getOpposite() {
        return new AllianceColor(selection == Selection.RED ? Selection.BLUE : Selection.RED);
    }
}