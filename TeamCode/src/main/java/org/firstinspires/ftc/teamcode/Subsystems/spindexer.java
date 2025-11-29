package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@SuppressWarnings("FieldCanBeLocal") // Suppress pointless Android Studio warnings
public class spindexer {
    private final Servo spindexerServo;
    private final ElapsedTime movementTimer = new ElapsedTime();
    private double currentPos = 0.0;
    private double targetPos = 0.0;

    // Positions for each spindexer state (these values are all need to be tuned)
    private final double INTAKE_1_POS = 0; // Also the home position
    private final double INTAKE_2_POS = 0.1;
    private final double INTAKE_3_POS = 0.2;
    private final double LAUNCH_1_POS = -0.1;
    private final double LAUNCH_2_POS = -0.2;
    private final double LAUNCH_3_POS = -0.3;

    // We can't directly get the position of a servo so we're just gonna calculate it with time (this has to be tuned too)
    private final int FULL_ROTATION_TIME = 300; // Time (in ms) it takes for a full rotation of the spindexer from -1 to 1

    public spindexer(HardwareMap hardwareMap) {
        spindexerServo = hardwareMap.get(Servo.class, "spindexer");
    }

    /**
     * Move the spindexer to a specific position
     * This should not be called by your OpModes, you should use the specific moveTo methods instead
     *
     * @param position The target position to move to (-1.0 to 1.0)
     */
    private void moveToPosition(double position) {
        movementTimer.reset(); // Starting a new move, reset the timer
        targetPos = position; // Set the target position
        spindexerServo.setPosition(position); // Command the servo to the target position
    }

    /**
     * Move the spindexer to intake an artifact into the first bay
     */
    public void moveToIntake1()  {
        moveToPosition(INTAKE_1_POS);
    }

    /**
     * Move the spindexer to intake an artifact into the second bay
     */
    public void moveToIntake2() {
        moveToPosition(INTAKE_2_POS);
    }

    /**
     * Move the spindexer to intake an artifact into the third bay
     */
    public void moveToIntake3() {
        moveToPosition(INTAKE_3_POS);
    }

    /**
     * Move the spindexer to launch an artifact from the first bay
     */
    public void moveToLaunch1() {
        moveToPosition(LAUNCH_1_POS);
    }

    /**
     * Move the spindexer to launch an artifact from the second bay
     */
    public void moveToLaunch2() {
        moveToPosition(LAUNCH_2_POS);
    }

    /**
     * Move the spindexer to launch an artifact from the third bay
     */
    public void moveToLaunch3() {
        moveToPosition(LAUNCH_3_POS);
    }

    /**
     * Check if the spindexer is in the target position
     */
    public boolean atTargetPos() {
        return currentPos == targetPos;
    }

    /**
     * Update the indexer position based on the elapsed time
     * You should call this every loop in your OpModes
     */
    public void update() {
        // Update the current position based on the elapsed time
        double elapsedTime = movementTimer.milliseconds();
        double positionChange = (elapsedTime / FULL_ROTATION_TIME) * (targetPos - currentPos);
        currentPos += positionChange;

        // Clamp the current position to the target position
        if ((targetPos - currentPos) * positionChange < 0) {
            currentPos = targetPos;
        }
    }
}