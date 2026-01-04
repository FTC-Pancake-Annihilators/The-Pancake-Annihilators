package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

public class Transfer {
    public CRServo leftAdvancer, rightAdvancer;

    private double pulseEndTime = 0;
    private double pulseDirection = 0;

    private boolean lastX2 = false;
    private boolean lastB2 = false;

    private ElapsedTime pulseTimer = new ElapsedTime();

    public Transfer(HardwareMap hwMap) {
        leftAdvancer = hwMap.get(CRServo.class, "leftAdvancer");
        rightAdvancer = hwMap.get(CRServo.class, "rightAdvancer");
    }

    // Manual timed pulse control (kept from your original code - useful for driver testing)
    public void handleTimedAdvancers() {
        // Detect button press (click) for X (forward pulse)
        if (gamepad2.x && !lastX2) {
            pulseDirection = 1.0;
            pulseEndTime = pulseTimer.milliseconds() + 50; // 50ms pulse
        }
        // Detect button press (click) for B (reverse pulse)
        else if (gamepad2.b && !lastB2) {
            pulseDirection = -1.0;
            pulseEndTime = pulseTimer.milliseconds() + 50;
        }

        lastX2 = gamepad2.x;
        lastB2 = gamepad2.b;

        // Run the pulse if active
        if (pulseTimer.milliseconds() < pulseEndTime) {
            leftAdvancer.setPower(pulseDirection);
            rightAdvancer.setPower(pulseDirection);
        } else {
            leftAdvancer.setPower(0);
            rightAdvancer.setPower(0);
            pulseDirection = 0;
        }
    }

    // New: Hold / ready position (stops feeding - safe default)
    public void reload() {
        leftAdvancer.setPower(0);
        rightAdvancer.setPower(0);
    }

    // New: Feed one artifact forward (short controlled pulse)
    public void feed() {
        leftAdvancer.setPower(1.0);
        rightAdvancer.setPower(1.0);
    }

    // Optional: Stop explicitly (same as reload)
    public void stop() {
        reload();
    }
}