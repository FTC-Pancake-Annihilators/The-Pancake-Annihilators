package org.firstinspires.ftc.teamcode.auto;

public class AutoRoutes {

    // ===== ROUTE 1 =====
    static void route1(Drive drive) {
        intake(drive);
        shoot(drive);

        intake(drive);
        shoot(drive);
    }

    // ===== ROUTE 2 =====
    static void route2(Drive drive) {
        intake(drive);
        intake(drive);
        shoot(drive);
        shoot(drive);
    }

    // ===== ROUTE 3 =====
    static void route3(Drive drive) {
        intake(drive);
        shoot(drive);

        intake(drive);
        shoot(drive);
    }

    // ===== ROUTE 4 =====
    static void route4(Drive drive) {
        intake(drive);
        intake(drive);
        shoot(drive);
    }

    // ===== BASIC ACTIONS =====
    private static void intake(Drive drive) {
        // drive.followPath(intakePath);
        // intakeMotor.setPower(1);
        sleep(700);
        // intakeMotor.setPower(0);
    }

    private static void shoot(Drive drive) {
        // shooterMotor.setVelocity(3000);
        sleep(500);
        // feederServo.setPosition(1);
        sleep(300);
        // feederServo.setPosition(0);
    }

    private static void sleep(long ms) {
        try { Thread.sleep(ms); }
        catch (InterruptedException ignored) {}
    }
}
