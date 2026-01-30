//package org.firstinspires.ftc.teamcode.MainOp;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.VoltageSensor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//
//
//public class WithF_ServoMode extends OpMode {
//    // Introducing all the Hardware
//    public DcMotorEx lf, rf, lb, rb, shooter;
//    public CRServo leftAdvancer, rightAdvancer;
//    public Servo blinkin; // Updated from RevBlinkinLedDriver to Servo
//    public DistanceSensor distL, distR;
//    public VoltageSensor voltSensor;
//
//    // Logic States
//    private boolean shooterOn = false, autoFaceOn = false, wasEmpty = false, isAlertingEmpty = false;
//    private double targetVelo = 0, pulseEndTime = 0, pulseDir = 0;
//    private ElapsedTime emptyTimer = new ElapsedTime(), blinkTimer = new ElapsedTime(), pulseTimer = new ElapsedTime();
//    private int blinkCount = 0;
//    private boolean lastRB2 = false, lastLB1 = false, lastX2 = false, lastB2 = false;
//
//    // Constants for all
//    final double EMPTY_CM = 8.0, minD = 53, maxD = 121, minV = 2400, maxV = 2780;
//    private webCamOp camera;
//
//    @Override
//    public void init() {
//        // Initializing Drive Motors
//        lf = hardwareMap.get(DcMotorEx.class, "lf_Drive");
//        rf = hardwareMap.get(DcMotorEx.class, "rf_Drive");
//        lb = hardwareMap.get(DcMotorEx.class, "lb_Drive");
//        rb = hardwareMap.get(DcMotorEx.class, "rb_Drive");
//
//        // Initializing Mechanisms
//        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
//        leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
//        rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
//
//        // Initializing Sensors & Indicators
//        blinkin = hardwareMap.get(Servo.class, "blinkin"); // Mapped as Servo
//        distL = hardwareMap.get(DistanceSensor.class, "distLeft");
//        distR = hardwareMap.get(DistanceSensor.class, "distRight");
//        voltSensor = hardwareMap.voltageSensor.iterator().next();
//
//        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//        camera = new webCamOp(hardwareMap);
//    }
//
//    @Override
//    public void init_loop() {
//        handleBatteryMonitor();
//    }
//
//    @Override
//    public void start() {
//        resetRuntime();
//    }
//
//    @Override
//    public void loop() {
//        camera.update();
//        AprilTagDetection tag = getTargetTag();
//
//        handleBucketCheck();
//        handleDrivetrain(tag);
//        handleShooter(tag);
//        handleAdvancers();
//        updateLEDs(tag);
//
//        telemetry.addData("Match Time", "%.1f", getRuntime());
//        telemetry.update();
//    }
//
//    // ----Battery CHECK (Servo Positions)-----
//    void handleBatteryMonitor() {
//        double v = voltSensor.getVoltage();
//        if (v > 13.5) blinkin.setPosition(0.7745); // Green
//        else if (v > 12.5) blinkin.setPosition(0.6445); // Orange
//        else blinkin.setPosition(0.6145); // Red
//    }
//
//    // --- BUCKET SENSORS ---
//    void handleBucketCheck() {
//        boolean empty = (distL.getDistance(DistanceUnit.CM) > EMPTY_CM && distR.getDistance(DistanceUnit.CM) > EMPTY_CM);
//        if (empty) {
//            if (!wasEmpty) { emptyTimer.reset(); wasEmpty = true; }
//            if (emptyTimer.milliseconds() > 500 && !isAlertingEmpty) {
//                isAlertingEmpty = true;
//                blinkCount = 0;
//                blinkTimer.reset();
//            }
//        } else wasEmpty = false;
//    }
//
//    // --- DRIVING & AUTO-FACE ---
//    void handleDrivetrain(AprilTagDetection tag) {
//        if (gamepad1.left_bumper && !lastLB1) autoFaceOn = !autoFaceOn;
//        lastLB1 = gamepad1.left_bumper;
//
//        double turn = gamepad1.right_stick_x;
//        if (autoFaceOn && tag != null) turn = Range.clip(-tag.ftcPose.bearing * 0.02, -0.4, 0.4);
//
//        double f = gamepad1.left_stick_y, s = -gamepad1.left_stick_x;
//        double d = Math.max(Math.abs(f) + Math.abs(s) + Math.abs(turn), 1);
//        lf.setPower((f + s + turn) / d); rf.setPower((f - s - turn) / d);
//        lb.setPower((f - s + turn) / d); rb.setPower((f + s - turn) / d);
//    }
//
//    // --- SHOOTER ---
//    void handleShooter(AprilTagDetection tag) {
//        if (gamepad2.right_bumper && !lastRB2) shooterOn = !shooterOn;
//        lastRB2 = gamepad2.right_bumper;
//
//        if (tag != null) {
//            targetVelo = Range.clip(((tag.ftcPose.range - minD)/(maxD - minD)) * (maxV - minV) + minV, minV, maxV);
//        } else {
//            targetVelo = maxV;
//        }
//        shooter.setVelocity(shooterOn ? targetVelo : 0);
//    }
//
//    // --- FEEDERS ---
//    void handleAdvancers() {
//        if (gamepad2.x && !lastX2) { pulseDir = 1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
//        else if (gamepad2.b && !lastB2) { pulseDir = -1.0; pulseEndTime = pulseTimer.milliseconds() + 50; }
//        lastX2 = gamepad2.x; lastB2 = gamepad2.b;
//
//        if (pulseTimer.milliseconds() < pulseEndTime) {
//            leftAdvancer.setPower(pulseDir); rightAdvancer.setPower(pulseDir);
//        } else {
//            leftAdvancer.setPower(0); rightAdvancer.setPower(0);
//        }
//    }
//
//    // --- LEDS (Servo Position Control) ---
//    void updateLEDs(AprilTagDetection tag) {
//        // PRIORITY 1: Hardware fail (Hot Pink / Black Strobe)
//        if (Double.isNaN(distL.getDistance(DistanceUnit.CM))) {
//            blinkin.setPosition((System.currentTimeMillis() / 200) % 2 == 0 ? 0.5745 : 0.9995);
//            return;
//        }
//
//        double matchTime = getRuntime();
//
//        // PRIORITY 2: Match timer (Red/White strobe at end)
//        if (matchTime > 115) {
//            blinkin.setPosition((System.currentTimeMillis() / 100) % 2 == 0 ? 0.6145 : 0.3845);
//            return;
//        }
//
//        // PRIORITY 3: Endgame warning (Gold)
//        if (matchTime > 90 && matchTime < 100) {
//            blinkin.setPosition(0.6745);
//            return;
//        }
//
//        // PRIORITY 4: Empty Bucket 5-Blink Alert (Red/Black)
//        if (isAlertingEmpty) {
//            if (blinkCount < 10) {
//                if (blinkTimer.milliseconds() > 50) { blinkCount++; blinkTimer.reset(); }
//                blinkin.setPosition(blinkCount % 2 == 0 ? 0.6145 : 0.9995);
//                return;
//            } else { isAlertingEmpty = false; }
//        }
//
//        // PRIORITY 5: Auto-Face Status (Blue logic)
//        if (autoFaceOn) {
//            if (tag == null) { // Searching (Strobe Blue)
//                blinkin.setPosition((System.currentTimeMillis() / 300) % 2 == 0 ? 0.8745 : 0.9995);
//            } else if (Math.abs(tag.ftcPose.bearing) > 5) { // Partial Lock (Strobe Light Blue)
//                blinkin.setPosition(0.8145);
//            } else { // Locked (Solid Blue)
//                blinkin.setPosition(0.8745);
//            }
//            return;
//        }
//
//        // PRIORITY 6: Range check (Violet strobe)
//        if (tag != null && (tag.ftcPose.range < minD || tag.ftcPose.range > maxD)) {
//            blinkin.setPosition((System.currentTimeMillis() / 250) % 2 == 0 ? 0.9145 : 0.9995);
//            return;
//        }
//
//        // PRIORITY 7: Shooter Ready (Green Pulse / Orange Idle)
//        if (shooterOn && Math.abs(shooter.getVelocity() - targetVelo) < 60) {
//            blinkin.setPosition(0.4745); // Green Pulse/Forest
//        } else {
//            blinkin.setPosition(0.6445); // Team Orange
//        }
//    }
//
//    private AprilTagDetection getTargetTag() {
//        if (camera.seesTag(20)) return camera.getTag(20);
//        if (camera.seesTag(24)) return camera.getTag(24);
//        return null;
//    }
//}