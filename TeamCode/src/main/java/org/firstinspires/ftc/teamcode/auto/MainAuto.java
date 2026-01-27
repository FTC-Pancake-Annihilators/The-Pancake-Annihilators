package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Main Auto", group = "Auto")
public class MainAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Drive drive = new Drive(hardwareMap);

        AutoConfig config = new AutoConfig();
        AutoSelector selector = new AutoSelector();

        while (!isStarted() && !isStopRequested()) {
            selector.update(gamepad1, telemetry, config);
        }

        waitForStart();

        switch (config.route) {
            case ROUTE1:
                AutoRoutes.route1(drive);
                break;
            case ROUTE2:
                AutoRoutes.route2(drive);
                break;
            case ROUTE3:
                AutoRoutes.route3(drive);
                break;
            case ROUTE4:
                AutoRoutes.route4(drive);
                break;
        }
    }
}
