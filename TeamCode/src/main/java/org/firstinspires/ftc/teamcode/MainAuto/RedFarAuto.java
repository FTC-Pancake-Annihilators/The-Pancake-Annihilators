package org.firstinspires.ftc.teamcode.MainAuto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.follower;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.subsystems;
import org.firstinspires.ftc.teamcode.util.AllianceColor;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Autonomous(name = "Red Far Auto", group = "Competition")
public class RedFarAuto extends LinearOpMode {

    subsystems mech;

    @Override
    public void runOpMode() {
        AllianceColor alliance = new AllianceColor(AllianceColor.Selection.RED);
        mech = new subsystems(hardwareMap, alliance);

        follower.setPose(Constants.redStartFar);

        telemetry.addData("Status", "Red Far Auto Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            follower.followPath(mech.farZonePath.get());
            while (opModeIsActive() && follower.isBusy()) sleep(20);

            mech.cycle(null);
            while (opModeIsActive() && mech.cycleState != subsystems.CycleState.IDLE) sleep(20);

            follower.followPath(mech.parkPath.get());
            while (opModeIsActive() && follower.isBusy()) sleep(20);
        }

        telemetry.addData("Auto", "Complete");
        telemetry.update();
    }
}