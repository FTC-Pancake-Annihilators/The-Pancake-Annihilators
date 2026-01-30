//package org.firstinspires.ftc.teamcode.MainAuto;
//
//import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.follower;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.teamcode.Mechanisms.subsystems;
//import org.firstinspires.ftc.teamcode.util.AllianceColor;
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//@Autonomous(name = "RedNear-PP", group = "Competition")
//public class RedNearAuto extends LinearOpMode {
//
//    subsystems mech;
//
//    @Override
//    public void runOpMode() {
//        AllianceColor alliance = new AllianceColor(AllianceColor.Selection.RED);
//        mech = new subsystems(hardwareMap, alliance);
//
//        // Set starting pose
//        follower.setPose(Constants.redStartNear);
//
//        telemetry.addData("Status", "Red Near Auto Ready");
//        telemetry.update();
//
//        waitForStart();
//
//        if (opModeIsActive()) {
//            // Drive to mid-field (close) shooting zone
//            follower.followPath(mech.nearZonePath.get());
//            while (opModeIsActive() && follower.isBusy()) {
//                sleep(20);
//            }
//
//            // Shoot 3 artifacts
//            mech.cycle(null);  // Start cycle
//            while (opModeIsActive() && mech.cycleState != subsystems.CycleState.IDLE) {
//                sleep(20);
//            }
//
//            // Park
//            follower.followPath(mech.parkPath.get());
//            while (opModeIsActive() && follower.isBusy()) {
//                sleep(20);
//            }
//        }
//
//        telemetry.addData("Auto", "Complete - Parked");
//        telemetry.update();
//    }
//}