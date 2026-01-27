//package org.firstinspires.ftc.teamcode.pedroPathing;
//
//import com.pedropathing.control.FilteredPIDFCoefficients;
//import com.pedropathing.control.PIDFCoefficients;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.follower.FollowerConstants;
//import com.pedropathing.ftc.FollowerBuilder;
//import com.pedropathing.ftc.drivetrains.MecanumConstants;
//import com.pedropathing.ftc.localization.Encoder;
//import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathConstraints;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//public class Constants {
//    public static Follower follower = null;
//
//    public static PIDFCoefficients shooterCoefficients = new PIDFCoefficients(300, 0,0,10);
//
//
//    // Starting poses
//    public static final Pose redStartNear = new Pose(87, 8, Math.toRadians(90));
//    public static final Pose blueStartNear = new Pose(55, 8, Math.toRadians(90));
//    public static final Pose redStartFar = new Pose(119, 128, Math.toRadians(45));
//    public static final Pose blueStartFar = new Pose(24.5, 128, Math.toRadians(135));
//    public static final Pose redCloseShoot = new Pose(87, 87, Math.toRadians(45));
//    public static final Pose blueCloseShoot = new Pose(55, 87, Math.toRadians(135));
//
//    public static final Pose farRedShoot = new Pose(60, 12, Math.toRadians(111));
//    public static final Pose farBlueShoot = new Pose(130, 132, Math.toRadians(69));
//
//    // Parking
//    public static final Pose redPark = new Pose(38, 33, Math.toRadians(90));
//    public static final Pose bluePark = new Pose(104, 33, Math.toRadians(90));
//
//    // Gate ram poses (back of robot hits gate)
//    public static final Pose redGateApproach = new Pose(129, 69, Math.toRadians(180));  // Face backward
//    public static final Pose blueGateApproach = new Pose(15, 69, Math.toRadians(0));
//    public static final double midFieldVelocity = 220.0;
//    public static final double farVelocity = 280.0;
//    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(7)
//            .useSecondaryTranslationalPIDF(true)
//            .useSecondaryHeadingPIDF(true)
//            .useSecondaryDrivePIDF(true);
//    //            .forwardZeroPowerAcceleration(deceleration)
////            .lateralZeroPowerAcceleration(deceleration);
//    public static MecanumConstants driveConstants = new MecanumConstants()
//            .maxPower(1)
//            .rightFrontMotorName("rf_Drive")
//            .rightRearMotorName("rb_Drive")
//            .leftRearMotorName("lb_Drive")
//            .leftFrontMotorName("lf_Drive")
//            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
//            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
//            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
//            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);
////            .xVelocity()
////            .yVelocity();
//
//
//
//
//
//
//    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
//            .rightFrontMotorName("rf_Drive")
//            .rightRearMotorName("rb_Drive")
//            .leftRearMotorName("lb_Drive")
//            .leftFrontMotorName("lf_Drive")
//            .leftFrontEncoderDirection(Encoder.FORWARD)
//            .leftRearEncoderDirection(Encoder.FORWARD)
//            .rightFrontEncoderDirection(Encoder.FORWARD)
//            .rightRearEncoderDirection(Encoder.FORWARD)
//            .robotLength(14.25)
//            .robotWidth(17.6);
////            .forwardTicksToInches(-61.7)
////            .strafeTicksToInches(-2.08)
////            .turnTicksToInches(-0.02);
//// DO the localization Test
//// Run the Tuning OpMode, then navigate under to Localization Test
////On your computer, connect to your robot's Wi-Fi, and navigate to Panels or the FTC Dashboard. Panels is accessible at the ip address 192.168.43.1:8001 when connected to robot wifi.
////You should see the robot's position on the field.
////Observe the movements, make sure moving forward increases x and strafing left increases y.
//
//    public static PathConstraints pathConstraints = new PathConstraints(
//            0.99, 100, 1, 1);
//
//
//
//
//    public static Follower createFollower(HardwareMap hardwareMap) {
//        return new FollowerBuilder(followerConstants, hardwareMap)
//                .pathConstraints(pathConstraints)
//                .mecanumDrivetrain(driveConstants)
//                .driveEncoderLocalizer(localizerConstants)
//                .build();
//    }
//}
