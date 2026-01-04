package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static Follower follower = null;
    public static final double closeShootPower = 0;   //Targets 230, really reaches 140
    public static final double farShootPower = 0;
    public static PIDFCoefficients shooterCoefficients = new PIDFCoefficients(300, 0,0,10);

    public static final Pose redCloseShoot = new Pose(76,76,Math.toRadians(135));
    public static final Pose blueCloseShoot = new Pose(68,72, Math.toRadians(-135));
    public static final Pose farRedShoot = new Pose(86, 16, 2.78);
    public static final Pose farBlueShoot = new Pose(58, 18, -2.7);
    public static final Pose redPark = new Pose(35,30, 0);
    public static final Pose bluePark = new Pose(100,30, 0);
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.0);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf_Drive")
            .rightRearMotorName("rb_Drive")
            .leftRearMotorName("lb_Drive")
            .leftFrontMotorName("lf_Drive")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD);





    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rf_Drive")
            .rightRearMotorName("rb_Drive")
            .leftRearMotorName("lb_Drive")
            .leftFrontMotorName("lf_Drive")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotLength(14.25)
            .robotWidth(17.6)
            .forwardTicksToInches(-61.7)
            .strafeTicksToInches(-2.08)
            .turnTicksToInches(-0.02);


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);




    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                .build();
    }
}
