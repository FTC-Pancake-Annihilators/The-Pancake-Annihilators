package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6)
            .forwardZeroPowerAcceleration(-25.9346931313679598)
            .lateralZeroPowerAcceleration(-67.342491844080064)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .translationalPIDFSwitch(4)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))

            .headingPIDFCoefficients(new PIDFCoefficients(
                    0,
                    0,
                    0,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    2,
                    0,
                    0,
                    0
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0,
                    0,
                    0,
                    0,
                    0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0,
                    0,
                    0,
                    0,
                    0
            ))
            .drivePIDFSwitch(15)
            .centripetalScaling(0.0005);
    

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf_Drive")
            .rightRearMotorName("rb_Drive")
            .leftRearMotorName("lb_Drive")
            .leftFrontMotorName("lf_Drive")
            .leftFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(78.261926752421046666666666666667)
            .yVelocity(61.494551922189565);



    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rf_Drive")
            .rightRearMotorName("rb_Drive")
            .leftRearMotorName("lb_Drive")
            .leftFrontMotorName("lf_Drive")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);
//            .forwardTicksToInches(multiplier)
//            .strafeTicksToInches(multiplier)
//            .turnTicksToInches(multiplier);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.009,
            50,
            1.25,
            10,
            1
    );


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                // Use one of the localizers below:
                .driveEncoderLocalizer(localizerConstants)

                .build();
    }
}
