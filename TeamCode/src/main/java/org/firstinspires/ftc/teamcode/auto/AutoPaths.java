package org.firstinspires.ftc.teamcode.auto;



public class AutoPaths {

    public static Pose2d mirror(Pose2d pose, AutoConfig.Alliance alliance) {
        if (alliance == AutoConfig.Alliance.BLUE) return pose;

        return new Pose2d(
                pose.x,
                -pose.y,
                pose.heading
        );
    }


    public static Pose2d startLeft(AutoConfig.Alliance alliance) {
        return mirror(new Pose2d(12, 62, Math.toRadians(90)), alliance);
    }

    public static Pose2d startRight(AutoConfig.Alliance alliance) {
        return mirror(new Pose2d(-12, 62, Math.toRadians(90)), alliance);
    }
}
