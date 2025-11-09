package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Config {
// public DcMotor leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
// public DcMotor rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
// public DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
// public DcMotor shooter = hardwareMap.get(DcMotor.class, "shooter");
// public CRServo leftAdvancer = hardwareMap.get(CRServo.class, "leftAdvancer");
// public CRServo rightAdvancer = hardwareMap.get(CRServo.class, "rightAdvancer");
// public final double FAR_POWER  = 0.75;
// public final double NEAR_POWER = 0.4;
//
//    // Status variables declared once at class leve
// public String shooterStatus = "OFF";
// public String intakeStatus = "OFF";
// public String advancerStatus = "OFF";
public DcMotor leftMotor;
public DcMotor rightMotor;
public DcMotor intakeMotor;
public DcMotorEx shooter;
public CRServo leftAdvancer;
public CRServo rightAdvancer;
public final double FAR_POWER  = 0.75;
public final double NEAR_POWER = 0.4;

public final double Far_Velo = 1750;
public final double Near_Velo = Far_Velo / 2;

public Config(HardwareMap complex_map) {
    leftMotor = complex_map.get(DcMotor.class, "leftMotor");
    rightMotor = complex_map.get(DcMotor.class, "rightMotor");
    intakeMotor = complex_map.get(DcMotor.class, "intakeMotor");
    shooter = complex_map.get(DcMotorEx.class, "shooter");
    leftAdvancer = complex_map.get(CRServo.class, "leftAdvancer");
    rightAdvancer = complex_map.get(CRServo.class, "rightAdvancer");
    leftMotor.setDirection(DcMotor.Direction.FORWARD);
    rightMotor.setDirection(DcMotor.Direction.REVERSE);
    shooter.setDirection(DcMotor.Direction.REVERSE);
    leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


}

}
