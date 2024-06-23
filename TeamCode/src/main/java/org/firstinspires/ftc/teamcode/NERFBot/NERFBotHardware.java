package org.firstinspires.ftc.teamcode.NERFBot;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class NERFBotHardware {
    private OpMode myOpMode;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public ServoImplEx yawTurret;
    public ServoImplEx pitchTurret;

    public DcMotorEx reload;

    public IMU imu;

    private HardwareMap hwMap;
    public NERFBotHardware(OpMode opmode) {
        myOpMode = opmode;
        hwMap = myOpMode.hardwareMap;
    }

    public void init()    {
        frontLeft = hwMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hwMap.get(DcMotorEx.class, "frontRight");
        backLeft = hwMap.get(DcMotorEx.class, "backLeft");
        backRight = hwMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        yawTurret = hwMap.get(ServoImplEx.class, "yawTurret");
        pitchTurret = hwMap.get(ServoImplEx.class, "pitchTurret");

        reload = hwMap.get(DcMotorEx.class, "reload");
        reload.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        reload.setDirection(DcMotorEx.Direction.REVERSE);
        reload.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        imu = hwMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    double max;

    public void drive(double forward, double turn, double yaw) {
        double leftFrontPower  = forward + turn + yaw;
        double rightFrontPower = forward - turn  - yaw;
        double leftBackPower  = forward - turn + yaw;
        double rightBackPower = forward + turn  - yaw;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


        if (max > 1.0)
        {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, rightFrontPower,leftBackPower, rightBackPower);
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower,double leftBackPower, double rightBackPower) {
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }
}