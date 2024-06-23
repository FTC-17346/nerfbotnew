package org.firstinspires.ftc.teamcode.NERFBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="NERF Bot TeleOp", group="NERFBot")
public class NERFBotTeleOp extends OpMode {
    NERFBotHardware robot;

    private ElapsedTime runtime = new ElapsedTime();

    private static double StandardYaw = 0.8;

    //@Override
    public void init() {
         robot = new NERFBotHardware(this);
         robot.init();
    }


    boolean xWasPressed = false;
    @Override
    public void loop() {

        robot.yawTurret.scaleRange(-.5, .5);
        robot.yawTurret.scaleRange(-.5, .5);

        if (gamepad1.back) {
            robot.imu.resetYaw();
        }

        double botHeading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double xTurret = gamepad2.right_stick_x;
        // Rotate the movement direction with the bot rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);


        double max;
        double yaw = gamepad1.right_stick_x;
        double pitchTurret = ((gamepad2.left_stick_y + 1)/2)/1000;

        double leftFrontPower = rotY + rotX + yaw;
        double rightFrontPower = rotY - rotX - yaw;
        double leftBackPower = rotY - rotX + yaw;
        double rightBackPower = rotY + rotX - yaw;
        double YawPosition = - yaw + xTurret;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        //max = Math.max(max, 1.5); // remove this line to speed up robot
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
        ////     leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
        //     rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
        //     rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
        robot.frontLeft.setPower(leftFrontPower);
        robot.frontRight.setPower(rightFrontPower);
        robot.backLeft.setPower(leftBackPower);
        robot.backRight.setPower(rightBackPower);
        //robot.yawTurret.setPosition(YawPosition);
        robot.pitchTurret.setPosition((robot.pitchTurret.getPosition() + pitchTurret));
               // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", YawPosition, pitchTurret);
        telemetry.addData("Yaw", "yaw: " + robot.yawTurret.getPosition());
        telemetry.addData("Pitch", "pitch: " + robot.pitchTurret.getPosition());
        telemetry.addData("Linear Position", robot.reload.getCurrentPosition());
        telemetry.update();
    }
}
