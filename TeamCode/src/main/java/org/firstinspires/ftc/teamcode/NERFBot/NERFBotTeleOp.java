package org.firstinspires.ftc.teamcode.NERFBot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "NERF Bot TeleOp", group = "NERFBot")
public class NERFBotTeleOp extends OpMode {
    NERFBotHardware robot;

    private ElapsedTime runtime = new ElapsedTime();

    //@Override
    public void init() {
        robot = new NERFBotHardware(this);
        robot.init();
    }

    @Override
    public void loop() {

        // Driving code
        if (gamepad1.back) {
            robot.resetHeading();
        }
        double botHeading = robot.getHeading();
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        // Rotate the movement direction with the bot rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        robot.drive(rotY, rotX, gamepad1.right_stick_x);

        // Turret pitch code
        double pitchControlValue = -gamepad2.left_stick_y * 0.2;
        robot.pitchControl.addToPosition(pitchControlValue);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Yaw", "yaw: " + robot.yawTurret.getPosition());
        telemetry.addData("Pitch", "pitch: " + robot.pitchControl.currentPosition);
        telemetry.addData("Linear Position", robot.reload.getCurrentPosition());
        telemetry.update();
    }
}
