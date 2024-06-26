package org.firstinspires.ftc.teamcode.NERFBot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="NERF Bot Autonomous", group="NERFBot")
public class NERFBotAutonomous extends LinearOpMode {
    NERFBotHardware robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new NERFBotHardware(this);
        robot.init();
        waitForStart();
        double[][] driveInstructions = new double[][] {
                // Format: Milliseconds, drive, turn
                {500,  0.5, 0},
                {1000, 0,   0.5},
                {1000, 0,   1},
                {1000, 0,   -1},
                {500, -0.5, -1},
        };
        for (int i = 0; i < driveInstructions.length && opModeIsActive(); i++) {
            double[] instruction = driveInstructions[i];
            //robot.drive(instruction[1], instruction[2]);
            sleep((long) instruction[0]);
        }
    }
}