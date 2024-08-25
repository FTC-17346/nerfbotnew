package org.firstinspires.ftc.teamcode.NERFBot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="NERF Bot Auto ODOM", group="NERFBot")
public class NERFBotOpticalOdometry extends LinearOpMode {
    NERFBotHardware robot;

    SparkFunOTOS myOtos;

    //@Override
    public void runOpMode(){
        robot = new NERFBotHardware(this);

        robot.init();
        robot.myOtos.resetTracking();
        robot.myOtos.calibrateImu();
        waitForStart();
        while (opModeIsActive()) {
            LinearMovement(1, 0, 0, .5);
        }



    }


    public void LinearMovement(double Xpos, double Ypos, double Heading, double Speed){
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();


//
        while ( pos.x < Xpos ) {
            robot.backLeft.setPower(-Speed);
            robot.frontLeft.setPower(Speed);
            robot.frontRight.setPower(-Speed);
            robot.backRight.setPower(Speed);
        }
        while ( pos.y < Ypos ) {
            robot.backLeft.setPower(Speed);
            robot.frontLeft.setPower(Speed);
            robot.frontRight.setPower(Speed);
            robot.backRight.setPower(Speed);
        }
        while ( pos.h < Heading ) {
            robot.backLeft.setPower(-Speed);
            robot.frontLeft.setPower(-Speed);
            robot.frontRight.setPower(Speed);
            robot.backRight.setPower(Speed);
        }

        robot.backLeft.setPower(0);
        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);

    }
}