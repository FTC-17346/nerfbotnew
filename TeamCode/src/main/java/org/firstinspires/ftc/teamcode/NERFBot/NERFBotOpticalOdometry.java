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
        SparkFunOTOS.Pose2D pos;
        while (opModeIsActive()) {
            pos = robot.myOtos.getPosition();
            LinearMovement(5, 0, 0, .2);
            telemetry.addLine("X Position :" +pos.x);
            telemetry.addLine("Y Position :" +pos.y);
            telemetry.update();
        }



    }


    public void LinearMovement(double Xpos, double Ypos, double Heading, double Speed){
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        if ( pos.x < Xpos -.3) {
            robot.backLeft.setPower(-Speed);
            robot.frontLeft.setPower(Speed);
            robot.frontRight.setPower(-Speed);
            robot.backRight.setPower(Speed);
        }
        else if ( pos.x > Xpos +.3){
            robot.backLeft.setPower(Speed);
            robot.frontLeft.setPower(-Speed);
            robot.frontRight.setPower(Speed);
            robot.backRight.setPower(-Speed);
        }
        else if ( pos.y < Ypos -.3) {
            robot.backLeft.setPower(Speed);
            robot.frontLeft.setPower(Speed);
            robot.frontRight.setPower(Speed);
            robot.backRight.setPower(Speed);
        }
        else if ( pos.y > Ypos +.3){
            robot.backLeft.setPower(-Speed);
            robot.frontLeft.setPower(-Speed);
            robot.frontRight.setPower(-Speed);
            robot.backRight.setPower(-Speed);
        }
        else if ( pos.h < Heading -.3) {
            robot.backLeft.setPower(-Speed);
            robot.frontLeft.setPower(-Speed);
            robot.frontRight.setPower(Speed);
            robot.backRight.setPower(Speed);
        }
        else if ( pos.h > Heading +.3) {
            robot.backLeft.setPower(Speed);
            robot.frontLeft.setPower(Speed);
            robot.frontRight.setPower(-Speed);
            robot.backRight.setPower(-Speed);
        }
        else {
            robot.backLeft.setPower(0);
            robot.frontLeft.setPower(0);
            robot.frontRight.setPower(0);
            robot.backRight.setPower(0);
        }

    }
}