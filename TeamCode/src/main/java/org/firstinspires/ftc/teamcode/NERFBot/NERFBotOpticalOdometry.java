package org.firstinspires.ftc.teamcode.NERFBot;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="NERF Bot GPT OTOS", group="NERFBot")
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
            LinearMovement(5, 0, 0, .6);
            telemetry.addLine("X Position :" +pos.x);
            telemetry.addLine("Y Position :" +pos.y);
            telemetry.update();
        }



    }


    public void LinearMovement(double destX, double destY, double destH, double maxSpeed) {
        SparkFunOTOS.Pose2D pos = robot.myOtos.getPosition();
        // Current position and heading (obtained from the sensor)
        double currentX = pos.x;
        double currentY = pos.y;
        double currentH = pos.h;

        // Destination position and heading

        // Speed variables
        double minSpeed = 0.1;
        double currentSpeed = 0;

        // Threshold for deceleration (distance from destination)
        double decelerationThreshold = 5.0; // Adjust as needed

        // Method to calculate the distance to the destination
        double calculateDistance = Math.sqrt(Math.pow(destX - currentX, 2) + Math.pow(destY - currentY, 2));


        // Method to move the robot towards the destination
            double distance = calculateDistance;

            while (distance > 0.1) { // Stop when close enough to the destination
                distance = calculateDistance;
                if (distance > decelerationThreshold) {
                    // Accelerate
                    currentSpeed = Math.min(currentSpeed + 0.05, maxSpeed);
                } else {
                    // Decelerate
                    currentSpeed = Math.max(minSpeed, maxSpeed * (distance / decelerationThreshold));
                }

                // Calculate direction towards the destination
                double angleToDest = Math.atan2(destY - currentY, destX - currentX);

                // Update current position based on speed and direction
                currentX += currentSpeed * Math.cos(angleToDest);
                currentY += currentSpeed * Math.sin(angleToDest);

                // Update heading to smoothly rotate towards the destination heading
                currentH += (destH - currentH) * 0.1; // Adjust rotation speed as needed

                // Print out current status for debugging
                System.out.printf("Current Position: (%.2f, %.2f) | Current Heading: %.2f | Current Speed: %.2f\n",
                        currentX, currentY, currentH, currentSpeed);

                // Add a small delay to simulate real-time movement
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }


            // Stop the robot once it reaches the destination
            currentSpeed = 0;
            System.out.println("Destination reached!");
        }

    }
}