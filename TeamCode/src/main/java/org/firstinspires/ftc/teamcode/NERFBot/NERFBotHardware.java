package org.firstinspires.ftc.teamcode.NERFBot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class NERFBotHardware {
    private OpMode myOpMode;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public ServoImplEx yawTurret;
    private ServoImplEx pitchTurret;
    private ServoImplEx trigger;

    public DcMotorEx reload;

    public IMU imu;

    SparkFunOTOS myOtos;
    private ElapsedTime runtime = new ElapsedTime();

    private HardwareMap hwMap;

    public NERFBotHardware(OpMode opmode) {
        myOpMode = opmode;
        hwMap = myOpMode.hardwareMap;
    }

    public void init() {
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
        trigger = hwMap.get(ServoImplEx.class, "trigger");

        pitchTurret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        yawTurret.setPwmRange(new PwmControl.PwmRange(500, 2500));
        trigger.setPwmRange(new PwmControl.PwmRange(500, 2500));

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

        myOtos = hwMap.get(SparkFunOTOS.class, "sensor_otos");

        configureOtos();
    }


    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public double getHeadingDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void drive(double forward, double turn, double yaw) {
        double leftFrontPower = forward + turn + yaw;
        double rightFrontPower = forward - turn - yaw;
        double leftBackPower = forward - turn + yaw;
        double rightBackPower = forward + turn - yaw;
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));


        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    public void setDrivePower(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }


    public enum TriggerState {
        OPEN,
        FIRING
    }

    public class TriggerControl {
        public TriggerState state = TriggerState.OPEN;
        public static final double FIRING_MS = 250;
        public ElapsedTime firingTime = new ElapsedTime();

        private void opening() {
            trigger.setPwmDisable();
        }

        private void firing() {
            trigger.setPwmEnable();
            trigger.setPosition(0.7);
        }

        public void fire() {
            if (state == TriggerState.FIRING) {
                if (firingTime.milliseconds() <= FIRING_MS) {
                    firing();
                } else {
                    opening();
                }
            } else {
                state = TriggerState.FIRING;
                firingTime.reset();
            }
        }

        public void open() {
            state = TriggerState.OPEN;
        }

        public void fireIf(boolean button) {
            if (button) fire();
            else open();
        }
    }

    public TriggerControl triggerControl = this.new TriggerControl();

    public class PitchControl {
        public static final double DEFAULT_POSITION = 0.4;
        public double currentPosition = DEFAULT_POSITION;

        public void setPosition(double pos) {
            currentPosition = pos;
            pitchTurret.setPosition(currentPosition);
        }

        private double last_update = -1;

        public void addToPosition(double positionChangePerSecond) {
            double elapsedTime = runtime.seconds() - last_update;
            last_update = runtime.seconds();
            if (elapsedTime > 0.2) elapsedTime = 0.01; // limit rapid changes
            double position = currentPosition + positionChangePerSecond * elapsedTime;
            position = Range.clip(position, .27, .54);
            setPosition(position);
        }
    }

    public PitchControl pitchControl = this.new PitchControl();

    public class YawControl {
        public static final double FULLY_STRAIGHT_POSITION = 0.48;

        public double currentPosition = FULLY_STRAIGHT_POSITION;


        public void setPosition(double pos) {
            currentPosition = pos;
            double adjustedPositionDegrees = (currentPosition * 300 +
                    getHeadingDegrees());
            adjustedPositionDegrees %= 360;
            if (adjustedPositionDegrees >= 0 && adjustedPositionDegrees <= 300) {
                // we are in a valid range so just set the position
                double adjustedPosition = adjustedPositionDegrees / 300;
                yawTurret.setPosition(adjustedPosition);
            } else {
                // set to the closer position
                // 300-330: 1 is closer
                // 330-360: 0 is closer
                // we could also just not set the position here if we prefer that
                if (adjustedPositionDegrees > 300 && adjustedPositionDegrees <= 330) {
                    yawTurret.setPosition(1);
                } else {
                    yawTurret.setPosition(0);
                }
            }
        }

        private double last_update = -1;

        public void addToPosition(double positionChangePerSecond) {
            double elapsedTime = runtime.seconds() - last_update;
            last_update = runtime.seconds();
            if (elapsedTime > 0.2) elapsedTime = 0.01; // limit rapid changes
            double position = currentPosition + positionChangePerSecond * elapsedTime;
            position = Range.clip(position, 0, 1);
            setPosition(position);
        }

    }
    private void configureOtos() {
//        telemetry.addLine("Configuring OTOS...");
//        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setPosition(currentPosition);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);

//        telemetry.addLine("OTOS configured! Press start to get position data!");
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
//        telemetry.update();
    }






    public YawControl yawControl = this.new YawControl();
}