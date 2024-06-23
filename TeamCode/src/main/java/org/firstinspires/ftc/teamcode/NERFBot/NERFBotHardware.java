package org.firstinspires.ftc.teamcode.NERFBot;

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
        pitchTurret = hwMap.get(ServoImplEx.class, "trigger");
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

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
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

    public YawControl yawControl = this.new YawControl();
}