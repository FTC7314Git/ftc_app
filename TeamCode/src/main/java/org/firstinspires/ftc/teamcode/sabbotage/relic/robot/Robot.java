package org.firstinspires.ftc.teamcode.sabbotage.relic.robot;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class Robot {


    private HardwareMap hardwareMap;
    public int HARDWARE_DELAY = 30;

    public static final double SERVO_JEWEL_ARM_POSITION_DOWN = .75;
    public static final double SERVO_JEWEL_ARM_POSITION_UP = 0;


    public static final double SERVO_JEWEL_WRIST_POSITION_FORWARD = .9;
    public static final double SERVO_JEWEL_WRIST_POSITION_MIDDLE = .53;
    public static final double SERVO_JEWEL_WRIST_POSITION_BACKWARD = .2;




    public static final double SERVO_PADDLE_LEFT_CLOSE = 0.0;
    public static final double SERVO_PADDLE_LEFT_RELEASE = 0.45;
    public static final double SERVO_PADDLE_LEFT_OPEN = 0.65;

    public static final double SERVO_PADDLE_RIGHT_CLOSE = 0;
    public static final double SERVO_PADDLE_RIGHT_RELEASE = 0.35;
    public static final double SERVO_PADDLE_RIGHT_OPEN = 0.6;

    public static final double SERVO_RELIC_WRIST_UP = 0;
    public static final double SERVO_RELIC_WRIST_DOWN = 0.7;

    public static final double SERVO_RELIC_GRAB_OPEN = 0;
    public static final double SERVO_RELIC_GRAB_CLOSE = 0.7;


    public static final double SERVO_BLOCK_PUSH_OUT = 0;
    public static final double SERVO_BLOCK_PUSH_IN = 0.7;

    private Robot.TeamEnum jewelColor;

    public int loopCounter;

    private long delayUntilTimeMilliSeconds;

    private int delayUntilLoopCount;


    public Telemetry telemetry;

    public DcMotor motorDriveRight;
    public DcMotor motorDriveLeft;
    public DcMotor motorBlockLift;

    public DcMotor motorRobotLift;
    public DcMotor motorExtendRelicArm;

    public Servo servoLeftPaddle;
    public Servo servoRightPaddle;
    public Servo servoJewelArm;
    public Servo servoJewelWrist;

    public Servo servoRelicWrist;
    public Servo servoRelicGrab;

    public Servo servoBlockPush;

    public ColorSensor colorSensorJewel;
    public DistanceSensor distanceSensorJewel;

    public BNO055IMU imu;
    public Orientation angles;

    private RelicRecoveryVuMark vuMark = null;


    public Robot() {

    }


    public void init(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        if (this.hardwareMap == null) {

            Log.i("ROBOT", "OOF!!!! hardwareMap is null");

        }

        this.motorDriveLeft = this.hardwareMap.dcMotor.get("motorDriveLeft");
        this.motorDriveRight = this.hardwareMap.dcMotor.get("motorDriveRight");
        this.motorBlockLift = this.hardwareMap.dcMotor.get("motorBlockLift");

        this.motorRobotLift = this.hardwareMap.dcMotor.get("motorRobotLift");


        this.motorExtendRelicArm = this.hardwareMap.dcMotor.get("motorExtendRelicArm");

        this.servoLeftPaddle = hardwareMap.servo.get("servoLeftPaddle");
        this.servoRightPaddle = hardwareMap.servo.get("servoRightPaddle");

        this.servoJewelArm = hardwareMap.servo.get("servoJewelArm");
        this.servoJewelWrist = hardwareMap.servo.get("servoJewelWrist");


        this.servoRelicWrist = hardwareMap.servo.get("servoRelicWrist");
        this.servoRelicGrab = hardwareMap.servo.get("servoRelicGrab");



        this.servoBlockPush = hardwareMap.servo.get("servoBlockPush");

        this.colorSensorJewel = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        this.distanceSensorJewel = hardwareMap.get(DistanceSensor.class, "colorDistanceSensor");

        resetHardwarePositions();
        setupIMU();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public void resetDriveMotors() {

        setDriveMotorForwardDirection();
        runWithoutEncoders();
    }


    public void resetEncodersAndStopMotors() {

        Log.w("ROBOT", "resetEncodersAndStopMotors..." + loopCounter);
        this.motorDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorDriveLeft.setPower(0);
        this.motorDriveRight.setPower(0);

    }


    private void resetHardwarePositions() {

        resetDriveMotors();
        resetServos();

    }


    public void resetServos() {

        this.servoRightPaddle.setDirection(Servo.Direction.FORWARD);
        this.servoLeftPaddle.setDirection(Servo.Direction.REVERSE);
        this.servoJewelArm.setDirection(Servo.Direction.REVERSE);
        this.servoJewelWrist.setDirection(Servo.Direction.REVERSE);


        this.servoRelicWrist.setDirection(Servo.Direction.FORWARD);
        this.servoRelicGrab.setDirection(Servo.Direction.FORWARD);


        this.servoJewelArm.setPosition(0.0);
        this.servoJewelWrist.setPosition(SERVO_JEWEL_WRIST_POSITION_MIDDLE);
    }

    public void setDriveMotorForwardDirection() {

        this.motorDriveLeft.setDirection(DcMotor.Direction.REVERSE);
        this.motorDriveRight.setDirection(DcMotor.Direction.FORWARD);
        this.motorBlockLift.setDirection(DcMotor.Direction.REVERSE);


    }

    public void setDriveMotorReverseDirection() {

        this.motorDriveLeft.setDirection(DcMotor.Direction.FORWARD);
        this.motorDriveRight.setDirection(DcMotor.Direction.REVERSE);


    }

    public void runWithoutEncoders() {

        this.motorDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motorDriveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    public void runWithEncoders_MAINTAINS_SPEED() {

        this.motorDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public boolean isStillWaiting() {

        if (delayUntilLoopCount > loopCounter || delayUntilTimeMilliSeconds > System.currentTimeMillis()) {
            Log.i("ROBOT", "Waiting..." + (delayUntilLoopCount - loopCounter)
                    + "  Waiting..mS." + (delayUntilTimeMilliSeconds - System.currentTimeMillis()));
            return true;
        }
        return false;
    }

    public void setLoopDelay() {

        this.delayUntilLoopCount = loopCounter + HARDWARE_DELAY;
    }

    public void setTimeDelay(long delayMilliSeconds) {

        this.delayUntilTimeMilliSeconds = System.currentTimeMillis() + delayMilliSeconds;
    }

    public void setVuMark(RelicRecoveryVuMark vuMark) {
        this.vuMark = vuMark;
    }

    public void setupIMU() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public RelicRecoveryVuMark getVuMark() {
        return this.vuMark;
    }


    public TeamEnum getJewelColor() {
        return jewelColor;
    }

    public void setJewelColor(TeamEnum jewelColor) {
        this.jewelColor = jewelColor;
    }

    public float getAngle() {
        if (angles.firstAngle < 0) {
            return angles.firstAngle + 360F;
        }
        return angles.firstAngle;
    }

    public float getAngleOffZero() {
        return angles.firstAngle;
    }

    public static enum ColorEnum {

        RED,

        BLUE,

        WHITE


    }

    public enum TurnEnum {

        RIGHT,

        LEFT
    }


    public enum TeamEnum {
        RED,

        BLUE
    }

    public enum RobotStartPositionEnum {

        RED_RIGHT_SIDE,

        RED_LEFT_SIDE,

        BLUE_RIGHT_SIDE,

        BLUE_LEFT_SIDE,

    }

    public enum StrafeEnum {

        RIGHT,

        LEFT
    }


    public enum DirectionEnum {

        FORWARD,

        REVERSE
    }

    public enum PaddlePosition {

        OPEN,

        CLOSE
    }

    public enum RobotLiftPositionEnum {

        UP,

        DOWN

    }

    public enum MotorPowerEnum {

        LowLow(0.1),

        Low(0.2),

        Med(0.4),

        High(0.6),

        FTL(0.8);

        private double motorPower;

        private MotorPowerEnum(double motorPower) {
            this.motorPower = motorPower;

        }

        public double getValue() {
            return this.motorPower;

        }

    }

}


