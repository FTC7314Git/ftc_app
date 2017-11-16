package org.firstinspires.ftc.teamcode.sabbotage.relic.robot;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public class Robot {


    private HardwareMap hardwareMap;
    public int HARDWARE_DELAY = 30;

    public static final double SERVO_JEWEL_ARM_POSITION_DOWN = .75;
    public static final double SERVO_JEWEL_ARM_POSITION_UP = 0;


    public static final double SERVO_PADDLE_LEFT_OPEN = 0.2;
    public static final double SERVO_PADDLE_LEFT_RELEASE = 0.45;
    public static final double SERVO_PADDLE_LEFT_CLOSE = 0.65;

    public static final double SERVO_PADDLE_RIGHT_OPEN = 0.1;
    public static final double SERVO_PADDLE_RIGHT_RELEASE = 0.35;
    public static final double SERVO_PADDLE_RIGHT_CLOSE = 0.6;



    private Robot.TeamEnum jewelColor;

    public int loopCounter;

    private long delayUntilTimeMilliSeconds;

    private int delayUntilLoopCount;


    public Telemetry telemetry;

    public DcMotor motorDriveRight;
    public DcMotor motorDriveLeft;
    public DcMotor motorBlockLift;


    public DcMotor motorRobotLift;
    public DcMotor motorRobotSideways;


    public Servo servoLeftPaddle;
    public Servo servoRightPaddle;
    public Servo servoJewelArm;

    public ColorSensor colorSensorJewel;
    public DistanceSensor distanceSensorJewel;


    public ModernRoboticsI2cGyro gyroSensor;
    private RelicRecoveryVuMark vuMark = null;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        if (this.hardwareMap == null) {

            Log.i("ROBOT", "OOF!!!! hardwareMap is null");

        }


        this.motorDriveLeft = this.hardwareMap.dcMotor.get("motorDriveLeft");
        this.motorDriveRight = this.hardwareMap.dcMotor.get("motorDriveRight");
        this.motorBlockLift = this.hardwareMap.dcMotor.get("motorBlockLift");

        this.motorRobotLift = this.hardwareMap.dcMotor.get("motorRobotLift");
        this.motorRobotSideways = this.hardwareMap.dcMotor.get("motorRobotSideways");

        this.servoLeftPaddle = hardwareMap.servo.get("servoLeftPaddle");
        this.servoRightPaddle = hardwareMap.servo.get("servoRightPaddle");

        this.servoJewelArm = hardwareMap.servo.get("servoJewelArm");

        this.gyroSensor = this.hardwareMap.get(ModernRoboticsI2cGyro.class, "gyroSensor");

        this.colorSensorJewel = hardwareMap.get(ColorSensor.class, "colorDistanceSensor");
        this.distanceSensorJewel = hardwareMap.get(DistanceSensor.class, "colorDistanceSensor");

        resetHardwarePositions();
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


    }


    private void resetHardwarePositions() {

        resetDriveMotors();
        resetServos();

    }


    public void resetServos() {

        this.servoRightPaddle.setDirection(Servo.Direction.FORWARD);
        this.servoLeftPaddle.setDirection(Servo.Direction.REVERSE);
        this.servoJewelArm.setDirection(Servo.Direction.FORWARD);


        this.servoJewelArm.setPosition(0.0);
//        this.servoRightPaddle.setPosition(0.0);
//        this.servoLeftPaddle.setPosition(0.1);
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
            Log.i("ROBOT", "Waiting..." + loopCounter + "  Waiting..mS." + System.currentTimeMillis());
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

    public RelicRecoveryVuMark getVuMark() {
        return this.vuMark;
    }


    public TeamEnum getJewelColor() {
        return jewelColor;
    }

    public void setJewelColor(TeamEnum jewelColor) {
        this.jewelColor = jewelColor;
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


