
package org.firstinspires.ftc.teamcode.sabbotage.relic.manual;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

@TeleOp(name = "ManualRobot", group = "Concept")
public class ManualControlOp extends OpMode {
    private static final String KEY = "Manual";

    private static final double SERVO_PADDLE_RIGHT_OPEN = 0.2;
    private static final double SERVO_PADDLE_RIGHT_RELEASE = 0.55;
    private static final double SERVO_PADDLE_RIGHT_CLOSE = 0.75;

    private static final double SERVO_PADDLE_LEFT_OPEN = 0.1;
    private static final double SERVO_PADDLE_LEFT_RELEASE = 0.35;
    private static final double SERVO_PADDLE_LEFT_CLOSE = 0.55;



    private static final int FLOOR = 150;
    private static final int FIRST_FLOOR = 0;
    private static final int SECOND_FLOOR = FIRST_FLOOR + FLOOR + 30;
    private static final int THIRD_FLOOR = SECOND_FLOOR + FLOOR;
    private static final int FOURTH_FLOOR = THIRD_FLOOR + FLOOR;


    boolean resetBlockLiftDoneFlag;
    private int targetBlockLiftPosition = 0;

    private Robot robot;

    private int loopCounter = 0;


    public ManualControlOp() {
    }

    @Override
    public void init() {

        this.robot = new Robot(hardwareMap, telemetry);

        init_ResetBlockLift_onlyRunsOnce();
//        init_InitEncoders();
    }

    private void init_InitEncoders() {
        robot.motorBlockLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void init_ResetBlockLift_onlyRunsOnce() {

        if (robot.isStillWaiting()) return;

        if (resetBlockLiftDoneFlag == false) {
            robot.motorBlockLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            resetBlockLiftDoneFlag = true;
            robot.setTimeDelay(1000);
            Log.i(KEY, "init_ResetBlockLift_onlyRunsOnce()");

        }
    }


    /*
     * This method will be called repeatedly in a loop
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#run()
     */
    @Override
    public void loop() {

        loopCounter = loopCounter + 1;

        driver_controlDriveMotors();
        operator_controlPaddles();
        operator_controlBlockLift();
    }

    private void operator_controlBlockLift() {

        determineTargetBlockLiftPosition();


        robot.motorBlockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBlockLift.setTargetPosition(this.targetBlockLiftPosition);
        robot.motorBlockLift.setPower(.6);

        Log.i(KEY, "encoder" + robot.motorBlockLift.getCurrentPosition());
    }

    private void determineTargetBlockLiftPosition() {

        if (gamepad2.dpad_down) {
            targetBlockLiftPosition = FIRST_FLOOR;
        } else if (gamepad2.dpad_left) {
            targetBlockLiftPosition = SECOND_FLOOR;
        } else if (gamepad2.dpad_up) {
            targetBlockLiftPosition = THIRD_FLOOR;
        } else if (gamepad2.dpad_right) {
            targetBlockLiftPosition = FOURTH_FLOOR;
        }

    }

    private void operator_controlPaddles() {

        Log.i(KEY, "Right Servo: " + robot.servoRightPaddle.getPosition());

        if (gamepad2.x) {
            robot.servoLeftPaddle.setPosition(SERVO_PADDLE_LEFT_RELEASE);
            robot.servoRightPaddle.setPosition(SERVO_PADDLE_RIGHT_RELEASE);
        }

        if (gamepad2.b) {
            robot.servoLeftPaddle.setPosition(SERVO_PADDLE_LEFT_CLOSE);
            robot.servoRightPaddle.setPosition(SERVO_PADDLE_RIGHT_CLOSE);
        }

        if (gamepad2.y) {
            robot.servoLeftPaddle.setPosition(SERVO_PADDLE_LEFT_OPEN);
            robot.servoRightPaddle.setPosition(SERVO_PADDLE_RIGHT_OPEN);
        }

    }


//    private int getRemainingDistance() {
//
//        return Math.abs(distanceEncoderCounts - robot.motorBlockLift.getCurrentPosition());
//    }

    private void driver_controlDriveMotors() {

        robot.motorDriveRight.setPower(scaleOutput(-gamepad1.right_stick_y));
        robot.motorDriveLeft.setPower(scaleOutput(-gamepad1.left_stick_y));

//        Log.i(KEY,"gamepad1.right_stick_y" + -gamepad1.right_stick_y);
//        Log.i(KEY, "WHEELS: [" + robot.motorDriveRight.getDirection()+ ": " +
//                String.format("%.0f", robot.motorDriveRight.getPower() * 100) + "]---[" +
//                robot.motorDriveRight.getDirection()+ ": " +String.format("%.0f", robot.motorDriveLeft.getPower() * 100) + "]");

    }

    private float conditionsPower(float input) {

        if( Math.abs(input) < .5) {
            return input / 5;
        }
        return input;
    }



    private float limitValue(float input) {

        if (input > 1.0) {

            return 1.0f;
        } else if (input < -1.0) {
            return -1.0f;
        }
        return input;
    }


    /*
     * Code to run when the op mode is first disabled goes here
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#stop()
     */
    @Override
    public void stop() {

        telemetry.addData("TextStop", "***Stop happened**" + loopCounter);

    }

    /*
     * This method scales the joystick inputValue so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleOutput(float inputValue) {

        double[] scaleArray = {.0, .05, .1, .15, .2, .25, .25, .25, .25, .25, .25, .3, .3, .35, .4, 1 , 1 };
        // get the corresponding index for the scaleOutput array.
        int index = (int) (inputValue * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (inputValue < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

}
