
package org.firstinspires.ftc.teamcode.sabbotage.relic.manual;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

@TeleOp(name = "ManualRobot", group = "Concept")
public class ManualControlOp extends OpMode {
    private static final String KEY = "Manual";

    private static final int FLOOR = 150;
    private static final int FIRST_FLOOR = 0;
    private static final int SECOND_FLOOR = FIRST_FLOOR + FLOOR + 60;
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


//        test_servo();
        driver_controlSideways();
        driver_controlDriveMotors_Tank();
        operator_controlPaddles();
        operator_controlBlockLift();
    }

    private void test_servo() {

        Log.i(KEY+ "test", "test_servo : " + gamepad2.left_stick_y);

            robot.servoJewelArm.setPosition(gamepad2.left_stick_y);
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

        if (gamepad2.x) {
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_RELEASE);
            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_RELEASE);
        }

        if (gamepad2.b) {
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_CLOSE);
            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
        }

        if (gamepad2.y) {
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_OPEN);
            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_OPEN);
        }

    }


//    private int getRemainingDistance() {
//
//        return Math.abs(distanceEncoderCounts - robot.motorBlockLift.getCurrentPosition());
//    }

    private void driver_controlDriveMotors_Tank() {

        robot.motorDriveRight.setPower(scaleOutput(-gamepad1.right_stick_y));
        robot.motorDriveLeft.setPower(scaleOutput(-gamepad1.left_stick_y));

    }

    private void driver_controlDriveMotors_Arcade() {

        double forward = scaleOutput(-gamepad1.right_stick_y);
        double turn = scaleOutput(-gamepad1.right_stick_x) / 2;

        double right = forward + turn;
        double left = forward - turn;

        robot.motorDriveRight.setPower((double) limitValue((float) right));
        robot.motorDriveLeft.setPower((double) limitValue((float) left));

    }

    private void driver_controlSideways() {

        if (gamepad1.a) {
            robot.motorRobotLift.setPower(-.5);
        } else if (gamepad1.y) {
            robot.motorRobotLift.setPower(.5);
        } else {
            robot.motorRobotLift.setPower(0);
        }

        Log.i(KEY, "x: " + gamepad1.left_stick_x);


        if (gamepad1.left_bumper) {
            Log.i(KEY, "left_bumper Sideways" );
            robot.motorRobotSideways.setPower(-.5);
        } else if (gamepad1.right_bumper) {
            Log.i(KEY, "right_bumper Sideways");
            robot.motorRobotSideways.setPower(.5);

        } else {
            Log.i(KEY, "cleared Sideways");
            robot.motorRobotSideways.setPower(0);
        }
    }


    private float conditionsPower(float input) {

        if (Math.abs(input) < .5) {
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

        double[] scaleArray = {.0, .05, .1, .15, .2, .25, .25, .25, .25, .25, .25, .3, .3, .35, .4, 1, 1};
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
