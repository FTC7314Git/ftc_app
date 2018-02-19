package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_Straight implements AutonomousOp.StepInterface, StepInterface {

    private Integer targetDistanceEncoderCounts = null;
    private Robot.DirectionEnum direction = null;
    private Robot.MotorPowerEnum motorPowerEnum;
    private Robot.RobotStartPositionEnum robotStartPositionEnum;
    private Robot robot;

    private boolean resetMotors_DoneFlag = false;
    private boolean initializedMotors_DoneFlag = false;

    private final int SLOW_FINISH_REMAINING_DISANCE = 500;
    private final int SLOW_START_DISTANCE = 1000;

    private final int DONE_TOLERANCE = 50;


    // Constructor, called to create an instance of this class.
    public Step_Straight(Integer distanceEncoderCounts, Robot.DirectionEnum direction) {

        this.targetDistanceEncoderCounts = distanceEncoderCounts;
        this.direction = direction;
        this.motorPowerEnum = Robot.MotorPowerEnum.Med;
    }


    // Constructor, called to create an instance of this class.
    public Step_Straight(Robot.RobotStartPositionEnum robotStartPositionEnum,
                         Robot.DirectionEnum direction) {

    }


    @Override
    public String getLogKey() {
        return "Step_Straight";
    }


    @Override
    public void runStep() {


        resetEncodersAndStopMotors_Only_Once();

//        logEncoders("run");

        initializeMotors_Only_Once();

        goStraight();


    }

    private DcMotor getEncoderMotor() {

        return robot.motorDriveRight;

    }

    private void goStraight() {

        double motorPower = determineMotorPower();

        Log.i(getLogKey(), "goStraight motorPower: " + motorPower);

        robot.motorDriveLeft.setPower(motorPower);
        robot.motorDriveRight.setPower(motorPower);

    }


    private double determineMotorPower() {


        int remainingDistance = getRemainingDistance();
        int traveledDistance = getTraveledDistance();

        if (traveledDistance < SLOW_START_DISTANCE) {

            return limitMinValue(this.motorPowerEnum.getValue() * traveledDistance / SLOW_START_DISTANCE);

        }

        if (remainingDistance < SLOW_FINISH_REMAINING_DISANCE) {

            return (this.motorPowerEnum.getValue() * remainingDistance / SLOW_FINISH_REMAINING_DISANCE);
        }


        return this.motorPowerEnum.getValue();

    }

    private double limitMinValue(double input) {

        if (input < .05) {

            return .05;
        }

        return input;
    }

    private void resetEncodersAndStopMotors_Only_Once() {

        if (!resetMotors_DoneFlag) {

            Log.i(getLogKey(), "resetEncodersAndStopMotors_Only_Once");

            robot.resetEncodersAndStopMotors();
            resetMotors_DoneFlag = true;

            robot.sleep(1000);
        }

    }


    private void initializeMotors_Only_Once() {


        if (!initializedMotors_DoneFlag) {

            initializeMotorDirection();


            robot.runWithEncoders_MAINTAINS_SPEED();

            robot.sleep(1000);
            initializedMotors_DoneFlag = true;

            Log.i(getLogKey(), "initializeMotors_Only_Once");


        }
    }

    private void initializeMotorDirection() {

        if (Robot.DirectionEnum.FORWARD.equals(this.direction)) {

            robot.setDriveMotorForwardDirection();

        } else {

            robot.setDriveMotorReverseDirection();

        }

    }

    private int getRemainingDistance() {

        DcMotor encoderMotor = getEncoderMotor();

        return Math.abs(targetDistanceEncoderCounts - encoderMotor.getCurrentPosition());
    }

    private int getTraveledDistance() {

        DcMotor encoderMotor = getEncoderMotor();

        return Math.abs(encoderMotor.getCurrentPosition());
    }


    @Override
    public boolean isStepDone() {

        if (robot.isStillWaiting() || !resetMotors_DoneFlag || !initializedMotors_DoneFlag) {
            return false;
        }


        if (isDistanceDone()) {

            logIt("Step is Done:");

            logEncoders(">done");
            robot.resetEncodersAndStopMotors();

            robot.motorDriveRight.setPower(0);
            robot.motorDriveLeft.setPower(0);
            logEncoders("<done");
            return true;
        }

        return false;
    }

    private boolean isDistanceDone() {

        return Math.abs(getRemainingDistance()) <= DONE_TOLERANCE;
    }

    private void logEncoders(String keySuffix) {

        String key = getLogKey() + keySuffix;

        Log.i(key, "--------------------------------------------------------------------------------------");
        Log.i(key, " CurrentPosition: RIGHT:" + robot.motorDriveRight.getCurrentPosition() + " --- LEFT:" + robot.motorDriveLeft.getCurrentPosition());
        Log.i(key, "----------------------");
        Log.i(key, " speed: RIGHT" + robot.motorDriveRight.getPower() + " --- LEFT:" + robot.motorDriveLeft.getPower());
        Log.i(key, "--------------------------------------------------------------------------------------");
    }

    private void logIt(String methodName) {

        StringBuilder sb = new StringBuilder();
        sb.append(methodName);
        sb.append(" CurrentPosition:" + robot.motorDriveLeft.getCurrentPosition());
        sb.append(" Target:" + robot.motorDriveLeft.getTargetPosition());
        sb.append(" Remaining:" + this.getRemainingDistance());
        Log.i(getLogKey(), sb.toString());

    }


    @Override
    public boolean isAborted() {
        return false;
    }


    @Override
    public void setRobot(Robot robot) {
        this.robot = robot;
    }


}
