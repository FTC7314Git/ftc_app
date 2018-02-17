package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_JewelScoringV2 implements AutonomousOp.StepInterface {

    private Robot robot;
    private boolean lowerJewelArmDoneFlag;
    private boolean raiseJewelArmDoneFlag;
    private boolean displaceJewelDoneFlag;
    private boolean resetJewelWristDoneFlag;
    private Robot.TeamEnum teamColor;
    private Robot.TeamEnum forwardJewelColor;

    private boolean resetMotors_DoneFlag = false;

    private Robot.DirectionEnum robotMovedDirection;

    private int voteRed = 0;
    private int voteBlue = 0;

    private final double SCALE_FACTOR = 255;

    // Constructor, called to create an instance of this class.
    public Step_JewelScoringV2(Robot.TeamEnum teamColor) {

        this.teamColor = teamColor;
    }


    @Override
    public String getLogKey() {
        return "Step_JewelScoring";
    }


    @Override
    public void runStep() {

        if (robot.isStillWaiting()) return;

        lowerJewelArm_runsOnlyOnce();

        raiseControlBlockLift();

        determineJewelColor();

        if (isJewelColorKnown()) {
            displaceJewel_runsOnlyOnce();
        }

        if (isJewelDisplaced()) {
            resetJewelWrist_runsOnlyOnce();
        }

        if (isJewelWristReset()) {
            raiseJewelArm_runsOnlyOnce();
        }
    }

    private void resetEncodersAndStopMotors_Only_Once() {

        if (!resetMotors_DoneFlag) {

            robot.resetEncodersAndStopMotors();
            resetMotors_DoneFlag = true;

            robot.setLoopDelay();
        }

    }

    private void raiseControlBlockLift() {


        int targetBlockLiftPosition = 150;

        robot.motorBlockLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorBlockLift.setTargetPosition(targetBlockLiftPosition);
        robot.motorBlockLift.setPower(.6);

        Log.i(getLogKey(), "raiseControlBlockLift encoder" + robot.motorBlockLift.getCurrentPosition());
    }

    private void logRightEncoder() {

        Log.i(getLogKey(), " RIGHT:" + robot.motorDriveRight.getCurrentPosition());
    }

    private void lowerJewelArm_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (lowerJewelArmDoneFlag == false) {
            robot.servoJewelArm.setPosition(robot.SERVO_JEWEL_ARM_POSITION_DOWN);
            lowerJewelArmDoneFlag = true;
            robot.setTimeDelay(500);
            Log.i(getLogKey(), "lowerJewelArm_runsOnlyOnce Robot at Angle:" + robot.getAngle());
            System.out.println("hi");
        }
    }

    private void raiseJewelArm_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (raiseJewelArmDoneFlag == false) {


            robot.servoJewelArm.setPosition(robot.SERVO_JEWEL_ARM_POSITION_UP);
            raiseJewelArmDoneFlag = true;
            robot.setTimeDelay(500);
            Log.i(getLogKey(), "raiseJewelArm_runsOnlyOnce");
        }
    }
    private void resetJewelWrist_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (resetJewelWristDoneFlag == false) {

            robot.servoJewelWrist.setPosition(robot.SERVO_JEWEL_WRIST_POSITION_MIDDLE);
            resetJewelWristDoneFlag = true;
            robot.setTimeDelay(100);
            Log.i(getLogKey(), "resetJewelWrist_runsOnlyOnce robot.SERVO_JEWEL_WRIST_POSITION_MIDDLE");
        }
    }


    private void displaceJewel_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (displaceJewelDoneFlag == false) {
            displaceJewel();
            displaceJewelDoneFlag = true;
            robot.setTimeDelay(1500);
            Log.i(getLogKey(), "displaceJewel_runsOnlyOnce");
        }
    }


    private void displaceJewel() {

        if (this.teamColor.equals(this.forwardJewelColor)) {
            robot.servoJewelWrist.setPosition(robot.SERVO_JEWEL_WRIST_POSITION_BACKWARD);
            Log.i(getLogKey(), "robot.SERVO_JEWEL_WRIST_POSITION_BACKWARD");
        } else {
            robot.servoJewelWrist.setPosition(robot.SERVO_JEWEL_WRIST_POSITION_FORWRD);
            Log.i(getLogKey(), "robot.SERVO_JEWEL_WRIST_POSITION_FORWRD");
        }

    }

    @Override
    public boolean isStepDone() {
        if (robot.isStillWaiting()) return false;

        if (this.raiseJewelArmDoneFlag) {

            return true;
        }
        return false;
    }

    private void determineJewelColor() {

        if (robot.isStillWaiting()) return;

        if (isJewelColorKnown()) {
            return;
        }

        if (readRedColor() > readBlueColor()) {

            this.voteRed++;
            Log.i(getLogKey(), "read RED");

        }

        if (readBlueColor() > readRedColor()) {

            this.voteBlue++;
            Log.i(getLogKey(), "read BLUE");
        }


    }

    private void analyzeVote() {


        if (voteRed > voteBlue) {
            this.forwardJewelColor = Robot.TeamEnum.RED;
            Log.i(getLogKey(), "analyzeVote(): Red WINS! " + voteRed + " blue:" + voteBlue);
        }
        if (voteBlue > voteRed) {
            this.forwardJewelColor = Robot.TeamEnum.BLUE;
            Log.i(getLogKey(), "analyzeVote(): Blue WINS!" + voteBlue + " red:" + voteRed);
        }
    }

    private boolean isJewelColorKnown() {

        if (Math.abs(this.voteRed - this.voteBlue) >= 10) {
            analyzeVote();
        }

        return this.forwardJewelColor != null;

    }

    private boolean isJewelWristReset() {
        return this.resetJewelWristDoneFlag && !robot.isStillWaiting();
    }


    private boolean isJewelDisplaced() {
        return this.displaceJewelDoneFlag && !robot.isStillWaiting();
    }

    private boolean isJewelArmRaised() {
        return this.raiseJewelArmDoneFlag && !robot.isStillWaiting();
    }

    private int readRedColor() {
        double red = this.robot.colorSensorJewel.red() * SCALE_FACTOR;
        return (int) red;
    }

    private int readBlueColor() {
        double blue = this.robot.colorSensorJewel.blue() * SCALE_FACTOR;
        return (int) blue;
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
