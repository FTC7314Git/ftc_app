package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_JewelScoring implements AutonomousOp.StepInterface {

    private Robot robot;
    private boolean lowerJewelArmDoneFlag;
    private boolean raiseJewelArmDoneFlag;
    private boolean displaceJewelDoneFlag;
    private boolean returnRobotToStartPositionDoneFlag;
    private Robot.TeamEnum teamColor;
    private Robot.TeamEnum forwardJewelColor;


    private Robot.DirectionEnum robotMovedDirection;

    private int voteRed = 0;
    private int voteBlue = 0;

    private final double SCALE_FACTOR = 255;

    // Constructor, called to create an instance of this class.
    public Step_JewelScoring(Robot.TeamEnum teamColor) {

        this.teamColor = teamColor;
    }


    @Override
    public String getLogKey() {
        return "Step_JewelScoring";
    }


    @Override
    public void runStep() {

        robot.motorDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (robot.isStillWaiting()) return;

        lowerJewelArm_runsOnlyOnce();

        determineJewelColor();

        if (isJewelColorKnown()) {
            displaceJewel_runsOnlyOnce();
        }

        if (isJewelDisplaced()) {
            raiseJewelArm_runsOnlyOnce();
        }

        if (isJewelArmRaised()) {
            returnRobotToStartPosition_runsOnlyOnce();
        }

    }

    private void logEncoders() {

        Log.i(getLogKey(), " RIGHT:" + robot.motorDriveRight.getCurrentPosition() + " --- LEFT:" + robot.motorDriveLeft.getCurrentPosition());
    }

    private void lowerJewelArm_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (lowerJewelArmDoneFlag == false) {
            robot.servoJewelArm.setPosition(robot.SERVO_JEWEL_ARM_POSITION_DOWN);
            lowerJewelArmDoneFlag = true;
            robot.setTimeDelay(500);
            Log.i(getLogKey(), "lowerJewelArm_runsOnlyOnce Robot at Angle:" + robot.getAngle());

        }
    }

    private void raiseJewelArm_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (raiseJewelArmDoneFlag == false) {

            stopRobot();

            robot.servoJewelArm.setPosition(robot.SERVO_JEWEL_ARM_POSITION_UP);
            raiseJewelArmDoneFlag = true;
            robot.setTimeDelay(500);
            Log.i(getLogKey(), "raiseJewelArm_runsOnlyOnce");
        }
    }

    private void stopRobot() {

        robot.motorDriveRight.setPower(0);
        robot.motorDriveLeft.setPower(0);

    }

    private void displaceJewel_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;


        if (displaceJewelDoneFlag == false) {
            driveRobotToDisplaceJewel();
            displaceJewelDoneFlag = true;
            robot.setTimeDelay(1000);
            Log.i(getLogKey(), "displaceJewel_runsOnlyOnce");
        }
    }


    private void returnRobotToStartPosition_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;


        if (returnRobotToStartPositionDoneFlag == false) {
            driveRobotReturnToStartPosition();
            returnRobotToStartPositionDoneFlag = true;
            robot.setTimeDelay(500);
            Log.i(getLogKey(), "returnRobotToStartPosition_runsOnlyOnce");
        }
    }

    private void driveRobotToDisplaceJewel() {

        double power = .3;

        if (this.teamColor.equals(this.forwardJewelColor)) {
            robot.motorDriveRight.setTargetPosition(-300);
            robot.motorDriveLeft.setTargetPosition(0);
            robot.motorDriveRight.setPower(power);
            robot.motorDriveLeft.setPower(power);
            this.robotMovedDirection = Robot.DirectionEnum.REVERSE;
            Log.i(getLogKey(), "driveRobotToDisplaceJewel REVERSE");

        } else {
            robot.motorDriveRight.setTargetPosition(+300);
            robot.motorDriveLeft.setTargetPosition(0);
            robot.motorDriveRight.setPower(power);
            robot.motorDriveLeft.setPower(power);
            this.robotMovedDirection = Robot.DirectionEnum.FORWARD;
            Log.i(getLogKey(), "driveRobotToDisplaceJewel forward");
        }
    }


    private void driveRobotReturnToStartPosition() {

        double power = .3;

        if (Robot.DirectionEnum.FORWARD.equals(this.robotMovedDirection)) {

            robot.motorDriveRight.setTargetPosition(0);
            robot.motorDriveLeft.setTargetPosition(0);
            robot.motorDriveRight.setPower(power);
            robot.motorDriveLeft.setPower(power);;
            Log.i(getLogKey(), "driveRobotReturnToStartPosition Backward");

        } else {
            robot.motorDriveRight.setTargetPosition(0);
            robot.motorDriveLeft.setTargetPosition(0);
            robot.motorDriveRight.setPower(power);
            robot.motorDriveLeft.setPower(power);;
            Log.i(getLogKey(), "driveRobotReturnToStartPosition forward");
        }
    }





    @Override
    public boolean isStepDone() {
        if (robot.isStillWaiting()) return false;

        logEncoders();
        if (this.returnRobotToStartPositionDoneFlag) {
            Log.i(getLogKey(), "Step is Done at angle:" + robot.getAngle());
            robot.resetEncodersAndStopMotors();

            return true;
        }
        return false;
    }

    private void determineJewelColor() {

        if (robot.isStillWaiting()) return;

        if (isJewelColorKnown()) {
            return;
        }

        if ( readRedColor() >  readBlueColor()){

            this.voteRed++;
            Log.i(getLogKey(), "read RED");

        }

        if ( readBlueColor() >  readRedColor()){

            this.voteBlue++;
            Log.i(getLogKey(), "read BLUE");

        }


    }

    private void analyzeVote() {


        if (voteRed > voteBlue) {
            this.forwardJewelColor = Robot.TeamEnum.RED;
            Log.i(getLogKey(), "analyzeVote(): Red WINS! " +voteRed + " blue:" + voteBlue );
        }
        if (voteBlue > voteRed) {
            this.forwardJewelColor = Robot.TeamEnum.BLUE;
            Log.i(getLogKey(), "analyzeVote(): Blue WINS!" +voteBlue + " red:" + voteRed );
        }
    }

    private boolean isJewelColorKnown() {

        if (Math.abs(this.voteRed - this.voteBlue) >= 10) {
            analyzeVote();
        }

        return this.forwardJewelColor != null;

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
