package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_CryptColumn implements AutonomousOp.StepInterface {

    private Robot robot;
    private boolean lowerJewelArmDoneFlag;
    private boolean raiseJewelArmDoneFlag;
    private boolean displaceJewelDoneFlag;
    private boolean returnRobotToStartPositionDoneFlag;
    private Robot.TeamEnum teamColor;
    private String currentPosition;


    private Robot.DirectionEnum robotMovedDirection;

    private int voteColumn = 0;
    private int voteClear = 0;

    private final double SCALE_FACTOR = 255;

    // Constructor, called to create an instance of this class.
    public Step_CryptColumn(Robot.TeamEnum teamColor) {

        this.teamColor = teamColor;


    }


    @Override
    public String getLogKey() {
        return "Step_CryptColumn";
    }


    @Override
    public void runStep() {

        // lower sensor arm.
        // drive left
        // isRobotAtColumn?, then update currentColumnPosition
        // hasRobotClearedColumn? ( has last readings been false? ), then enable isRobotAtColumn
        // isRobotAtTargetColumn?, then finish.
        //

        robot.motorRobotSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.motorRobotSideways.setPower(.5);

        determineAtCryptColumn();

        if (isAtColumn()) {
            Log.i(getLogKey(), "ROBOT AT COLUMN");
        }

    }

    private void logEncoders() {

        Log.i(getLogKey(), " RIGHT:" + robot.motorDriveRight.getCurrentPosition() + " --- LEFT:" + robot.motorDriveLeft.getCurrentPosition());
    }

    private void stopRobot() {

        robot.motorRobotSideways.setPower(0);

    }

    @Override
    public boolean isStepDone() {
        if (robot.isStillWaiting()) return false;

        logEncoders();
        if (!isAtColumn()) {
            return false;
        }
        Log.i(getLogKey(), "Step is Done:");
        stopRobot();
        return true;
    }

    private void determineAtCryptColumn() {

        if (robot.isStillWaiting()) return;

        if (isAtColumn()) {
            return;
        }

        if (readRedColor() > 10 || readBlueColor() > 10){

            this.voteColumn++;
            Log.i(getLogKey(), "read COLUMN");

        } else {

            this.voteClear++;
            Log.i(getLogKey(), "read CLEAR");

        }


    }

    private void analyzeVote() {


        if (voteColumn > voteClear) {
            this.currentPosition = "column";
            Log.i(getLogKey(), "analyzeVote(): COLUMN WINS! " + voteColumn + " Clear:" + voteClear);
        }
        if (voteClear > voteColumn) {
            this.currentPosition = "clear";
            Log.i(getLogKey(), "analyzeVote(): CLEAR WINS!" + voteClear + " Column:" + voteColumn);
        }
    }

    private boolean isAtColumn() {

        if (Math.abs(this.voteColumn - this.voteClear) >= 10) {
            analyzeVote();
        }

        return this.currentPosition != null;

    }

    private int readRedColor() {
        double red = this.robot.colorSensorColumn.red() * SCALE_FACTOR;
        Log.i(getLogKey(), "COLOR:" + red + " | DISTANCE:" + this.robot.distanceSensorColumn.getDistance(DistanceUnit.CM));
        return (int) red;
    }

    private int readBlueColor() {
        double blue = this.robot.colorSensorColumn.blue() * SCALE_FACTOR;
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
