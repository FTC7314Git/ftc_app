package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;


public class Step_CryptColumn implements AutonomousOp.StepInterface {

    private Robot robot;
    private boolean lowerJewelArmDoneFlag;
    private boolean raiseJewelArmDoneFlag;
    private boolean displaceJewelDoneFlag;
    private boolean returnRobotToStartPositionDoneFlag;
    private Robot.TeamEnum teamColor;

    private RelicRecoveryVuMark currentPositionEnum;
    private RelicRecoveryVuMark targetPositionEnum;

    private boolean isWaitingForColumnClear;

    private int columnCount;


    private int voteColumn = 0;
    private int voteClear = 0;

    private final double SCALE_FACTOR = 255;
    private final int BLUE_COLOR_VALUE = 20000;
    private final int RED_COLOR_VALUE = 20000;


    // Constructor, called to create an instance of this class.
    public Step_CryptColumn() {

    }


    @Override
    public String getLogKey() {
        return "Step_CryptColumn";
    }


    @Override
    public void runStep() {

        // lower sensor arm.
        // sideways drive left
        // isRobotAtColumn?, then update currentColumnPosition
        // hasRobotClearedColumn? ( has last readings been false? ), then enable isRobotAtColumn
        // isRobotAtTargetColumn?, then finish.
        //

        logDebugInfo();
        setTargetColumnPosition_runsOnlyOnce();

        lowerColumnSensorArm();

        robot.motorRobotSideways.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorRobotSideways.setPower(-Robot.MotorPowerEnum.Low.getValue());

        readColorSensorAndVotePosition();

        if (isAtColumn()) {
            atColumnEvaluateRobotLocation();
        } else if (isAtClear()) {

            this.isWaitingForColumnClear = false;
        }


    }

    private void logDebugInfo() {


        StringBuilder sb = new StringBuilder();

        if (isWaitingForColumnClear) {
            sb.append(" Waiting!!! ForColumnClear:" + this.isWaitingForColumnClear);
        }
        sb.append(" currentPositionEnum:" + this.currentPositionEnum);
        sb.append(" voteColumn:" + this.voteColumn);
        sb.append(" voteClear:" + this.voteClear);
        Log.i(getLogKey(), sb.toString());

    }

    private void lowerColumnSensorArm() {

        if (robot.isStillWaiting()) return;

        // TODO add servo

    }


    public void atColumnEvaluateRobotLocation() {

        if (robot.isStillWaiting() || this.isWaitingForColumnClear) return;

        updateRobotColumnPosition();


        if (isAtTargetColumnPosition()) {
            Log.i(getLogKey(), "At Target Location!!!");
            stopRobot();
        } else {

            this.isWaitingForColumnClear = true;
            resetVoteCounts();
//            raiseColumnSensorArm();
//            robot.setTimeDelay(1500);

        }

    }

    private boolean isAtTargetColumnPosition() {

        return this.targetPositionEnum.equals(this.currentPositionEnum);

    }

    private void raiseColumnSensorArm() {

        // TODO maybe....

    }

    private void updateRobotColumnPosition() {


        columnCount++;

        if (columnCount == 1) {
            this.currentPositionEnum = RelicRecoveryVuMark.LEFT;
        } else if (columnCount == 2) {
            this.currentPositionEnum = RelicRecoveryVuMark.CENTER;
        } else if (columnCount == 3) {
            this.currentPositionEnum = RelicRecoveryVuMark.RIGHT;
        } else {
            this.currentPositionEnum = RelicRecoveryVuMark.UNKNOWN;
        }


    }

    private void resetVoteCounts() {

        this.voteClear = 0;
        this.voteColumn = 0;

    }


    private void setTargetColumnPosition_runsOnlyOnce() {

        if (targetPositionEnum == null) {

//            this.targetPositionEnum = robot.getVuMark();
            this.targetPositionEnum = RelicRecoveryVuMark.CENTER;

        }
    }


    private void stopRobot() {

        robot.motorRobotSideways.setPower(0);

    }

    @Override
    public boolean isStepDone() {
        if (robot.isStillWaiting()) return false;

        if (isAtTargetColumnPosition()) {
            Log.i(getLogKey(), "Step is Done:");
            stopRobot();
            return true;
        }
        return false;
    }

    private void readColorSensorAndVotePosition() {

        if (robot.isStillWaiting()) return;

        if (readRedColor() > RED_COLOR_VALUE | readBlueColor() > BLUE_COLOR_VALUE) {

            voteColumn();

        } else {

            voteClear();

        }

    }

    private void voteColumn() {

        if (isWaitingForColumnClear) return;

        this.voteColumn++;
        this.voteClear = 0;
//        Log.i(getLogKey(), "read COLUMN");

    }

    private void voteClear() {

        if (this.voteClear < 15) {

            this.voteClear++;
            this.voteColumn = 0;

        }
//        Log.i(getLogKey(), "read CLEAR");

    }

    private boolean isAtColumn() {

        return (this.voteColumn - this.voteClear) >= 5;

    }

    private boolean isAtClear() {

        return (this.voteClear - this.voteColumn) >= 5;

    }

    private int readRedColor() {
        double red = this.robot.colorSensorColumn.red() * SCALE_FACTOR;
        Log.i(getLogKey(), "RED COLOR:" + red + " | DISTANCE:" + this.robot.distanceSensorColumn.getDistance(DistanceUnit.CM));
        return (int) red;
    }

    private int readBlueColor() {
        double blue = this.robot.colorSensorColumn.blue() * SCALE_FACTOR;
        Log.i(getLogKey(), "BLUE COLOR:" + blue + " | DISTANCE:" + this.robot.distanceSensorColumn.getDistance(DistanceUnit.CM));

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
