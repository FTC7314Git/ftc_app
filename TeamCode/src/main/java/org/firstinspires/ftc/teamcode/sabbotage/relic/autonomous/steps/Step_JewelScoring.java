package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.graphics.Color;
import android.util.Log;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_JewelScoring implements AutonomousOp.StepInterface {

    private Robot robot;
    private boolean lowerJewelArmDoneFlag;
    private boolean raiseJewelArmDoneFlag;
    private boolean displaceJewelDoneFlag;
    private Robot.TeamEnum teamColor;
    private Robot.TeamEnum forwardJewelColor;

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

        if (robot.isStillWaiting()) return;

        lowerJewelArm_runsOnlyOnce();

        determineJewelColor();

        if (isJewelColorKnown()) {
            displaceJewel_runsOnlyOnce();
        }

        if (isJewelDisplaced()) {
            raiseJewelArm_runsOnlyOnce();
        }


    }

    private void lowerJewelArm_runsOnlyOnce() {


        if (robot.isStillWaiting()) return;


        if (lowerJewelArmDoneFlag == false) {
            robot.servoJewelArm.setPosition(robot.SERVO_JEWEL_ARM_POSITION_DOWN);
            lowerJewelArmDoneFlag = true;
            robot.setTimeDelay(2000);
        }
    }

    private void raiseJewelArm_runsOnlyOnce() {

        if (robot.isStillWaiting()) return;

        if (raiseJewelArmDoneFlag == false) {

            stopRobot();

            robot.servoJewelArm.setPosition(robot.SERVO_JEWEL_ARM_POSITION_UP);
            raiseJewelArmDoneFlag = true;
            robot.setTimeDelay(2000);
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
            robot.setTimeDelay(2000);
        }
    }


    private void driveRobotToDisplaceJewel() {

        double power = .2;

        if (this.teamColor.equals(this.forwardJewelColor)) {

            robot.motorDriveRight.setPower(-power);
            robot.motorDriveLeft.setPower(-power);


        } else {
            robot.motorDriveRight.setPower(+power);
            robot.motorDriveLeft.setPower(+power);
        }
    }

    @Override
    public boolean isStepDone() {

        if (robot.isStillWaiting()) return false;


        if (raiseJewelArmDoneFlag) {
            Log.i(getLogKey(), "Step is Done:");
            return true;
        }
        return false;
    }

    private void determineJewelColor() {

        if (robot.isStillWaiting()) return;


        if (isJewelColorKnown()) {
            return;
        }

        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV(readRedColor(), readGreanColor(), readBlueColor(),
                hsvValues);


        // TODO Add threshold condition for jewel colors

        this.forwardJewelColor = Robot.TeamEnum.BLUE;

    }

    private boolean isJewelColorKnown() {
        return this.forwardJewelColor != null;
    }


    private boolean isJewelDisplaced() {
        return this.displaceJewelDoneFlag && !robot.isStillWaiting();
    }

    private int readRedColor() {
        double red = this.robot.colorSensorJewel.red() * SCALE_FACTOR;
        return (int) red;
    }

    private int readBlueColor() {
        double blue = this.robot.colorSensorJewel.blue() * SCALE_FACTOR;
        return (int) blue;
    }

    private int readGreanColor() {
        double green = this.robot.colorSensorJewel.green() * SCALE_FACTOR;
        return (int) green;
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
