package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.graphics.Color;
import android.util.Log;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_ReadForwardJewelColor implements AutonomousOp.StepInterface {

    private Robot robot;
    private boolean initDoneFlag;
    private Robot.TeamEnum teamColor;
    private Robot.TeamEnum fowardJewelColor;
    private final double SCALE_FACTOR = 255;


    // Constructor, called to create an instance of this class.
    public Step_ReadForwardJewelColor(Robot.TeamEnum teamColor) {

        this.teamColor = teamColor;
    }


    @Override
    public String getLogKey() {
        return "Step_ReadForwardJewelColor";
    }




    @Override
    public void runStep() {


    }

    @Override
    public boolean isStepDone() {

        if (hasFoundJewelColor()) {

            this.robot.setJewelColor(this.fowardJewelColor);

            logIt("Step is Done:");
            return true;
        }

        return false;
    }


    private void logIt(String methodName) {

        StringBuilder sb = new StringBuilder();
        sb.append(methodName);
        sb.append(" CurrentPosition:" + robot.motorDriveLeft.getCurrentPosition());
        sb.append(" Target:" + robot.motorDriveLeft.getTargetPosition());
        Log.i(getLogKey(), sb.toString());

    }

    private boolean hasFoundJewelColor() {

        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV(readRedColor(), readGreanColor(), readBlueColor(),
                hsvValues);


        // TODO Add threshold condition for jewel colors

        return false;


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
