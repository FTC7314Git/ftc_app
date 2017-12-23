package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;


import android.util.Log;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_TurnLeft implements AutonomousOp.StepInterface {

    private static final double TARGET_TOLERANCE = 1;
    private static final int SLOW_MODE_REMAINING_ANGLE = 60;

    private Robot.MotorPowerEnum motorPowerEnum = Robot.MotorPowerEnum.Med;

    protected Robot robot;

    private boolean resetMotors_DoneFlag = false;
    protected Double targetAngle;


    // Constructor, called to create an instance of this class.
    public Step_TurnLeft(double angleDegrees) {

        this.targetAngle = angleDegrees;
    }

    @Override
    public String getLogKey() {
        return "Step_TurnLeft";
    }


    @Override
    public void runStep() {

        resetMotorsAndInitializeDirection_Only_Once();

        if (robot.isStillWaiting()) {
            return;
        }

        // We should not need to do this, but have see when motor(s) have not been reset.
        robot.runWithEncoders_MAINTAINS_SPEED();

        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        turn();

    }

    protected double determinePower() {


        double absRemainingAngle = Math.abs(remainingAngle());


        if (absRemainingAngle < SLOW_MODE_REMAINING_ANGLE) {

            return limitMinValue(this.motorPowerEnum.getValue() * absRemainingAngle / (SLOW_MODE_REMAINING_ANGLE));
        }


        return this.motorPowerEnum.getValue();

    }

    private double limitMinValue(double input) {

        if (input < .15) {

            return .15;
        }

        return input;
    }


    private boolean isAtTargetAngle() {

        return Math.abs(remainingAngle()) <= TARGET_TOLERANCE;
    }

    protected double remainingAngle() {

        return this.targetAngle - robot.getAngle();

    }

    private void turn() {


        double power = determinePower();

        if (hasOverShotTargetAngle()) {

            robot.motorDriveRight.setPower(-power);

            robot.motorDriveLeft.setPower(+power);

            logIt("Overshot:");

        } else {


            robot.motorDriveRight.setPower(+power);

            robot.motorDriveLeft.setPower(-power);
            logIt("Turning:");
        }
    }

    protected boolean hasOverShotTargetAngle() {
        return remainingAngle() < 0;
    }


    private void resetMotorsAndInitializeDirection_Only_Once() {

        if (resetMotors_DoneFlag == false) {

            robot.runWithoutEncoders();
            robot.setDriveMotorForwardDirection();

            resetMotors_DoneFlag = true;
            robot.setLoopDelay();
        }

    }


    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting() || resetMotors_DoneFlag == false) {
            return false;

        }


        if (isAtTargetAngle()) {

            logIt("isStepDone");

            robot.motorDriveRight.setPower(0);
            robot.motorDriveLeft.setPower(0);
            return true;
        }


        return false;
    }


    private void logIt(String methodName) {

        StringBuilder sb = new StringBuilder();
        sb.append(" , OS:" + hasOverShotTargetAngle());
        sb.append(" , CurrentAngle:" + robot.getAngle());
        sb.append(" , TargetAngle:" + targetAngle);
        sb.append(" , RemainingAngle:" + remainingAngle());
        sb.append(" , L Power:" + robot.motorDriveLeft.getPower() + " , R Power:" + robot.motorDriveRight.getPower());

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
