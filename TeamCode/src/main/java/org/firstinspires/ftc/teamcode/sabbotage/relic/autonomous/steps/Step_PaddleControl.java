package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

import android.util.Log;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_PaddleControl implements AutonomousOp.StepInterface {


    private Robot robot;
    private Robot.PaddlePosition paddlePosition;

    // Constructor, called to create an instance of this class.
    public Step_PaddleControl(Robot.PaddlePosition paddlePosition) {

        this.paddlePosition = paddlePosition;
    }


    @Override
    public String getLogKey() {
        return "Step_PaddleControl";
    }


    @Override
    public void runStep() {

        if (Robot.PaddlePosition.CLOSE.equals(this.paddlePosition)) {

            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
        }

        if (Robot.PaddlePosition.OPEN.equals(this.paddlePosition)) {

            robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_OPEN);
            robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_CLOSE);
        }


    }


    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting()) {
            return false;
        }

        return true;
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
