package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.internal.AutonomousOp;
import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public class Step_PaddleControl implements AutonomousOp.StepInterface, StepInterface {


    private Robot robot;
    private Robot.PaddlePosition paddlePosition;

    boolean paddlePositionDoneFlag;

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

        Log.i(getLogKey(), "runStep");
        if (paddlePositionDoneFlag == false) {

            if (Robot.PaddlePosition.CLOSE.equals(this.paddlePosition)) {

                robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
                robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_LEFT_CLOSE);
                Log.i(getLogKey(), "CLOSE Servos");
            }

            if (Robot.PaddlePosition.OPEN.equals(this.paddlePosition)) {

                robot.servoLeftPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_OPEN);
                robot.servoRightPaddle.setPosition(Robot.SERVO_PADDLE_RIGHT_OPEN);
                Log.i(getLogKey(), "Open Servos");
            }

            paddlePositionDoneFlag = true;

            robot.sleep(1000);

        }

    }


    @Override
    public boolean isStepDone() {


        if (robot.isStillWaiting()) {
            return false;
        }

        return paddlePositionDoneFlag;
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
