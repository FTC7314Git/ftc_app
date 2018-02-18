package org.firstinspires.ftc.teamcode.sabbotage.relic.autonomous.steps;

import org.firstinspires.ftc.teamcode.sabbotage.relic.robot.Robot;

public interface StepInterface {


    void setRobot(Robot robot);

    void runStep();

    boolean isStepDone();

    String getLogKey();
}
