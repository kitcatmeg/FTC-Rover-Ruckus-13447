package org.firstinspires.ftc.robotcontroller.internal;

public interface IIMU {

    double getXAngle();

    double getYAngle();

    double getZAngle();

    double getXAcc();

    double getYAcc();

    double getZAcc();

    double getXVel();

    double getYVel();

    double getZVel();

    void calibrateDefaults();

    void setOffset(double offset);

    void setCurrentPosToZero();

    void init();

}