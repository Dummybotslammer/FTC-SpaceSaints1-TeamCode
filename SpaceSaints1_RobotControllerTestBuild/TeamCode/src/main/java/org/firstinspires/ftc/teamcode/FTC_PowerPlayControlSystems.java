package org.firstinspires.ftc.teamcode;

public class FTC_PowerPlayControlSystems {

    public static double[] PID_Output(double error, double previousError, long deltaTime, double[] PID_values, double[] PID_coefficients) {
        //time is in nanoSeconds
        //PID_values in order: Proportional Value, Cumulative Integral Value, Derivative Value
        //PID_coefficients in order: Proportional Gain, Integral Gain, Derivative Gain
        PID_values[0] = PID_coefficients[0] * error;
        PID_values[1] += PID_coefficients[1] * (deltaTime * error);
        PID_values[2] = PID_coefficients[2] * (error - previousError / deltaTime);
        return PID_values;
    }

    public static double[][] DifferentialDrivePID_Output(
            double leftError,
            double previousLeftError,
            double rightError,
            double previousRightError,
            long deltaTime,
            double[] leftPID_values,
            double[] leftPID_coefficients,
            double[] rightPID_values,
            double[] rightPID_coefficients) {
        //left -
        //right +
        leftPID_values = PID_Output(leftError*-1, previousLeftError*-1, deltaTime, leftPID_values, leftPID_coefficients);
        rightPID_values = PID_Output(rightError, previousRightError, deltaTime, rightPID_values, rightPID_coefficients);

        double[][] PID_values = {leftPID_values, rightPID_values};
        return PID_values;
    }

}
