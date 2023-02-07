package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class AutoPivotTest extends LinearOpMode {
    //--HARDWARE VARIABLE INIT--
    private DcMotor rightDrive; //PORT 0
    private DcMotor leftDrive; //PORT 1
    private DcMotorEx liftMotor; //PORT 2
    private DcMotorEx pivotMotor; //PORT 3

    //--CUSTOM FUNCTION DECLARATIONS--
    private int moveDynamic(double powerR, double powerL) {
        this.leftDrive.setPower(-1.0D * powerL);
        this.rightDrive.setPower(powerR);
        return 1;
    }

    @Override
    public void runOpMode() {
        //--HARDWARE  MAPPINGS--
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        pivotMotor = hardwareMap.get(DcMotorEx.class, "pivotMotor");

        //--RUNTIME VARIABLES:--
        //For Drive-Train
        final double FwdDriveSpeedMax = 0.70;
        final double TurnSpeedMax = 0.5;
        double RightDriveSpeed = 0.5;
        double LeftDriveSpeed = 0.5;
        double threshold = 0.5;
        //For Linear Lift
        int liftPosition = 0;
        double liftPower = 0.8;
        //For Lift Pivot
        int pivotPosition = 0;
        double pivotPower = 1.0;

        this.telemetry.addData("Status:", "Running");

        //--MOTOR ENCODER CONFIGURATIONS:--
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setTargetPosition(liftPosition);
        liftMotor.setPower(liftPower);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setTargetPosition(pivotPosition);
        pivotMotor.setPower(pivotPower);
        pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //Runtime Loop:
        while (opModeIsActive()) {
            /*
            LeftDriveSpeed = 0;
            RightDriveSpeed = 0;
            double stick_x = this.gamepad1.left_stick_x;
            double stick_y = this.gamepad1.right_stick_y;
            */
            /*
            //Drive-Train Controls:
            if (stick_y > threshold) {
                LeftDriveSpeed += FwdDriveSpeedMax;
                RightDriveSpeed += FwdDriveSpeedMax;
            }

            if (stick_y < -threshold) {
                LeftDriveSpeed += FwdDriveSpeedMax * -1;
                RightDriveSpeed += FwdDriveSpeedMax * -1;
            }

            if (stick_x < -threshold) {
                RightDriveSpeed += TurnSpeedMax * -1;
                LeftDriveSpeed += TurnSpeedMax;
            }

            if (stick_x > threshold) {
                RightDriveSpeed += TurnSpeedMax;
                LeftDriveSpeed += TurnSpeedMax * -1;
            }

            //Lift Controls
            if (gamepad1.dpad_up && liftMotor.getCurrentPosition() == liftPosition) {
                liftPosition++;
                liftMotor.setTargetPosition(liftPosition);
            }

            else if (gamepad1.dpad_down && liftMotor.getCurrentPosition() == liftPosition) {
                liftPosition--;
                liftMotor.setTargetPosition(liftPosition);
            }

            else {
                //pass?
            }
            */

            /*
            //Pivot Controls
            if (gamepad1.right_bumper && pivotMotor.getCurrentPosition() == pivotPosition) {
                pivotPosition++;
                pivotMotor.setTargetPosition(pivotPosition);
            }

            else if (gamepad1.left_bumper && pivotMotor.getCurrentPosition() == pivotPosition) {
                pivotPosition--;
                pivotMotor.setTargetPosition(pivotPosition);
            }

            else {
                //pass?
            }
            */

            //moveDynamic(RightDriveSpeed, LeftDriveSpeed);

            pivotMotor.setTargetPosition(150);

            this.telemetry.update();
        }
    }
}
