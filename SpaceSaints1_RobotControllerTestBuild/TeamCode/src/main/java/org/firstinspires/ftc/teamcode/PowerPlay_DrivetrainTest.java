package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class PowerPlay_DrivetrainTest extends LinearOpMode {
    private DcMotor left;
    private DcMotor right;

    //--CUSTOM FUNCTION DECLARATIONS--
    private int moveLinear(double power) {
        this.left.setPower(-1.0D * power);
        this.right.setPower(power);
        return 1;
    }

    private int moveDynamic(double powerR, double powerL) {
        this.left.setPower(-1.0D * powerL);
        this.right.setPower(powerR);
        return 1;
    }

    private int setBrakes() {
        this.left.setPower(0.0D);
        this.right.setPower(0.0D);
        this.left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return 1;
    }

    private void brake() {
        this.left.setPower(0.0D);
        this.right.setPower(0.0D);
    }


    @Override
    public void runOpMode() {
        //--HARDWARE MAPPINGS:--
        this.left = (DcMotor)this.hardwareMap.get(DcMotor.class, "leftDrive");
        this.right = (DcMotor)this.hardwareMap.get(DcMotor.class, "rightDrive");

        this.telemetry.addData("Status:", "Running");

        //--RUNTIME VARIABLES:--
        //For Drive-Train
        final double FwdDriveSpeedMax = 0.70;
        final double TurnSpeedMax = 0.5;
        //double FwdDriveSpeed = 0.70;
        double RightDriveSpeed = 0.5;
        double LeftDriveSpeed = 0.5;
        double threshold = 0.5;
        double increment = 0.05;

        waitForStart();

        //Runtime Loop:
        while (opModeIsActive()) {
            LeftDriveSpeed = 0;
            RightDriveSpeed = 0;
            double x = this.gamepad1.left_stick_x;
            double y = this.gamepad1.right_stick_y;

            //Drive-Train Controls:
            if (y > threshold) {
                LeftDriveSpeed += FwdDriveSpeedMax;
                RightDriveSpeed += FwdDriveSpeedMax;
            }

            if (y < -threshold) {
                LeftDriveSpeed += FwdDriveSpeedMax * -1;
                RightDriveSpeed += FwdDriveSpeedMax * -1;
            }

            if (x < -threshold) {
                RightDriveSpeed += TurnSpeedMax * -1;
                LeftDriveSpeed += TurnSpeedMax;
            }

            if (x > threshold) {
                RightDriveSpeed += TurnSpeedMax;
                LeftDriveSpeed += TurnSpeedMax * -1;
            }

            moveDynamic(RightDriveSpeed, LeftDriveSpeed);

            this.telemetry.update();
        }
    }
}
