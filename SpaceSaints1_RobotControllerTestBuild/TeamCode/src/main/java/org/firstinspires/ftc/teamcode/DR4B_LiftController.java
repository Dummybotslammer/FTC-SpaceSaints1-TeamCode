package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp
public class DR4B_LiftController extends LinearOpMode {
    private DcMotorEx liftMotor;

    @Override
    public void runOpMode() {
        //--HARDWARE MAPPINGS:--
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");

        //--RUNTIME VARIABLES:--
        int liftPosition = 0;
        double liftPower = 0.8;

        this.telemetry.addData("Status", "Running");

        //--MOTOR ENCODER CONFIGURATIONS:--
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(liftPosition);
        liftMotor.setPower(liftPower);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        //Runtime Loop:
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !(liftMotor.isBusy())) {
                liftPosition++;
                liftMotor.setTargetPosition(liftPosition);
            }

            else if (gamepad1.dpad_down && !(liftMotor.isBusy())) {
                liftPosition--;
                liftMotor.setTargetPosition(liftPosition);
            }

            else {
                //pass?
            }

            //liftMotor.setTargetPosition(liftPosition);
        }
    }
}
