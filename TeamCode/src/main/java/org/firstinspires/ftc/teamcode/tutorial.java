package org.firstinspires.ftc.teamcode;

//Libraries
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tutorial")
public class tutorial extends OpMode {

    //For the motor config(Soon)
    DcMotor motor;

    //When Init button is pressed
    @Override
    public void init() {
        //Sends the text to the device and update it
        telemetry.addData("Initialization:", "is a success");
        telemetry.update();
    }
    //When pressing the start button, loop the code
    @Override
    public void loop() {
        //When "A" button is pressed...
        if (gamepad1.a){
            telemetry.addData("Button:", "A");
            telemetry.update();
        }
    }
}