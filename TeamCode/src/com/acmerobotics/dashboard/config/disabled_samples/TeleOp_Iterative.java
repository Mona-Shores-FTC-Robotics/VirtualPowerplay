/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package com.acmerobotics.dashboard.config.disabled_samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ObjectClasses.DriveTrain;

@TeleOp(name="TeleOp Mode", group="Iterative OpMode")
@Disabled
public class TeleOp_Iterative extends OpMode
{
    // Declare OpMode members.
    DriveTrain MecDrive = new DriveTrain();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        MecDrive.init(hardwareMap);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        // GamePad Inputs
        MecDrive.drive = -gamepad1.left_stick_y; //-1.0 to 1.0
        MecDrive.strafe = gamepad1.left_stick_x; //-1.0 to 1.0 // right trigger strafe right, left trigger strafe left
        MecDrive.turn  = -gamepad1.right_stick_x; //-1.0 to 1.0

        //  Robot Functions
        MecDrive.MecanumDrive();

        // Show the elapsed game time and wheel power.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
        telemetry.addData("Status", "Run Time: " + runtime);
        telemetry.addData("Motors", "leftfront(%.2f), rightfront (%.2f)", MecDrive.LFDrive.getPower(), MecDrive.RFDrive.getPower());
        telemetry.addData("Motors", "leftback (%.2f), rightback (%.2f)",MecDrive.LBDrive.getPower(), MecDrive.RBDrive.getPower());
        telemetry.addData("Drive", MecDrive.drive);
        telemetry.addData("Strafe", MecDrive.strafe);
        telemetry.addData("Turn", MecDrive.turn);

        telemetry.addData("Motors ", MecDrive.LFDrive.getMode());
        telemetry.update();
    }

    @Override
    public void stop() {
    }

}
