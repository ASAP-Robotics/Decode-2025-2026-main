/*
 * Copyright 2025 ASAP Robotics (FTC Team 22029)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class ActiveIntake {
    private DcMotorEx intake_intakeMotor;
    public boolean intaking = false;
    public boolean ejecting = false;
    public boolean ballIn = false;

    ActiveIntake(DcMotorEx intakeMotor) {
        intake_intakeMotor = intakeMotor;
        intake_intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD); // spin forwards
        intake_intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // brake if zero power
    }

    /**
     * @brief stops the intake from spinning
     */
    public void stop() {
        intake_intakeMotor.setPower(0);
        intaking = false;
        ejecting = false;
    }

    /**
     * @brief brings a ball into the intake
     * @note placeholder; TODO: update
     */
    public void intake() {
        intake_intakeMotor.setPower(1);
        intaking = true;
    }

    /**
     * @brief ejects a ball from the intake
     * @note placeholder; TODO: update
     */
    public void eject() {
        intake_intakeMotor.setPower(-1);
        ejecting = true;
    }
}
