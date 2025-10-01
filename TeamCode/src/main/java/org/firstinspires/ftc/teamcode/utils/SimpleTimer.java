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

package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class SimpleTimer {
    private final ElapsedTime timer = new ElapsedTime();
    private double duration; // seconds
    private boolean running = false;

    public SimpleTimer(double durationSeconds) {
        this.duration = durationSeconds;
    }

    /**
     * @brief start or restart the timer
     */
    public void start() {
        timer.reset();
        running = true;
    }

    /**
     * @brief start or restart the timer with a set duration
     */
    public void start(double durationSeconds) {
        this.duration = durationSeconds;
        start();
    }

    /**
     * stop the timer
     */
    public void stop() {
        running = false;
    }

    /**
     * @brief call each loop to update the timer
     */
    public void update() {
        if (running && timer.seconds() >= duration) {
            running = false; // auto-stop when done
        }
    }

    /**
     * @brief returns if the timer is running
     * @return true if the timer is running, false if the timer isn't running
     */
    public boolean isRunning() {
        update();
        return running;
    }

    /**
     * @brief returns if the timer is done
     * @return true if the timer is done, false if the timer isn't done
     */
    public boolean isFinished() {
        update();
        return !running && timer.seconds() >= duration;
    }

    /**
     * @brief returns time since the timer was started
     * @return the elapsed time since the timer was started
     */
    public double elapsed() {
        update();
        return timer.seconds();
    }

    /**
     * @brief returns the time left on the timer
     * @return the amount of time left on the timer, or 0 if finished
     */
    public double remaining() {
        update();
        return Math.max(0, duration - timer.seconds());
    }
}
