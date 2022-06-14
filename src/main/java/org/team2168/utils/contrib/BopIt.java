/**
 * Copyright 2022 Nathan Jankowski
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

package org.team2168.utils.contrib;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * WPILib controller implementation
 */
public class BopIt {
    private static final int CENTER_BUTTON = 2;
    private static final int TWIST_BUTTON = 3;
    private static final int PULL_BUTTON = 6;
    private static final int SPIN_BUTTON = 5;
    private static final int FLICK_BUTTON = 4;
    private static final int TOGGLE_BUTTON = 7;

    private Joystick joystick;


    public BopIt(int port) {
        joystick = new Joystick(port);
    }

    public Button center() {
        return new Button(() -> joystick.getRawButton(CENTER_BUTTON) && !joystick.getRawButton(TOGGLE_BUTTON));
    }
    
    public Button centerToggled() {
        return new Button(() -> joystick.getRawButton(CENTER_BUTTON) && joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button twist() {
        return new Button(() -> joystick.getRawButton(TWIST_BUTTON) && !joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button twistToggled() {
        return new Button(() -> joystick.getRawButton(TWIST_BUTTON) && joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button pull() {
        return new Button(() -> joystick.getRawButton(PULL_BUTTON) && !joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button pullToggled() {
        return new Button(() -> joystick.getRawButton(PULL_BUTTON) && joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button spin() {
        return new Button(() -> joystick.getRawButton(SPIN_BUTTON) && !joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button spinToggled() {
        return new Button(() -> joystick.getRawButton(SPIN_BUTTON) && joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button flick() {
        return new Button(() -> joystick.getRawButton(FLICK_BUTTON) && !joystick.getRawButton(TOGGLE_BUTTON));
    }

    public Button flickToggled() {
        return new Button(() -> joystick.getRawButton(FLICK_BUTTON) && joystick.getRawButton(TOGGLE_BUTTON));
    }
}