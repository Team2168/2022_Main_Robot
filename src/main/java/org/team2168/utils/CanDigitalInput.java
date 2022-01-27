package org.team2168.utils;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;

public class CanDigitalInput {

    BaseTalon motor;

    public CanDigitalInput(BaseTalon motor) {
        this.motor = motor;
    }

    public boolean isFwdLimitSwitchClosed() {
        return (this.motor.isFwdLimitSwitchClosed() == 1);
    }

    public boolean isRevLimitSwitchClosed(){
            return (this.motor.isRevLimitSwitchClosed() == 1);
    }
}
