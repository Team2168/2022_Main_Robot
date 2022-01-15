package org.team2168;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class CanDigitalInput {

    TalonFX motor;

    public CanDigitalInput(TalonFX motor){
        this.motor = motor;
    }

    public boolean getForwardLimit(){
        if(this.motor.getSensorCollection().isFwdLimitSwitchClosed() == 1)
            return true;
        else 
            return false;
    }

    public boolean getReverseLimit(){
        if(this.motor.getSensorCollection().isRevLimitSwitchClosed() == 1)
            return true;
        else 
            return false;
    }
}
