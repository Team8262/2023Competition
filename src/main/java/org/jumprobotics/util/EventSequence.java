// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.jumprobotics.util;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;


/** Add your docs here. */
public class EventSequence {

    private Supplier<Boolean> endFlag;
    private Command command;
    public EventSequence(Supplier<Boolean> endFlag, Command command){
        this.endFlag = endFlag;
        this.command = command;
    }

    public boolean isFinished(){
        return endFlag.get();
    }

    public void runCommand(){
        command.schedule();
    }

}
