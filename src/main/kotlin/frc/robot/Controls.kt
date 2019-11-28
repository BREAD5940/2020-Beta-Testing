package frc.robot

import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.wrappers.hid.* // ktlint-disable no-wildcard-imports

object Controls {

    val driverControllerLowLevel = XboxController(0)
    val driverFalconXbox = driverControllerLowLevel.mapControls {
        button(kX).changeOn(InstantCommand(Runnable { println("Hullo!") }))
    }

    val operatorWPIJoystick = XboxController(1)
    val operatorFalconXbox = operatorWPIJoystick.mapControls {
    }

    fun update() {
        driverFalconXbox.update()
//        operatorFalconHID.update()
        operatorFalconXbox.update()
    }
}

private fun Command.andThen(block: () -> Unit) = sequential { +this@andThen ; +InstantCommand(Runnable(block)) }
