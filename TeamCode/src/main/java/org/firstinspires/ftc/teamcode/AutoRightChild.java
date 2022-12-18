package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoRightChild", group = "ConceptBlue")
public class AutoRightChild extends AutoParent{
    public double adjustTurn(double angle){
       return -angle;
    }
}
