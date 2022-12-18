package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "AutoLeftChild", group = "ConceptBlue")
public class AutoLeftChild extends AutoParent{
    public double adjustTurn(double angle){
       return angle;
    }
}
