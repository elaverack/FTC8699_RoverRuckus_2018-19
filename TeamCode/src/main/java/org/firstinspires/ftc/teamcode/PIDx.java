package org.firstinspires.ftc.teamcode;

/**
 * Created by Akshay on 11/15/17.
 */

public class PIDx {

    private int P;
    private int I;
    private int D;
    private boolean switched = false;

    public PID(double p, double i, double d)
    {
        P = p;
        I = i;
        D = d;
    }

    //make set methods pls akshay
    //make get methods too

    private void signs()
    {
        if (switched)
        {
            if(P > 0)
            {
                P*=-1;
            }
            if(I > 0)
            {
                I *= -1;
            }
            if(D > 0)
            {
                D *= -1;
            }
        }
    }

    public void setP(double p)
    {
        P = p;
        signs();
    }

    public void setI(double i)
    {
        I = i;
    }

    public void setD(double d)
    {
        D = d;
        signs();
    }

    public int getP() {
        return P;
    }

    public int getI() {
        return I;
    }

    public int getD() {
        return D;
    }

    public double
}
