/*
 * Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/


/****************************************************************/
class ArmXyzNoHeaveNLP_ForwardDiff : public ArmCommonNLP
{
public:
    /****************************************************************/
    ArmXyzNoHeaveNLP_ForwardDiff(ArmSolver &slv_) : ArmCommonNLP(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "xyz_pose+no_heave+forward_diff";
    }

    /****************************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m, Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag, IndexStyleEnum &index_style)
    {
        n=x0.length();
        m=2+2;
        nnz_jac_g=6+6;
        nnz_h_lag=0;
        index_style=TNLP::C_STYLE;

        return true;
    }

    /****************************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l, Ipopt::Number *x_u,
                         Ipopt::Index m, Ipopt::Number *g_l, Ipopt::Number *g_u)
    {
        size_t offs;

        offs=0;
        for (size_t i=0; i<3; i++)
        {
            x_l[offs+i]=torso.l_min;
            x_u[offs+i]=torso.l_max;
        }

        offs+=3;
        iKinChain *chain=upper_arm.asChain();
        for (size_t i=0; i<upper_arm.getDOF(); i++)
        {
            x_l[offs+i]=(*chain)[i].getMin();
            x_u[offs+i]=(*chain)[i].getMax();
        }

        offs+=upper_arm.getDOF();
        for (size_t i=0; i<3; i++)
        {
            x_l[offs+i]=lower_arm.l_min;
            x_u[offs+i]=lower_arm.l_max;
        }

        g_l[0]=g_u[0]=0.0;
        g_l[1]=torso.cos_alpha_max; g_u[1]=1.0;

        g_l[2]=g_u[2]=0.0;
        g_l[3]=lower_arm.cos_alpha_max; g_u[3]=1.0;

        latch_idx.clear();
        latch_gl.clear();
        latch_gu.clear();

        latch_idx.push_back(1);
        latch_gl.push_back(g_l[1]);
        latch_gu.push_back(g_u[1]);

        latch_idx.push_back(3);
        latch_gl.push_back(g_l[3]);
        latch_gu.push_back(g_u[3]);

        return true;
    }

    /****************************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number &obj_value)
    {
        computeQuantities(x,new_x);

        Ipopt::Number postural_torso=0.0;
        Ipopt::Number postural_torso_yaw=0.0;
        Ipopt::Number postural_upper_arm=0.0;
        Ipopt::Number postural_lower_arm=0.0;
        Ipopt::Number tmp;

        if (wpostural_torso!=0.0)
        {
            tmp=x[0]-x[1]; 
            postural_torso+=tmp*tmp;
            tmp=x[1]-x[2];
            postural_torso+=tmp*tmp;
        }

        if (wpostural_torso_yaw!=0.0)
            postural_torso_yaw+=x[3]*x[3];

        if (wpostural_upper_arm!=0.0)
        {
            for (size_t i=1; i<upper_arm.getDOF(); i++)
            {
                tmp=x[3+i]-x0[3+i];
                postural_upper_arm+=tmp*tmp;
            }
        }

        if (wpostural_lower_arm!=0.0)
        {
            tmp=x[9]-x[10];
            postural_lower_arm+=tmp*tmp;
            tmp=x[10]-x[11];
            postural_lower_arm+=tmp*tmp;            
        }

        obj_value=norm2(xd-T.getCol(3).subVector(0,2))+
                  wpostural_torso*postural_torso+
                  wpostural_torso_yaw*postural_torso_yaw+
                  wpostural_upper_arm*postural_upper_arm+
                  wpostural_lower_arm*postural_lower_arm;

        return true;
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=xd-T.getCol(3).subVector(0,2);
        Vector e_fw;
        Matrix M;

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        TripodState d_fw;

        // (torso)
        M=H*d2.T*TN;

        x_dx[0]=x[0]+drho;
        d_fw=tripod_fkin(1,x_dx);
        e_fw=xd-(d_fw.T*M).getCol(3).subVector(0,2);
        grad_f[0]=2.0*(dot(e,e_fw-e)/drho + wpostural_torso*(x[0]-x[1]));
        x_dx[0]=x[0];

        x_dx[1]=x[1]+drho;
        d_fw=tripod_fkin(1,x_dx);
        e_fw=xd-(d_fw.T*M).getCol(3).subVector(0,2);
        grad_f[1]=2.0*(dot(e,e_fw-e)/drho + wpostural_torso*(2.0*x[1]-x[0]-x[2]));
        x_dx[1]=x[1];

        x_dx[2]=x[2]+drho;
        d_fw=tripod_fkin(1,x_dx);
        e_fw=xd-(d_fw.T*M).getCol(3).subVector(0,2);
        grad_f[2]=2.0*(dot(e,e_fw-e)/drho + wpostural_torso*(x[2]-x[1]));
        x_dx[2]=x[2];

        // (upper_arm)
        Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
        grad_f[3]=grad[0] + 2.0*wpostural_torso_yaw*x[3];
        for (size_t i=1; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*wpostural_upper_arm*(x[3+i]-x0[3+i]);

        // (lower_arm)
        M=d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        grad_f[9]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[9]-x[10]));
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        grad_f[10]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(2.0*x[10]-x[9]-x[11]));
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        grad_f[11]=2.0*(dot(e,e_fw-e)/drho + wpostural_lower_arm*(x[11]-x[10]));
        x_dx[11]=x[11];

        return true;
    }

    /****************************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Index m, Ipopt::Number *g)
    {
        computeQuantities(x,new_x);

        double e1=hd1-din1.p[2];
        g[0]=e1*e1;
        g[1]=din1.n[2];

        double e2=hd2-din2.p[2];
        g[2]=e2*e2;
        g[3]=din2.n[2];

        latch_x_verifying_alpha(n,x,g);

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            // g[0] (torso)
            iRow[0]=0; jCol[0]=0;
            iRow[1]=0; jCol[1]=1;
            iRow[2]=0; jCol[2]=2;

            // g[1] (torso)
            iRow[3]=1; jCol[3]=0;
            iRow[4]=1; jCol[4]=1;
            iRow[5]=1; jCol[5]=2;

            // g[2] (lower_arm)
            iRow[6]=2; jCol[6]=9;
            iRow[7]=2; jCol[7]=10;
            iRow[8]=2; jCol[8]=11;

            // g[3] (lower_arm)
            iRow[9]=3;  jCol[9]=9;
            iRow[10]=3; jCol[10]=10;
            iRow[11]=3; jCol[11]=11;
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw;

            // g[0,1] (torso)
            double e1=hd1-din1.p[2];

            x_dx[0]=x[0]+drho;
            tripod_fkin(1,x_dx,&d_fw);
            values[0]=-2.0*e1*(d_fw.p[2]-din1.p[2])/drho;
            values[3]=(d_fw.n[2]-din1.n[2])/drho;
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            tripod_fkin(1,x_dx,&d_fw);
            values[1]=-2.0*e1*(d_fw.p[2]-din1.p[2])/drho;
            values[4]=(d_fw.n[2]-din1.n[2])/drho;
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            tripod_fkin(1,x_dx,&d_fw);
            values[2]=-2.0*e1*(d_fw.p[2]-din1.p[2])/drho;
            values[5]=(d_fw.n[2]-din1.n[2])/drho;
            x_dx[2]=x[2];

            // g[2,3] (lower_arm)
            double e2=hd2-din2.p[2];

            x_dx[9]=x[9]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[6]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[9]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[7]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[10]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            values[8]=-2.0*e2*(d_fw.p[2]-din2.p[2])/drho;
            values[11]=(d_fw.n[2]-din2.n[2])/drho;
            x_dx[11]=x[11];
        }

        return true;
    }
};


/****************************************************************/
class ArmXyzNoHeaveNLP_CentralDiff : public ArmXyzNoHeaveNLP_ForwardDiff
{
public:
    /****************************************************************/
    ArmXyzNoHeaveNLP_CentralDiff(ArmSolver &slv_) : ArmXyzNoHeaveNLP_ForwardDiff(slv_)
    {
    }

    /****************************************************************/
    string get_mode() const
    {
        return "xyz_pose+no_heave+central_diff";
    }

    /****************************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number* x, bool new_x,
                     Ipopt::Number *grad_f)
    {
        computeQuantities(x,new_x);

        Vector e=xd-T.getCol(3).subVector(0,2);
        Vector e_fw,e_bw;
        Matrix M;

        Ipopt::Number x_dx[12];
        for (Ipopt::Index i=0; i<n; i++)
            x_dx[i]=x[i];

        TripodState d_fw,d_bw;

        // (torso)
        M=H*d2.T*TN;

        x_dx[0]=x[0]+drho;
        d_fw=tripod_fkin(1,x_dx);
        e_fw=xd-(d_fw.T*M).getCol(3).subVector(0,2);
        x_dx[0]=x[0]-drho;
        d_bw=tripod_fkin(1,x_dx);
        e_bw=xd-(d_bw.T*M).getCol(3).subVector(0,2);
        grad_f[0]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_torso*(x[0]-x[1]);
        x_dx[0]=x[0];

        x_dx[1]=x[1]+drho;
        d_fw=tripod_fkin(1,x_dx);
        e_fw=xd-(d_fw.T*M).getCol(3).subVector(0,2);
        x_dx[1]=x[1]-drho;
        d_bw=tripod_fkin(1,x_dx);
        e_bw=xd-(d_bw.T*M).getCol(3).subVector(0,2);
        grad_f[1]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_torso*(2.0*x[1]-x[0]-x[2]);
        x_dx[1]=x[1];

        x_dx[2]=x[2]+drho;
        d_fw=tripod_fkin(1,x_dx);
        e_fw=xd-(d_fw.T*M).getCol(3).subVector(0,2);
        x_dx[2]=x[2]-drho;
        d_bw=tripod_fkin(1,x_dx);
        e_bw=xd-(d_bw.T*M).getCol(3).subVector(0,2);
        grad_f[2]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_torso*(x[2]-x[1]);
        x_dx[2]=x[2];

        // (upper_arm)
        Vector grad=-2.0*(J_.submatrix(0,2,0,upper_arm.getDOF()-1).transposed()*e);
        grad_f[3]=grad[0] + 2.0*wpostural_torso_yaw*x[3];
        for (size_t i=1; i<grad.length(); i++)
            grad_f[3+i]=grad[i] + 2.0*wpostural_upper_arm*(x[3+i]-x0[3+i]);

        // (lower_arm)
        M=d1.T*H;

        x_dx[9]=x[9]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        x_dx[9]=x[9]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
        grad_f[9]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[9]-x[10]);
        x_dx[9]=x[9];

        x_dx[10]=x[10]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        x_dx[10]=x[10]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
        grad_f[10]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(2.0*x[10]-x[9]-x[11]);
        x_dx[10]=x[10];

        x_dx[11]=x[11]+drho;
        d_fw=tripod_fkin(2,x_dx);
        e_fw=xd-(M*d_fw.T*TN).getCol(3).subVector(0,2);
        x_dx[11]=x[11]-drho;
        d_bw=tripod_fkin(2,x_dx);
        e_bw=xd-(M*d_bw.T*TN).getCol(3).subVector(0,2);
        grad_f[11]=dot(e,e_fw-e_bw)/drho + 2.0*wpostural_lower_arm*(x[11]-x[10]);
        x_dx[11]=x[11];

        return true;
    }

    /****************************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                    Ipopt::Index m, Ipopt::Index nele_jac, Ipopt::Index *iRow,
                    Ipopt::Index *jCol, Ipopt::Number *values)
    {
        if (values==NULL)
        {
            // g[0] (torso)
            iRow[0]=0; jCol[0]=0;
            iRow[1]=0; jCol[1]=1;
            iRow[2]=0; jCol[2]=2;

            // g[1] (torso)
            iRow[3]=1; jCol[3]=0;
            iRow[4]=1; jCol[4]=1;
            iRow[5]=1; jCol[5]=2;

            // g[2] (lower_arm)
            iRow[6]=2; jCol[6]=9;
            iRow[7]=2; jCol[7]=10;
            iRow[8]=2; jCol[8]=11;

            // g[3] (lower_arm)
            iRow[9]=3;  jCol[9]=9;
            iRow[10]=3; jCol[10]=10;
            iRow[11]=3; jCol[11]=11;
        }
        else
        {
            computeQuantities(x,new_x);

            Ipopt::Number x_dx[12];
            for (Ipopt::Index i=0; i<n; i++)
                x_dx[i]=x[i];

            TripodState d_fw,d_bw;

            // g[0,1] (torso)
            double e1=hd1-din1.p[2];

            x_dx[0]=x[0]+drho;
            tripod_fkin(1,x_dx,&d_fw);
            x_dx[0]=x[0]-drho;
            tripod_fkin(1,x_dx,&d_bw);
            values[0]=-e1*(d_fw.p[2]-d_bw.p[2])/drho;
            values[3]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[0]=x[0];

            x_dx[1]=x[1]+drho;
            tripod_fkin(1,x_dx,&d_fw);
            x_dx[1]=x[1]-drho;
            tripod_fkin(1,x_dx,&d_bw);
            values[1]=-e1*(d_fw.p[2]-d_bw.p[2])/drho;
            values[4]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[1]=x[1];

            x_dx[2]=x[2]+drho;
            tripod_fkin(1,x_dx,&d_fw);
            x_dx[2]=x[2]-drho;
            tripod_fkin(1,x_dx,&d_bw);
            values[2]=-e1*(d_fw.p[2]-d_bw.p[2])/drho;
            values[5]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[2]=x[2];

            // g[2,3] (lower_arm)
            double e2=hd2-din2.p[2];

            x_dx[9]=x[9]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[9]=x[9]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[6]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[9]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[9]=x[9];

            x_dx[10]=x[10]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[10]=x[10]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[7]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[10]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[10]=x[10];

            x_dx[11]=x[11]+drho;
            tripod_fkin(2,x_dx,&d_fw);
            x_dx[11]=x[11]-drho;
            tripod_fkin(2,x_dx,&d_bw);
            values[8]=-e2*(d_fw.p[2]-d_bw.p[2])/drho;
            values[11]=(d_fw.n[2]-d_bw.n[2])/(2.0*drho);
            x_dx[11]=x[11];
        }

        return true;
    }
};


