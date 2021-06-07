#include "kdl_ros_control/kdl_object.h"
#include "kdl_ros_control/alglib/optimization.h"
#include "kdl_ros_control/utils.h"

using namespace alglib;

Contact::Contact()
{
    F_ = KDL::Frame::Identity();
    T_ = KDL::Twist::Zero();
    W_ = KDL::Wrench::Zero();
    B_ = Eigen::Matrix<double,6,3>::Zero();
    B_ << 1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0;
    Bo_ << 0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,1;
    mu_ = 0.5;
    theta_ = std::atan(mu_);
    z_hat_ = Eigen::Vector3d(0,0,1);
    KDL::Vector fhat1 = KDL::Rot(KDL::Vector( theta_,0,0))*KDL::Vector(0,0,1);
    KDL::Vector fhat2 = KDL::Rot(KDL::Vector(-theta_,0,0))*KDL::Vector(0,0,1);
    KDL::Vector fhat3 = KDL::Rot(KDL::Vector(0, theta_,0))*KDL::Vector(0,0,1);
    KDL::Vector fhat4 = KDL::Rot(KDL::Vector(0,-theta_,0))*KDL::Vector(0,0,1);
    Fc_hat_.block(0,0,3,1) = toEigen(fhat1);
    Fc_hat_.block(0,1,3,1) = toEigen(fhat2);
    Fc_hat_.block(0,2,3,1) = toEigen(fhat3);
    Fc_hat_.block(0,3,3,1) = toEigen(fhat4);

}

KDL::Frame Contact::getFrame() const
{
    return F_;
}

KDL::Wrench Contact::getWrench() const
{
    return W_;
}

Eigen::Matrix<double, 3, 4> Contact::getConeVersors() const
{
    return Fc_hat_;
}

Eigen::Matrix<double, 6, 3> Contact::getB() const
{
    return B_;
}

void Contact::setFrame(const KDL::Frame &value)
{
    F_ = value;
}
void Contact::setWrench(const KDL::Wrench &value)
{
    W_ = value;
}
void Contact::setTwist(const KDL::Twist &value)
{
    T_ = value;
}

Eigen::Matrix<double, 6, 3> Contact::getBo() const
{
    return Bo_;
}

double Contact::getTheta() const
{
    return theta_;
}

void KDLObject::addContact(const KDL::Frame &F_bc)
{
    Contact c;
    c.setFrame(F_bc);
    contacts_.push_back(c);

    KDL::Frame F_cb = F_bc.Inverse();
    Eigen::Matrix<double,6,6> Ad_F_cb = adjoint(toEigen(F_cb.p), toEigen(F_cb.M));
    G_.block(0,(contacts_.size()-1)*3,6,3) = Ad_F_cb.transpose()*c.getB();
    Go_.block(0,(contacts_.size()-1)*3,3,3) = (Ad_F_cb.transpose()*c.getBo()).block(3,0,3,3);
}

KDLObject::KDLObject()
{

}

KDLObject::KDLObject(double &m, Eigen::Matrix3d &I, std::vector<KDL::Frame> &c, double &mu) : mu_(mu), mass_(m), I_(I)
{
    g_ << 0.0, 0.0, -9.81;
    V_b_ = KDL::Twist::Zero();
    W_b_ = KDL::Wrench::Zero();

    massMatrix = Eigen::Matrix<double,6,6>::Zero();
    massMatrix.block(0,0,3,3) = m*Eigen::Matrix<double,3,3>::Identity();
    massMatrix.block(3,3,3,3) = I;

    G_.setZero();
    for (unsigned int i = 0; i < c.size(); i++)
        addContact(c.at(i));

    std::cout << "Object created " << std::endl;
    std::cout << "Grasp matrix: " << std::endl;
    std::cout << G_ << std::endl;
    std::cout << "Friction cone matrix: " << std::endl;
    std::cout << getContacts().at(0).getConeVersors() << std::endl;
}

void KDLObject::computeContactForces(KDL::Wrench &F_b)
{
    //
    // This example demonstrates minimization of nonconvex function
    //     F(x0,x1) = -(x0^2+x1^2)
    // subject to constraints x0,x1 in [1.0,2.0]
    // Exact solution is [x0,x1] = [2,2].
    //
    // Non-convex problems are harded to solve than convex ones, and they
    // may have more than one local minimum. However, ALGLIB solves may deal
    // with such problems (altough they do not guarantee convergence to
    // global minimum).
    //
    // IMPORTANT: this solver minimizes  following  function:
    //     f(x) = 0.5*x'*A*x + b'*x.
    // Note that quadratic term has 0.5 before it. So if you want to minimize
    // quadratic function, you should rewrite it in such way that quadratic term
    // is multiplied by 0.5 too.
    //
    // For example, our function is f(x)=-(x0^2+x1^2), but we rewrite it as
    //     f(x) = 0.5*(-2*x0^2-2*x1^2)
    // and pass diag(-2,-2) as quadratic term - NOT diag(-1,-1)!
    //

    int decisionVariablesNr = 34, linearConstraintsNr = 34;
    real_2d_array H, A; // cost, linear constraints
    real_1d_array x0, s, bndl, bndu; // starting point, scale, lower, upper bound
    integer_1d_array At; // constraint tyoe >0 =0 <0
    H.setlength(decisionVariablesNr, decisionVariablesNr);
    A.setlength(linearConstraintsNr, decisionVariablesNr+1);
    At.setlength(decisionVariablesNr);
    x0.setlength(decisionVariablesNr);
    s.setlength(decisionVariablesNr);
    bndl.setlength(decisionVariablesNr);
    bndu.setlength(decisionVariablesNr);

    // linear constraints
    Eigen::Matrix<double,34,35> A_eigen;
    A_eigen.setZero();
    A_eigen.block(0,0,6,6) = Eigen::Matrix<double,6,6>::Identity();
    A_eigen.block(0,6,6,12) = G_;
    A_eigen.block(6,6,12,12) = -Eigen::Matrix<double,12,12>::Identity();
    A_eigen.block(6,18,3,4) = contacts_.at(0).getConeVersors();
    A_eigen.block(9,22,3,4) = contacts_.at(0).getConeVersors();
    A_eigen.block(12,26,3,4) = contacts_.at(0).getConeVersors();
    A_eigen.block(15,30,3,4) = contacts_.at(0).getConeVersors();
    A_eigen.block(18,18,16,16) = Eigen::Matrix<double,16,16>::Identity();
    A_eigen.block(0,34,6,1) = toEigen(F_b);

    for (unsigned int i = 0; i < A_eigen.rows(); i++ ){
        if (i < 18)
            At(i) = 0.0; // equality constraints
        else
            At(i) = 1.0; // inequality constraints

        for (unsigned int j = 0; j < A_eigen.cols(); j++ ){
            A(i,j) = A_eigen(i,j);
        }
    }

// std::cout << "A matrix:" << std::endl;
// std::cout << A_eigen << std::endl;

    // bound constraints
    Eigen::Matrix<double,35,1> X;
    for ( int i = 0; i < decisionVariablesNr; i++ )
    {
       bndl(i) = fp_neginf;
       bndu(i) = fp_posinf;
       if (i>=18)
              bndl(i) = 0.0;
    }
    bndl(2) = 0; //Fb*z - Fbz
    bndu(2) = 0;
    bndl(3) = 0; //Fb*b - Fbb
    bndu(3) = 0;
    bndl(4) = 0; //Fb*b - Fbb
    bndu(4) = 0;
    bndl(5) = 0; //Fb*b - Fbb
    bndu(5) = 0;
    bndl(8) = 0.0; //Fc1z
    bndl(11) = 0.0; //Fc2z
    bndl(14) = 0.0; //Fc3z
    bndl(17) = 0.0; //Fc4z

    // cost function
    Eigen::Matrix<double,34,34> H_eigen = 1*Eigen::Matrix<double,34,34>::Identity();
    H_eigen.block(0,0,6,6) = 2000*Eigen::Matrix<double,6,6>::Identity();
    for ( int i = 0; i < H_eigen.rows(); i++ ){
        for ( int j = 0; j < H_eigen.cols(); j++ )
             H(i,j) = H_eigen(i,j);
    }

    real_1d_array x;
    minqpstate state;
    minqpreport rep;

    // create solver, set quadratic/linear terms, constraints
    minqpcreate(34, state);
    minqpsetquadraticterm(state, H);
    // minqpsetstartingpoint(state, x0);
    minqpsetbc(state, bndl, bndu);
    minqpsetlc(state, A, At);

    // Set scale of the parameters.
    // It is strongly recommended that you set scale of your variables.
    // Knowing their scales is essential for evaluation of stopping criteria
    // and for preconditioning of the algorithm steps.
    // You can find more information on scaling at http://www.alglib.net/optimization/scaling.php
    //
    // NOTE: there also exists minqpsetscaleautodiag() function
    //       which automatically determines variable scales; however,
    //       it does NOT work for non-convex problems.
    //    minqpsetscale(state, s);
    minqpsetscaleautodiag(state);

    //
    // Solve problem with BLEIC-based QP solver.
    //
    // This solver is intended for problems with moderate (up to 50) number
    // of general linear constraints and unlimited number of box constraints.
    //
    // It may solve non-convex problems as long as they are bounded from
    // below under constraints.
    //
    // Default stopping criteria are used.
    //
    minqpsetalgobleic(state, 0, 0, 0, 0);
    minqpoptimize(state);
    minqpresults(state, x, rep);
//    std::cout << "Termination type:" << std::endl;
//    printf("%d\n", int(rep.terminationtype));
    //printf("%s\n", x.tostring(2).c_str()); // EXPECTED: [2,2]

    Eigen::VectorXd x_eigen = toEigen(x);
    std::cout << "tilde Fb: " << x_eigen.block(0,0,6,1).transpose() << std::endl;
    Eigen::Matrix<double,6,1> F = -(x_eigen.block(0,0,6,1) - toEigen(F_b));
    std::cout << "Fb star: " << toEigen(F_b).transpose() << std::endl;
    std::cout << "Fb: " << F.transpose() << std::endl;
    F_b = toKDLWrench(F);
    Eigen::Matrix<double,12,1> Fc = x_eigen.block(6,0,12,1);
    std::cout << "Fc:" << std::endl;
    std::cout << Fc.transpose() << std::endl;
    std::cout << "Fb: " << G_*Fc << std::endl;
    Eigen::Matrix<double,16,1> lambda = x_eigen.block(18,0,16,1);
    std::cout << "lambda:" << std::endl;
    std::cout << lambda.transpose() << std::endl;

    std::vector<KDL::Wrench> c_w;
    c_w.resize(this->getContacts().size());
    for (unsigned int i = 0; i<getContacts().size(); i++)
        c_w.at(i) = KDL::Wrench(toKDL(Fc.block(0+3*i,0,3,1)), KDL::Vector(0,0,0));

    this->setContactWrenches(c_w);

    //
    // Solve problem with DENSE-AUL solver.
    //
    // This solver is optimized for problems with up to several thousands of
    // variables and large amount of general linear constraints. Problems with
    // less than 50 general linear constraints can be efficiently solved with
    // BLEIC, problems with box-only constraints can be solved with QuickQP.
    // However, DENSE-AUL will work in any (including unconstrained) case.
    //
    // Algorithm convergence is guaranteed only for convex case, but you may
    // expect that it will work for non-convex problems too (because near the
    // solution they are locally convex).
    //
    // Default stopping criteria are used.
    //
//    minqpsetalgodenseaul(state, 1.0e-9, 1.0e+4, 5);
//    minqpoptimize(state);
//    minqpresults(state, x, rep);
//    // printf("%s\n", x.tostring(2).c_str()); // EXPECTED: [2,2]

//    x_eigen = toEigen(x);
//    std::cout << "tilde Fb: " << x_eigen.block(0,0,6,1).transpose() << std::endl;
//    F = -(x_eigen.block(0,0,6,1) - toEigen(F_b));
//    std::cout << "Fb star: " << toEigen(F_b).transpose() << std::endl;
//    std::cout << "Fb: " << F.transpose() << std::endl;
//    F_b = toKDLWrench(F);
//    Fc = x_eigen.block(6,0,12,1);
//    std::cout << "Fc:" << std::endl;
//    std::cout << Fc.transpose() << std::endl;
//    std::cout << "Fb: " << G_*Fc << std::endl;
//    lambda = x_eigen.block(18,0,16,1);
//    std::cout << "lambda:" << std::endl;
//    std::cout << lambda.transpose() << std::endl;
//    x_eigen = toEigen(x);
//    F = -(x_eigen.block(0,0,6,1) - toEigen(F_b));
//    std::cout << "Fb:" << std::endl;
//    std::cout << F << std::endl;
//    Fc = x_eigen.block(6,0,12,1);
//    std::cout << "Fc:" << std::endl;
//    std::cout << Fc << std::endl;
//    lambda = x_eigen.block(18,0,16,1);
//    std::cout << "lambda:" << std::endl;
//    std::cout << lambda << std::endl;


//    // Hmm... this problem is bounded from below (has solution) only under constraints.
//    // What it we remove them?
//    //
//    // You may see that BLEIC algorithm detects unboundedness of the problem,
//    // -4 is returned as completion code. However, DENSE-AUL is unable to detect
//    // such situation and it will cycle forever (we do not test it here).
//    real_1d_array nobndl = "[-inf,-inf]";
//    real_1d_array nobndu = "[+inf,+inf]";
//    minqpsetbc(state, nobndl, nobndu);
//    minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
//    minqpoptimize(state);
//    minqpresults(state, x, rep);
//    printf("%d\n", int(rep.terminationtype)); // EXPECTED: -4
}

KDL::Wrench KDLObject::computeID()
{
    KDL::Wrench Fb;
    computeC();
    computeN();
    Eigen::Matrix<double,6,6> M = Eigen::Matrix<double,6,6>::Identity();
    M.block(0,0,3,3) = mass_*Eigen::Matrix3d::Identity();
    M.block(3,3,3,3) = I_;
    Fb = toKDLWrench(C_ - N_);
    return Fb;
}

void KDLObject::computeC()
{
    KDL::Twist t = getBodyVelocity();
    Eigen::Vector3d v = toEigen((t.vel));
    Eigen::Vector3d omega = toEigen((t.rot));

    C_ << omega.cross(mass_*v), omega.cross(I_*omega);
}

void KDLObject::computeN()
{
    KDL::Rotation R = getFrame().M.Inverse();
    N_ << mass_*toEigen(R)*g_, 0.0, 0.0, 0.0;
}

KDL::Twist KDLObject::computeFD(const KDL::Wrench &Fb) const
{
    Eigen::Matrix<double,6,1> Vbdot = getMassMatrix().inverse()*(-C_ + N_ + toEigen(Fb));
    return toKDLTwist(Vbdot);
}

Eigen::Matrix<double, 6, 6> KDLObject::getMassMatrix() const
{
    return massMatrix;
}


void KDLObject::computeContactVelocities(KDL::Twist &V_b)
{

}

Eigen::Matrix<double, 6, 12> KDLObject::getGraspMatrix() const
{
    return G_;
}

Eigen::Matrix<double, 3, 12> KDLObject::getGraspMatrixOri() const
{
    return Go_;
}

double KDLObject::getMass() const
{
    return mass_;
}

Eigen::Matrix<double, 3, 3> KDLObject::getInertia() const
{
    return I_;
}

KDL::Frame KDLObject::getFrame() const
{
    return F_b_;
}

void KDLObject::setFrame(const KDL::Frame &value)
{
    F_b_ = value;
}

KDL::Wrench KDLObject::getBodyWrench() const
{
    return W_b_;
}

void KDLObject::setBodyWrench(const KDL::Wrench &value)
{
    W_b_ = value;
}

KDL::Twist KDLObject::getBodyVelocity() const
{
    return V_b_;
}

void KDLObject::setBodyVelocity(const KDL::Twist &value)
{
    V_b_ = value;
}

KDL::Twist KDLObject::getSpatialVelocity() const
{
    return s_V_b_;
}

void KDLObject::setSpatialVelocity(const KDL::Twist &value)
{
    s_V_b_ = value;
}

std::vector<Contact> KDLObject::getContacts() const
{
    return contacts_;
}

void KDLObject::setContactFrames(const std::vector<KDL::Frame> &value)
{
    for (unsigned int i = 0; i < value.size(); i++)
        contacts_.at(i).setFrame(value.at(i));
}

void KDLObject::setContactWrenches(const std::vector<KDL::Wrench> &value)
{
    for (unsigned int i = 0; i < value.size(); i++)
        contacts_.at(i).setWrench(value.at(i));
}

std::vector<KDL::Wrench> KDLObject::getContactsWrenches() const
{
    std::vector<KDL::Wrench> w;
    w.resize(contacts_.size());

    for (unsigned int i = 0; i < getContacts().size(); i++)
        w.at(i) = contacts_.at(i).getWrench();
    return w;
}
