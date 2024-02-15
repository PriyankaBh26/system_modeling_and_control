# include <iostream>
# include <vector>
# include <cmath>
# include <Eigen/Dense>

# include "multi_joint_robots/multi_joint_robots_helper_funs.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


void TestTfmatInverse() {
    MatrixXd Mat(4,4); 
    Mat << 1, 0,  0, 0,
           0, 0, -1, 0,
           0, 1,  0, 3,
           0, 0,  0, 1;

    MatrixXd Mat_inverse_expected(4,4);
    Mat_inverse_expected << 1,  0, 0,  0,
                            0,  0, 1, -3,
                            0, -1, 0,  0,
                            0,  0, 0,  1;

    MatrixXd Mat_inverse = TfmatInverse(Mat);
    std::cout << "\nTestTfmatInverse:\n";
    std::cout << "\n Mat_inverse - Mat_inverse_expected:\n";
    std::cout << Mat_inverse - Mat_inverse_expected;
    std::cout << "\n";
}

void TestSkewSymMatToVec() {
    MatrixXd W(3,3);
    W << 0, -3,  2,
         3,  0, -1,
        -2,  1,  0;
    VectorXd w = SkewSymMatToVec(W);
    VectorXd w_expected(3);
    w_expected << 1, 2, 3;
    std::cout << "\nTestSkewSymMatToVec\n";
    std::cout << "\n w - w_expected : " << w.transpose() - w_expected.transpose();
    std::cout << "\n";

}

void TestVecToSkewSymMat() {
    MatrixXd W_expected(3,3);
    W_expected << 0, -3,  2,
                  3,  0, -1,
                 -2,  1,  0;
    VectorXd w(3);
    w << 1, 2, 3;
    MatrixXd W = VecToSkewSymMat(w);
    std::cout << "\nTestVecToSkewSymMat\n";
    std::cout << "\n W - W_expected : " << W - W_expected;
    std::cout << "\n";

}

void Testad() {
    VectorXd V(6);
    V << 1, 2, 3, 4, 5, 6;

    MatrixXd ad_wv_expected(6,6);
    ad_wv_expected << 0, -3,  2,  0,  0,  0,
                        3,  0, -1,  0,  0,  0,
                        -2,  1,  0,  0,  0,  0,
                        0, -6,  5,  0, -3,  2,
                        6,  0, -4,  3,  0, -1,
                        -5,  4,  0, -2,  1,  0;

    MatrixXd ad_wv = ad(V);

    std::cout << "\nTestad\n";
    std::cout << "\n ad_wv - ad_wv_expected : " << ad_wv - ad_wv_expected;
    std::cout << "\n";


}

void TestMatrixExp3() {
    MatrixXd W(3,3);
    W << 0, -3,  2,
              3,  0, -1,
             -2,  1,  0;

    MatrixXd expmat_expected(3,3);
    expmat_expected << -0.69492056,  0.71352099,  0.08929286,
              -0.19200697, -0.30378504,  0.93319235,
               0.69297817,  0.6313497 ,  0.34810748;

    VectorXd w = SkewSymMatToVec(W);
    double theta = w.norm();

    MatrixXd expmat = MatrixExp3(W, theta);

    std::cout << "\n TestMatrixExp3\n";
    std::cout << "\n expmat - expmat_expected:\n" << expmat - expmat_expected;         
    std::cout << "\n";
}

void TestMatrixLog3() {
    MatrixXd R(3,3);
    R << 0, 0, 1,
         1, 0, 0,
         0, 1, 0;
    
    MatrixXd W_expected(3,3);
    W_expected << 0, -1.20919958,  1.20919958,
                  1.20919958,    0, -1.20919958,
                 -1.20919958,  1.20919958,  0;

    MatrixXd W = MatrixLog3(R);
    std::cout << "\nTestMatrixLog3\n";
    std::cout << "\n W - W_expected\n: " << W - W_expected;
    std::cout << "\n";
}

void TestSe3ToVec() {
    MatrixXd V_B(4,4);
    V_B <<   0, -3,  2, 4,
             3,  0, -1, 5,
            -2,  1,  0, 6,
             0,  0,  0, 0;

    VectorXd V_b_expected(6);
    V_b_expected << 1, 2, 3, 4, 5, 6;
    VectorXd V_b = Se3ToVec(V_B);
    std::cout << "\nTestSe3ToVec\n";
    std::cout << "\n V_b - V_b_expected : " << V_b.transpose() - V_b_expected.transpose();
    std::cout << "\n";
}

void TestMatrixExp6() {
    MatrixXd V_B(4,4);
    V_B << 0,          0,           0,           0,
            0,          0, -1.57079633,  2.35619449,
            0, 1.57079633,           0,  2.35619449,
            0,          0,           0,           0;
            
    MatrixXd tf_expected(4,4);
    tf_expected << 1, 0,  0, 0,
                    0, 0, -1, 0,
                    0, 1,  0, 3,
                    0, 0,  0, 1;

    MatrixXd t_f = MatrixExp6(V_B);

    std::cout << "\nTestMatrixExp6\n";
    std::cout << "\n t_f - tf_expected:\n " << t_f;
    std::cout << "\n";
 }
 
void TestMatrixLog6() {
    MatrixXd tf_body(4,4);
    tf_body << 1, 0,  0, 0,
               0, 0, -1, 0,
               0, 1,  0, 3,
               0, 0,  0, 1;
    MatrixXd V_B_expected(4,4);
    V_B_expected << 0,          0,           0,           0,
                    0,          0, -1.57079633,  2.35619449,
                    0, 1.57079633,           0,  2.35619449,
                    0,          0,           0,           0;

    MatrixXd V_B = MatrixLog6(tf_body);

    std::cout << "\nTestMatrixLog6\n";
    std::cout << "\n V_B - V_B_expected:\n " << V_B - V_B_expected;
    std::cout << "\n";
}

void TestVecToSe3() {
    VectorXd V_b(6);
    V_b << 1, 2, 3, 4, 5, 6;

    MatrixXd V_B = VecToSe3(V_b);

    MatrixXd V_B_expected(4,4);
    V_B_expected <<   0, -3,  2, 4,
             3,  0, -1, 5,
            -2,  1,  0, 6,
             0,  0,  0, 0;

    std::cout << "\nTestVecToSe3\n";
    std::cout << "\n V_B - V_B_expected:\n " << V_B - V_B_expected;
    std::cout << "\n";

}

int main() {

    // TestSkewSymMatToVec();

    // TestVecToSkewSymMat();

    Testad();

    // TestMatrixExp3();

    // TestMatrixLog3();

    // TestSe3ToVec();

    // TestMatrixExp6();

    // TestMatrixLog6();

    // TestVecToSe3();

    return 0;
}