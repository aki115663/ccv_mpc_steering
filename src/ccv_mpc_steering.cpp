//ros
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <ccv_dynamixel_msgs/CmdPoseByRadian.h>


//ipopt
#include <Eigen/Core>
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

class MPC{
public:
    MPC();

    // state, ref_x, ref_y, ref_yaw
    std::vector<double> solve(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

};

class FG_eval{
public:
    FG_eval(Eigen::VectorXd, Eigen::VectorXd, Eigen::VectorXd);

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector&, const ADvector&);

private:
    Eigen::VectorXd ref_x;
    Eigen::VectorXd ref_y;
    Eigen::VectorXd ref_yaw;

};

class MPCPathTracker
{
public:
    MPCPathTracker(void);

    void path_callback(const nav_msgs::PathConstPtr&);
    void joint_callback(const sensor_msgs::JointState::ConstPtr&);

    void process(void);
    void calc_ref_trajectory(void);
    std::vector<double> calc_yaw(int);

private:
    ros::NodeHandle nh;
    ros::Publisher velocity_pub;

    ros::Publisher path_pub;
    ros::Publisher ref_path_pub;
    ros::Publisher cmd_pos_pub;
    ros::Subscriber path_sub;
    ros::Subscriber joint_sub;
    MPC mpc;
    nav_msgs::Path path;
    sensor_msgs::JointState joint_state;
    tf::TransformListener listener;
    geometry_msgs::PoseStamped current_pose;
    geometry_msgs::PoseStamped previous_pose;
    tf::StampedTransform _transform;
    geometry_msgs::TransformStamped transform;
    Eigen::VectorXd ref_x;
    Eigen::VectorXd ref_y;
    Eigen::VectorXd ref_yaw;
    bool first_transform = true;
    double last_time;
    int current_index;
    
};


// ホライゾン長さ
int T = 15;
// 周期
double DT = 0.1;// [s]
const double HZ = 10;
// 目標速度
double VREF=1.2;// [m/s]
// 最大速度
double MAX_VELOCITY=1.5; // [m/s]
// 最大角速度
double MAX_ANGULAR_VELOCITY=1.57;// [rad/s]
// ホイール角加速度
double WHEEL_ANGULAR_ACCELERATION_LIMIT=10;// [rad/s^2]
// double WHEEL_ANGULAR_ACCELERATION_LIMIT=1000000;// [rad/s^2]
// ホイール角速度
// double WHEEL_ANGULAR_VELOCITY_LIMIT=10000000;// [rad/s]
double WHEEL_ANGULAR_VELOCITY_LIMIT=10;// [rad/s]
//ステア角
double STEERING_ANGLE_LIMIT=M_PI/6; //[rad]
//ステア角速度
double STEERING_ANGULAR_VELOCITY_LIMIT=10; //[rad/s]
// ホイール半径
double WHEEL_RADIUS=0.1435;// [m]
// トレッド
double TREAD=0.501;// [m]
//ピッチオフセット
double PITCH_OFFSET = 3.0*M_PI/180.0; //[rad]
// グリッドマップ分解能
double RESOLUTION=0.1;// [m]
std::string WORLD_FRAME="odom";
std::string ROBOT_FRAME="base_link";
std::string VELOCITY_TOPIC_NAME="/local/cmd_vel";
std::string INTERMEDIATE_PATH_TOPIC_NAME="/local_path";

// state
size_t x_start = 0;
size_t y_start = x_start + T;
size_t yaw_start = y_start + T;
size_t v_start = yaw_start + T;
size_t omega_start = v_start + T;
size_t omega_r_start = omega_start + T;
size_t omega_l_start = omega_r_start + T;
size_t steer_r_start = omega_l_start + T;
size_t steer_l_start = steer_r_start + T;
// input
size_t domega_r_start = steer_l_start + T;
size_t domega_l_start = domega_r_start + T - 1;
size_t dsteer_r_start = domega_l_start + T - 1;
size_t dsteer_l_start = dsteer_r_start + T - 1;


// 最適化失敗時は最後の成功データを使う
int failure_count = 0;
std::vector<double> result;

MPCPathTracker::MPCPathTracker(void)
{
    velocity_pub = nh.advertise<geometry_msgs::Twist>(VELOCITY_TOPIC_NAME, 1);
    cmd_pos_pub = nh.advertise<ccv_dynamixel_msgs::CmdPoseByRadian>("/local/cmd_pos", 1);
    path_pub = nh.advertise<geometry_msgs::PoseArray>("/mpc_path", 1);
    ref_path_pub = nh.advertise<geometry_msgs::PoseArray>("/reference_path", 1);
    path_sub = nh.subscribe(INTERMEDIATE_PATH_TOPIC_NAME, 1, &MPCPathTracker::path_callback, this);
    joint_sub = nh.subscribe("/sq2_ccv/joint_states", 10, &MPCPathTracker::joint_callback, this);
    ref_x = Eigen::VectorXd::Zero(T);
    ref_y = Eigen::VectorXd::Zero(T);
    ref_yaw = Eigen::VectorXd::Zero(T);
    std::cout << "=== mpc_path_tracker ===" << std::endl;
    
}

void MPCPathTracker::path_callback(const nav_msgs::PathConstPtr& msg)
{
    // std::cout << "path callback" << std::endl;
    path = *msg;
}
void MPCPathTracker::joint_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    joint_state = *msg;
    // for(int i=0; i<joint_state.name.size(); i++){
    //     std::cout << joint_state.name[i] << std::endl;
    // }
    // omega_r = msg->velocity[8];
    // omega_l = msg->velocity[6];
    // steer_r = msg->position[7];
    // steer_l = msg->position[5];
    // std::cout << "R/L" << steer_r << "/" << steer_l << std::endl;
}

MPC::MPC(){}

std::vector<double> MPC::solve(Eigen::VectorXd state, Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
{
    /*
     * state:x, y, yaw, v, omega, omega_r, omega_l, steer_r, steer_l
     */
    bool ok = true;
    size_t i;
    typedef CPPAD_TESTVECTOR(double) Dvector;
    double x = state[0];
    double y = state[1];
    double yaw = state[2];
    double v = state[3];
    double omega = state[4];
    double omega_r = state[5];
    double omega_l = state[6];
    double steer_r = state[7];
    double steer_l = state[8];

    // 9(x, y, yaw, v, omega, omega_r, omega_l, steer_r, steer_l), 4(domega_r, domega_l, dsteer_r, dsteer_l)
    size_t n_variables = 9 * T + 4 * (T - 1);
    //std::cout << "n_variables : " << n_variables << std::endl;
    //std::cout << "dsteer_l_start : " << dsteer_l_start << std::endl;
    size_t n_constraints = 9 * T;
    //std::cout << "constraints size:" << n_constraints << std::endl;

    Dvector vars(n_variables);
    for(int i=0;i<n_variables;i++){
        vars[i] = 0.0;
    }

    vars[x_start] = x;
    vars[y_start] = y;
    vars[yaw_start] = yaw;
    vars[v_start] = v;
    vars[omega_start] = omega;
    vars[omega_r_start] = omega_r;
    vars[omega_l_start] = omega_l;
    vars[steer_r_start] = steer_r;
    vars[steer_l_start] = steer_l;

    Dvector vars_lower_bound(n_variables);
    Dvector vars_upper_bound(n_variables);

    for(int i=0;i<v_start;i++){
        // x, y, yaw
        vars_lower_bound[i] = -1.0e19;
        vars_upper_bound[i] = 1.0e19;
    }
    for(int i=v_start;i<omega_start;i++){
        // v
        vars_lower_bound[i] = 0.0;
        vars_upper_bound[i] = MAX_VELOCITY;
    }
    for(int i=omega_start;i<omega_r_start;i++){
        // omega
        vars_lower_bound[i] = -MAX_ANGULAR_VELOCITY;
        vars_upper_bound[i] = MAX_ANGULAR_VELOCITY;
    }
    for(int i=omega_r_start;i<steer_r_start;i++){
        // omega_r, omega_l
        vars_lower_bound[i] = -WHEEL_ANGULAR_VELOCITY_LIMIT;
        vars_upper_bound[i] = WHEEL_ANGULAR_VELOCITY_LIMIT;
    }
    for(int i=steer_r_start; i<domega_r_start; i++){
        //steer_r, steer_l
        vars_lower_bound[i] = -STEERING_ANGLE_LIMIT;
        vars_upper_bound[i] = STEERING_ANGLE_LIMIT;
    }
    for(int i=domega_r_start;i<dsteer_r_start;i++){
        // domega_r, domega_l
        vars_lower_bound[i] = -WHEEL_ANGULAR_ACCELERATION_LIMIT;
        vars_upper_bound[i] = WHEEL_ANGULAR_ACCELERATION_LIMIT;
    }
    for(int i=dsteer_r_start; i<n_variables; i++){
        //dsteer_r, dsteer_l
        vars_lower_bound[i] = -STEERING_ANGULAR_VELOCITY_LIMIT;
        vars_upper_bound[i] = STEERING_ANGULAR_VELOCITY_LIMIT;
    }
    // 等式制約
    Dvector constraints_lower_bound(n_constraints);
    Dvector constraints_upper_bound(n_constraints);

    for(int i=0;i<n_constraints;i++){
        constraints_lower_bound[i] = 0.0;
        constraints_upper_bound[i] = 0.0;
    }

    // t=0の設定
    constraints_lower_bound[x_start] = x;
    constraints_lower_bound[y_start] = y;
    constraints_lower_bound[yaw_start] = yaw;
    constraints_lower_bound[v_start] = v;
    constraints_lower_bound[omega_start] = omega;
    constraints_lower_bound[omega_r_start] = omega_r;
    constraints_lower_bound[omega_l_start] = omega_l;
    constraints_lower_bound[steer_r_start] = steer_r;
    constraints_lower_bound[steer_l_start] = steer_l;

    constraints_upper_bound[x_start] = x;
    constraints_upper_bound[y_start] = y;
    constraints_upper_bound[yaw_start] = yaw;
    constraints_upper_bound[v_start] = v;
    constraints_upper_bound[omega_start] = omega;
    constraints_upper_bound[omega_r_start] = omega_r;
    constraints_upper_bound[omega_l_start] = omega_l;
    constraints_upper_bound[steer_r_start] = steer_r;
    constraints_upper_bound[steer_l_start] = steer_l;

    //std::cout<<"===X0 bound==="<<std::endl;
    //std::cout<<constraints_lower_bound[x_start]<<std::endl;
    //std::cout<<constraints_upper_bound[x_start]<<std::endl;
    


    FG_eval fg_eval(ref_x, ref_y, ref_yaw);
    std::string options;
    options += "Integer print_level  0\n";
    // options += "Integer print_level  5\n";

    options += "Sparse  true                forward\n";
    options += "Sparse  true                reverse\n";

    options += "Numeric max_cpu_time                    0.5\n";
    options += "Numeric acceptable_tol                    1e-6\n";

    CppAD::ipopt::solve_result<Dvector> solution;

    // std::cout << "Optimization start" << std::endl;
    CppAD::ipopt::solve<Dvector, FG_eval>(
            options, vars, vars_lower_bound, vars_upper_bound, constraints_lower_bound,
            constraints_upper_bound, fg_eval, solution);


    std::cout << "===Optimization end===" << std::endl;
    ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
    //std::cout << solution.status << std::endl;
    if(ok) std::cout << "================================================================================================" << std::endl;
    else std::cout << "not ok" << std::endl;
    //std::cout << solution.x.size() << std::endl;

    auto cost = solution.obj_value;
    std::cout << "Cost " << std::endl;
    std::cout << cost << std::endl;


    if(ok){
        failure_count = 0;
        result.clear();
        // 何故か0だとうまく行かない
        result.push_back(solution.x[v_start+1]);
        result.push_back(solution.x[omega_start+1]);
        result.push_back(solution.x[steer_r_start+1]);
        result.push_back(solution.x[steer_l_start+1]);
        //予測軌道
        for(int i = 0; i < T-1; i++){
            result.push_back(solution.x[x_start+i+1]);
            result.push_back(solution.x[y_start+i+1]);
            result.push_back(solution.x[yaw_start+i+1]);
        }
#if 0
    }else{
        if(failure_count < T - 1){
            failure_count++;
        }
        result.push_back(solution.x[v_start+1+failure_count]);
        result.push_back(solution.x[omega_start+1+failure_count]);
        result.push_back(solution.x[steer_r_start+1+failure_count]);
        result.push_back(solution.x[steer_l_start+1+failure_count]);
        result[v_start] = result[v_start + failure_count];
        result[omega_start] = result[omega_start + failure_count];
        result[steer_r] = result[steer_r_start + failure_count];
        result[steer_l] = result[steer_l_start + failure_count];

        //予測軌道
        for(int i = failure_count; i < T-1; i++){
            /*
            result.push_back(solution.x[x_start+i+1]);
            result.push_back(solution.x[y_start+i+1]);
            result.push_back(solution.x[yaw_start+i+1]);
            */
        }
#endif
    }
    /*
    std::cout << "--- result ---" << std::endl;
    for(int i=0;i<result.size();i++){
        std::cout << result[i] << std::endl;
    }
    */

    return result;


}

FG_eval::FG_eval(Eigen::VectorXd ref_x, Eigen::VectorXd ref_y, Eigen::VectorXd ref_yaw)
{
    this->ref_x = ref_x;
    this->ref_y = ref_y;
    this->ref_yaw = ref_yaw;
}

void FG_eval::operator()(ADvector& fg, const ADvector& vars)
{
    std::cout << "FG_eval() start" << std::endl;
    // コスト
    fg[0] = 0;
#if 0
    // 初期条件
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + omega_start] = vars[omega_start];
    fg[1 + omega_r_start] = vars[omega_r_start];
    fg[1 + omega_l_start] = vars[omega_l_start];
    fg[1 + steer_r_start] = vars[steer_r_start];
    fg[1 + steer_l_start] = vars[steer_l_start]; 

    for(int i=0; i<T-1; i++){
        //t+1
        AD<double> x1 = vars[x_start + i + 1];
        AD<double> y1 = vars[y_start + i + 1];
        AD<double> yaw1 = vars[yaw_start + i + 1];
        AD<double> v1 = vars[v_start + i + 1];
        AD<double> omega1 = vars[omega_start + i + 1];
        AD<double> omega_r1 = vars[omega_r_start + i + 1];
        AD<double> omega_l1 = vars[omega_l_start + i + 1];
        AD<double> steer_r1 = vars[steer_r_start + i + 1];
        AD<double> steer_l1 = vars[steer_l_start + i + 1];
        //t
        AD<double> x0 = vars[x_start + i];
        AD<double> y0 = vars[y_start + i];
        AD<double> yaw0 = vars[yaw_start + i];
        AD<double> v0 = vars[v_start + i];
        AD<double> omega0 = vars[omega_start + i];
        AD<double> omega_r0 = vars[omega_r_start + i];
        AD<double> omega_l0 = vars[omega_l_start + i];
        AD<double> steer_r0 = vars[steer_r_start + i];
        AD<double> steer_l0 = vars[steer_l_start + i];
        //入力ホライゾンはt+1を考慮しない
        AD<double> domega_r0 = vars[domega_r_start + i];
        AD<double> domega_l0 = vars[domega_l_start + i];
        AD<double> dsteer_r0 = vars[dsteer_r_start + i];
        AD<double> dsteer_l0 = vars[dsteer_l_start + i];

        double epsilon = std::numeric_limits<double>::epsilon();  //0割確認用
        //base_linkを基準にした x-y 速度
        AD<double> vx1 = (WHEEL_RADIUS / 2) * (CppAD::cos(steer_l1) * omega_l1 + CppAD::cos(steer_r1) * omega_r1);
        AD<double> vy1 = (WHEEL_RADIUS / 2) * (CppAD::sin(steer_l1) * omega_l1 + CppAD::sin(steer_r1) * omega_r1);
        AD<double> vx0 = (WHEEL_RADIUS / 2) * (CppAD::cos(steer_l0) * omega_l0 + CppAD::cos(steer_r0) * omega_r0);
        AD<double> vy0 = (WHEEL_RADIUS / 2) * (CppAD::sin(steer_l0) * omega_l0 + CppAD::sin(steer_r0) * omega_r0);
        // AD<double> v = CppAD::sqrt( CppAD::pow(vx1, 2) + CppAD::pow(vy1, 2) );
        AD<double> v=CppAD::sqrt(vx1*vx1 + vy1*vy1);
        AD<double> R;

        if(CppAD::sin(CppAD::abs(steer_r1 - steer_l1)) < epsilon){
            R = 0.501;
            // std::cout << "R=TREAD(1) "<< i << std::endl;
        }
        else{
            AD<double> Rr = CppAD::sin(CppAD::abs(steer_l1)) / CppAD::sin(CppAD::abs(steer_r1 - steer_l1)) * TREAD;
            AD<double> Rl = CppAD::sin(CppAD::abs(steer_r1)) / CppAD::sin(CppAD::abs(steer_l1 - steer_r1)) * TREAD;
            R = CppAD::abs(Rr - Rl);
            if(R < epsilon){
                // std::cout << "R=TREAD(2) "<< i << std::endl;
                R=0.501;
            }
        }

        //コスト

        //制約
        fg[2 + omega_r_start + i] = omega_r1 - (omega_r0 + domega_r0 * DT);
        fg[2 + omega_l_start + i] = omega_l1 - (omega_l0 + domega_l0 * DT);
        fg[2 + steer_r_start + i] = steer_r1 - (steer_r0 + dsteer_r0 * DT);
        fg[2 + steer_l_start + i] = steer_l1 - (steer_l0 + dsteer_l0 * DT);
        fg[2 + v_start + i] = v1 - (WHEEL_RADIUS / 2.0) * (omega_r1 + omega_l1);
        fg[2 + omega_start + i] = omega1 - (WHEEL_RADIUS / TREAD) * (omega_r1 - omega_l1);
        fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * DT);
        fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * DT);
        fg[2 + yaw_start + i] = yaw1 - (yaw0 + omega0 * DT);


    }

#endif
#if 1
    for(int i=0;i<T-1;i++){
        // pathとの距離
        AD<double> v_direction = vars[yaw_start + i] + (vars[steer_r_start + i] + vars[steer_l_start + i])/2;
        fg[0] += (CppAD::pow(vars[x_start + i] - ref_x[i], 2) + CppAD::pow(vars[y_start + i] - ref_y[i], 2));
        // 向き
        // fg[0] += CppAD::pow(vars[yaw_start + i] - ref_yaw[i], 2);
        fg[0] += CppAD::pow(v_direction - ref_yaw[i], 2);
        // 速度
        fg[0] += CppAD::pow(VREF - vars[v_start + i], 2);
        //できればステアを切らない
        // fg[0] += CppAD::pow((vars[steer_r_start + i] + vars[steer_l_start + i]), 2);
        // 角加速度
        //fg[0] += 0.1 * CppAD::pow(vars[omega_start + i] - vars[omega_start + i+ 1], 2);
    }

    std::cout << "constrains start" << std::endl;
    //constraint
    //初期状態
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + yaw_start] = vars[yaw_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + omega_start] = vars[omega_start];
    fg[1 + omega_r_start] = vars[omega_r_start];
    fg[1 + omega_l_start] = vars[omega_l_start];
    fg[1 + steer_r_start] = vars[steer_r_start];
    fg[1 + steer_l_start] = vars[steer_l_start]; 

    std::cout << "constraints loop start" << std::endl;

    for(int i=0;i<T-1;i++){
        //t+1
        AD<double> x1 = vars[x_start + i + 1];
        AD<double> y1 = vars[y_start + i + 1];
        AD<double> yaw1 = vars[yaw_start + i + 1];
        AD<double> v1 = vars[v_start + i + 1];
        AD<double> omega1 = vars[omega_start + i + 1];
        AD<double> omega_r1 = vars[omega_r_start + i + 1];
        AD<double> omega_l1 = vars[omega_l_start + i + 1];
        AD<double> steer_r1 = vars[steer_r_start + i + 1];
        AD<double> steer_l1 = vars[steer_l_start + i + 1];
        //t
        AD<double> x0 = vars[x_start + i];
        AD<double> y0 = vars[y_start + i];
        AD<double> yaw0 = vars[yaw_start + i];
        AD<double> v0 = vars[v_start + i];
        AD<double> omega0 = vars[omega_start + i];
        AD<double> omega_r0 = vars[omega_r_start + i];
        AD<double> omega_l0 = vars[omega_l_start + i];
        AD<double> steer_r0 = vars[steer_r_start + i];
        AD<double> steer_l0 = vars[steer_l_start + i];
        //入力ホライゾンはt+1を考慮しない
        AD<double> domega_r0 = vars[domega_r_start + i];
        AD<double> domega_l0 = vars[domega_l_start + i];
        AD<double> dsteer_r0 = vars[dsteer_r_start + i];
        AD<double> dsteer_l0 = vars[dsteer_l_start + i];

        //制約
        double epsilon = std::numeric_limits<double>::epsilon();  //0割確認用

        //base_linkを基準にした x-y 速度
        AD<double> vx1 = (WHEEL_RADIUS / 2) * (CppAD::cos(steer_l1) * omega_l1 + CppAD::cos(steer_r1) * omega_r1);
        AD<double> vy1 = (WHEEL_RADIUS / 2) * (CppAD::sin(steer_l1) * omega_l1 + CppAD::sin(steer_r1) * omega_r1);
        AD<double> vx0 = (WHEEL_RADIUS / 2) * (CppAD::cos(steer_l0) * omega_l0 + CppAD::cos(steer_r0) * omega_r0);
        AD<double> vy0 = (WHEEL_RADIUS / 2) * (CppAD::sin(steer_l0) * omega_l0 + CppAD::sin(steer_r0) * omega_r0);
        // AD<double> v = CppAD::sqrt( CppAD::pow(vx1, 2) + CppAD::pow(vy1, 2) );
        AD<double> v=CppAD::sqrt(vx1*vx1 + vy1*vy1);
        AD<double> R;

        if(CppAD::sin(CppAD::abs(steer_r1 - steer_l1)) < epsilon){
            R = 0.501;
            // std::cout << "R=TREAD(1) "<< i << std::endl;
        }
        else{
            AD<double> Rr = CppAD::sin(CppAD::abs(steer_l1)) / CppAD::sin(CppAD::abs(steer_r1 - steer_l1)) * TREAD;
            AD<double> Rl = CppAD::sin(CppAD::abs(steer_r1)) / CppAD::sin(CppAD::abs(steer_l1 - steer_r1)) * TREAD;
            R = CppAD::abs(Rr - Rl);
            if(R < epsilon){
                // std::cout << "R=TREAD(2) "<< i << std::endl;
                R=0.501;
            }
        }


        fg[2 + omega_r_start + i] = omega_r1 - (omega_r0 + domega_r0 * DT);
        fg[2 + omega_l_start + i] = omega_l1 - (omega_l0 + domega_l0 * DT);
        fg[2 + steer_r_start + i] = steer_r1 - (steer_r0 + dsteer_r0 * DT);
        fg[2 + steer_l_start + i] = steer_l1 - (steer_l0 + dsteer_l0 * DT);
        fg[2 + v_start + i] = v1 - (WHEEL_RADIUS / 2.0) * (omega_r1 + omega_l1);
        // fg[2 + v_start + i] = v1 - v;
        // fg[2 + omega_start + i] = omega1 - (WHEEL_RADIUS / TREAD) * (omega_r1 - omega_l1);
        fg[2 + omega_start + i] = omega1 - (WHEEL_RADIUS / R ) * (omega_r1 - omega_l1);
        fg[2 + x_start + i] = x1 - (x0 + vx0 * CppAD::cos(yaw0) * DT - vy0 * CppAD::sin(yaw0) * DT);
        fg[2 + y_start + i] = y1 - (y0 + vx0 * CppAD::sin(yaw0) * DT + vy0 * CppAD::cos(yaw0) * DT);
        // fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(yaw0) * DT);
        // fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(yaw0) * DT);
        fg[2 + yaw_start + i] = yaw1 - (yaw0 + omega0 * DT);
#endif

    }
}

double get_distance(geometry_msgs::PoseStamped& pose0, geometry_msgs::PoseStamped& pose1)
{
    return sqrt((pose0.pose.position.x - pose1.pose.position.x) * (pose0.pose.position.x - pose1.pose.position.x) + (pose0.pose.position.y - pose1.pose.position.y) * (pose0.pose.position.y - pose1.pose.position.y));
}


void MPCPathTracker::calc_ref_trajectory()
{
    int length = path.poses.size();
    //current index
    double min_distace=1e10;
    for(int i=0; i<length; i++){
        double distanse=get_distance(path.poses[i], current_pose);
        if(min_distace > distanse){
            min_distace = distanse;
            current_index = i;
        }
    }

    //calcurate yaw
    std::vector<double> yaw(length);
    for (size_t i = 0; i < length - 1; i++)
    {
        double x1 = path.poses[i].pose.position.x;
        double y1 = path.poses[i].pose.position.y;
        double x2 = path.poses[i+1].pose.position.x;
        double y2 = path.poses[i+1].pose.position.y;
        yaw[i] = atan2(y2-y1, x2-x1);
        // std::cout<<"=== ATAN ==="<<std::endl;
        // std::cout<< yaw[i] <<std::endl;
    }
    

    //ref trajectory
    int m = VREF*DT / RESOLUTION + 1;  
    for(int i=0;i<T;i++){
        if(i*m<path.poses.size()){
            ref_x[i] = path.poses[current_index + i*m].pose.position.x;
            ref_y[i] = path.poses[current_index + i*m].pose.position.y;
            ref_yaw[i] = yaw[current_index + i*m];
        }else{
            ref_x[i] = path.poses[length - 1].pose.position.x;
            ref_y[i] = path.poses[length - 1].pose.position.y;
            ref_yaw[i] = yaw[length - 1];
        }
    }

    //show ref_path for debug
    geometry_msgs::PoseArray reference_path;
    reference_path.header.frame_id = WORLD_FRAME;
    for(int i=0; i<T; i++){
        geometry_msgs::Pose temp;
        temp.position.x = ref_x[i];
        temp.position.y = ref_y[i];
        temp.orientation = tf::createQuaternionMsgFromYaw(ref_yaw[i]);
        reference_path.poses.push_back(temp);
    }
    ref_path_pub.publish(reference_path);
}

void MPCPathTracker::process(void)
{
    bool transformed = false;
    geometry_msgs::PoseStamped pose;

    try{
        listener.lookupTransform(WORLD_FRAME, ROBOT_FRAME, ros::Time(0), _transform);
        tf::transformStampedTFToMsg(_transform, transform);
        current_pose.header = transform.header;
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.orientation = transform.transform.rotation;
        pose.header = current_pose.header;
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.orientation = transform.transform.rotation;
        transformed = true;

        // std::cout << "current x:"<< current_pose.pose.position.x << std::endl;
    }catch(tf::TransformException &ex){
        std::cout << ex.what() << std::endl;
    }


    if(!path.poses.empty() &&  !joint_state.name.empty() && transformed){
        std::cout << "=== diff drive mpc ===" << std::endl;
        ros::Time start_time = ros::Time::now();
        if(first_transform){
            last_time = ros::Time::now().toSec();
            first_transform = false;
            current_index=0;
        }else{
            double current_time = ros::Time::now().toSec();
            double dt = current_time - last_time;
            last_time = current_time;
            double dx = current_pose.pose.position.x - previous_pose.pose.position.x;
            double dy = current_pose.pose.position.y - previous_pose.pose.position.y;
            double dyaw = tf::getYaw(current_pose.pose.orientation) - tf::getYaw(previous_pose.pose.orientation);
            double v = sqrt(dx * dx + dy * dy) / dt;
            double omega = dyaw / dt;
            double omega_r = joint_state.velocity[8];
            double omega_l = joint_state.velocity[6];
            double steer_r = joint_state.position[7];
            double steer_l = joint_state.position[5];
            // std::cout << "steer r : " << steer_r*180/M_PI << std::endl;
            // std::cout << "steer l : " << steer_l*180/M_PI << std::endl;

            
            std::cout << "time : " << current_time << std::endl;
            Eigen::VectorXd state(9);
            state << current_pose.pose.position.x, current_pose.pose.position.y, tf::getYaw(current_pose.pose.orientation), v, omega, omega_r, omega_l, steer_r, steer_l;
            calc_ref_trajectory();
            //std::cout<<"===Current X==="<<std::endl;
            //std::cout<<current_pose.pose.position.x<<std::endl;

            result = mpc.solve(state, ref_x, ref_y, ref_yaw);

            //std::cout << "===KYORI===" << std::endl;
            //std::cout << CppAD::pow(result[2]-current_pose.pose.position.x, 2) + CppAD::pow(result[3]-current_pose.pose.position.y, 2)<< std::endl;
            // std::cout << "solved" << std::endl;
            geometry_msgs::Twist velocity;
            velocity.linear.x = result[0];
            velocity.angular.z = result[1];
            ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
            cmd_pos.steer_r= result[2];
            cmd_pos.steer_l= result[3];
            cmd_pos.fore=PITCH_OFFSET;
            cmd_pos.rear=PITCH_OFFSET;
            cmd_pos.roll=0.0;

            // geometry_msgs::Twist velocity;
            // velocity.linear.x = VREF;
            // velocity.linear.y = 0;
            // velocity.linear.z = 0;
            // velocity.angular.z = 0.0;
            // velocity.angular.x = 0.0;
            // velocity.angular.y = 0.0;
            // ccv_dynamixel_msgs::CmdPoseByRadian cmd_pos;
            // cmd_pos.steer_r= 0.0;
            // cmd_pos.steer_l= 0.0;
            // cmd_pos.fore=PITCH_OFFSET;
            // cmd_pos.rear=PITCH_OFFSET;
            // cmd_pos.roll=0.0;
            std::cout << velocity << std::endl;
            velocity_pub.publish(velocity);
            cmd_pos_pub.publish(cmd_pos);
            
            // // mpc表示
            geometry_msgs::PoseArray mpc_path;
            mpc_path.header.frame_id = WORLD_FRAME;
            double yaw0 = tf::getYaw(pose.pose.orientation);
            for(int i=0;i<T-1;i++){
                geometry_msgs::Pose temp;
                temp.position.x = result[4+3*i];
                temp.position.y = result[5+3*i];
                temp.orientation = tf::createQuaternionMsgFromYaw(result[6+3*i]);
                mpc_path.poses.push_back(temp);
            }

            path_pub.publish(mpc_path);
            // ~mpc表示
            path.poses.erase(path.poses.begin());
        }    
    }
    previous_pose = current_pose;

}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "ccv_mpc_steering");
    ros::NodeHandle local_nh("~");

    local_nh.param("HORIZON_T", T);
    local_nh.getParam("VREF", VREF);
    local_nh.getParam("MAX_ANGULAR_VELOCITY", MAX_ANGULAR_VELOCITY);
    local_nh.getParam("MAX_WHEEL_ANGULAR_ACCELERATION", WHEEL_ANGULAR_ACCELERATION_LIMIT);
    local_nh.getParam("MAX_WHEEL_ANGULAR_VELOCITY", WHEEL_ANGULAR_VELOCITY_LIMIT);
    local_nh.getParam("WHEEL_RADIUS", WHEEL_RADIUS);
    local_nh.getParam("TREAD", TREAD);
    local_nh.getParam("MAX_VELOCITY", MAX_VELOCITY);
    local_nh.param("/RESOLUTION", RESOLUTION);
    local_nh.param("/ROBOT_FRAME", ROBOT_FRAME);
    local_nh.param("/WORLD_FRAME", WORLD_FRAME);
    local_nh.param("/VELOCITY_TOPIC_NAME", VELOCITY_TOPIC_NAME);
    local_nh.param("/INTERMEDIATE_PATH_TOPIC_NAME", INTERMEDIATE_PATH_TOPIC_NAME);

    std::cout << "T: " << T << std::endl;
    std::cout << "VREF: " << VREF << std::endl;
    std::cout << "MAX_VELOCITY: " << MAX_VELOCITY << std::endl;
    std::cout << "MAX_ANGULAR_VELOCITY: " << MAX_ANGULAR_VELOCITY << std::endl;
    std::cout << "WHEEL_ANGULAR_VELOCITY_LIMIT: " << WHEEL_ANGULAR_VELOCITY_LIMIT << std::endl;
    std::cout << "WHEEL_ANGULAR_ACCELERATION_LIMIT: " << WHEEL_ANGULAR_ACCELERATION_LIMIT << std::endl;
    std::cout << "WHEEL_RADIUS: " << WHEEL_RADIUS << std::endl;
    std::cout << "TREAD: " << TREAD << std::endl;
    std::cout << "RESOLUTION: " << RESOLUTION << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME << std::endl;
    std::cout << "WORLD_FRAME: " << WORLD_FRAME << std::endl;
    std::cout << "VELOCITY_TOPIC_NAME: " << VELOCITY_TOPIC_NAME << std::endl;
    std::cout << "INTERMEDIATE_PATH_TOPIC_NAME: " << INTERMEDIATE_PATH_TOPIC_NAME << std::endl;

    MPCPathTracker mpc_path_tracker;
    ros::Rate loop_rate(HZ);

    while(ros::ok()){
        mpc_path_tracker.process();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
