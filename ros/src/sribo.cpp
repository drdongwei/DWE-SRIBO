 #include "sribo.h"


/**
 * @brief Construct a new dwe::dwe object
 */
dwe::dwe(int delta,int lopt,int k, Eigen::Vector3d& kappa,double paramP,double paramQ,double paramR,int order,bool enable_flow){

    // Initialize member param
    delta_= delta;
    lopt_ = lopt;
    k_ = k;
    kappa_ = kappa;

    paramP_ = paramP;
    paramQ_ = paramQ;
    paramR_ = paramR;
    enable_flow_ = enable_flow;

    order_ = order;

    count_j_ = k_ * lopt_ + delta_;
    LOOP_num_ = 1;
    
    // Initialize matrix
    MHE_flow_.setOnes(3,k_*lopt_+1);
    MHE_imu_.setOnes(3,k_*lopt_+1);
    MHE_imu_att_.setOnes(4,k_*lopt_+1);
    MHE_uwb_.setOnes(1,k_*lopt_+1);
    deltaTime_.setZero(1,k_*lopt_+1);
    MHE_height_.setZero(1,k_*lopt_+1);
    xt_.setOnes(6,1);
    xt_real_.setOnes(6,1);
    x1_.setOnes(6,k_*lopt_+1);

    VICON_inital_.setZero(3,1);

    VICON_pos01_.setOnes(3,k_*lopt_+1);
    VICON_pos02_.setOnes(3,k_*lopt_+1);
    VICON_pos_.setOnes(3,k_*lopt_+1);
    VICON_vel01_.setOnes(3,k_*lopt_+1);
    VICON_vel02_.setOnes(3,k_*lopt_+1);
    VICON_vel_.setOnes(3,k_*lopt_+1);
    VICON_imu_.setOnes(3,k_*lopt_+1);
    VICON_uwb_.setOnes(1,k_*lopt_+1);

    // Initialize matrix in solveonce
    Qp.setOnes(1,3);
    Qv.setOnes(1,3);
    Pp.setOnes(1,3);
    Pv.setOnes(1,3);

    tP << Pp,Pv;
    P = tP.asDiagonal()*paramP_;

    tQ << Qp,Qv;
    Q = tQ.asDiagonal()*paramQ_;

    num_state_= k_*lopt_;

    Ex.setZero(16*num_state_, 6*(num_state_+1));       
    Eu.setZero(16*num_state_, 13*num_state_);         
    W.setZero(16*num_state_, 16*num_state_);           
    ur.setZero(13*num_state_,1);             
    xxe.setZero(6,lopt_);     

    t_matrix.setZero(6*(num_state_+1),6*(order_+1)); 
    for (int j=1;j<=6;j++){
        t_matrix(j-1,(j-1)*(order_+1)) = 1;
    }
}

/**
 * @brief Destructor
 */
dwe::~dwe(){
}


/**
 * @brief initial data
 */
void dwe::Initialdataxt(){
    // resize xt_
    xt_.conservativeResize(6, LOOP_num_);
    xt_real_.conservativeResize(6, LOOP_num_);

    // initialize loc with vicon
    xt_(0,LOOP_num_-1) = VICON_pos_(0,lopt_);
    xt_(1,LOOP_num_-1) = VICON_pos_(1,lopt_);
    xt_(2,LOOP_num_-1) = VICON_pos_(2,lopt_);
    xt_(3,LOOP_num_-1) = VICON_vel_(0,lopt_);
    xt_(4,LOOP_num_-1) = VICON_vel_(1,lopt_);
    xt_(5,LOOP_num_-1) = VICON_vel_(2,lopt_);

    // initialize loc with vicon
    xt_real_(0,LOOP_num_-1) = VICON_pos01_(0,lopt_);
    xt_real_(1,LOOP_num_-1) = VICON_pos01_(1,lopt_);
    xt_real_(2,LOOP_num_-1) = VICON_pos01_(2,lopt_);
    xt_real_(3,LOOP_num_-1) = xt_(3,LOOP_num_-1);
    xt_real_(4,LOOP_num_-1) = xt_(4,LOOP_num_-1);
    xt_real_(5,LOOP_num_-1) = xt_(5,LOOP_num_-1);

} 

void dwe::flow_cb(const mavros_msgs::OpticalFlowRad::ConstPtr &msg){
    flag_init_imu_ = true;
    MHE_flow_.leftCols(lopt_)=MHE_flow_.rightCols(lopt_);
    MHE_flow_(0,lopt_)=msg->integrated_x;
    MHE_flow_(1,lopt_)=msg->integrated_y;
    flow_quality_ = msg->quality;

    MHE_height_.leftCols(lopt_)=MHE_height_.rightCols(lopt_);
    MHE_height_(0,lopt_) = - msg->distance + 0.22;  // ground

    // abrupt diff 
    if (abs(MHE_height_(0,lopt_) - MHE_height_(0,lopt_-1))>=0.5){
        MHE_height_(0,lopt_) = MHE_height_(0,lopt_-1);
    }

    // Calculate the dt_
    currentTime_= msg->header.stamp.sec + msg->header.stamp.nsec/1000000000.0;

    if (previousTime_==0) {
        // First message received, initialize previousTime
        previousTime_ = currentTime_;
    }
    deltaTime_.leftCols(lopt_)=deltaTime_.rightCols(lopt_);
    deltaTime_ (0,lopt_)= currentTime_ - previousTime_;
    previousTime_ = currentTime_;

    // get dt_
    if(LOOP_num_ <= lopt_+1){
        dt_ = deltaTime_.sum()/ LOOP_num_;
    }
    else{
        dt_ = deltaTime_.sum()/ deltaTime_.size();
    }

    // replace the initial data until takeoff
    if( !hasTakenOff_ ){
        Initialdataxt();

        ROS_INFO("Initialized data: x:%f y:%f z:%f", xt_real_(0, LOOP_num_ - 1), xt_real_(1, LOOP_num_ - 1), xt_real_(2, LOOP_num_ - 1));
        ROS_INFO("dt: %.4lf seconds", dt_);
        ROS_INFO("********************************");
    }
    else{
        flag_init_ok_ = true;
        ROS_INFO("---------------------------------");
    }

    if (dt_== 0) {
        // First message received, initialize previousTime
        MHE_flow_(2,lopt_)=0;
        return;
    }

    MHE_flow_(2,lopt_)=(MHE_height_(0,lopt_)-MHE_height_(0,int(lopt_/2)))/(dt_*(lopt_-int(lopt_/2)));
}


/**
 * @brief rotation matrx
 */
MatrixXd dwe::qua2mat(){
    double _w = MHE_imu_att_(0, lopt_);
    double _x = MHE_imu_att_(1, lopt_);
    double _y = MHE_imu_att_(2, lopt_);
    double _z = MHE_imu_att_(3, lopt_);
    Matrix3d R_qua2mat;
    R_qua2mat <<1-2*pow(_y,2)-2*pow(_z,2), 2*_x*_y-2*_z*_w, 2*_x*_z+2*_y*_w,
                2*_x*_y+2*_z*_w, 1-2*pow(_x,2)-2*pow(_z,2), 2*_y*_z-2*_x*_w,
                2*_x*_z-2*_y*_w, 2*_y*_z+2*_x*_w, 1-2*pow(_x,2)-2*pow(_y,2);
    return R_qua2mat;    
}


// Define the mean filter function
MatrixXd dwe::meanFilter(Eigen::MatrixXd data){
    int numRows = data.rows();
    int numCols = data.cols();

    Eigen::MatrixXd filteredMatrix(numRows, numCols);

    for (int row = 0; row < numRows; ++row) {
        double rowSum = 0.0;
        for (int col = 0; col < numCols; ++col) {
            rowSum += data(row, col);
        }
        double rowMean = rowSum / numCols;

        for (int col = 0; col < numCols; ++col) {
            filteredMatrix(row, col) = rowMean;
        }
    }

    return filteredMatrix;
}

/**
 * @brief callback from uav
 */
void dwe::imu_new_cb(const sensor_msgs::Imu::ConstPtr &msg){
    MHE_imu_.leftCols(lopt_)=MHE_imu_.rightCols(lopt_);
    MHE_imu_(0,lopt_)=msg->linear_acceleration.x * 9.8;
    MHE_imu_(1,lopt_)=msg->linear_acceleration.y * 9.8;
    MHE_imu_(2,lopt_)=msg->linear_acceleration.z * 9.8;

    MHE_imu_att_(0, lopt_) = msg->orientation.w;
    MHE_imu_att_(1, lopt_) = msg->orientation.x;
    MHE_imu_att_(2, lopt_) = msg->orientation.y;
    MHE_imu_att_(3, lopt_) = msg->orientation.z;

    MHE_imu_.col(lopt_) = qua2mat()* MHE_imu_.col(lopt_);
    MHE_imu_(2,lopt_)=MHE_imu_(2,lopt_) - 9.8;
}


void dwe::uwb_cb(const nlink_parser::LinktrackNodeframe3::ConstPtr &msg){
    MHE_uwb_.leftCols(lopt_)=MHE_uwb_.rightCols(lopt_);
    int uwb_size = msg->nodes.size();
    if (uwb_size == 1){
        MHE_uwb_(0,lopt_)=msg->nodes[0].dis;

        // abrupt diff
        if (abs(MHE_uwb_(0,lopt_) - MHE_uwb_(0,lopt_-1))>=0.5){
            MHE_uwb_ = meanFilter(MHE_uwb_);
        }
        return;
    }

    // uwb lost
    ROS_INFO("MHE_uwb lOST!");
    MHE_uwb_(0,lopt_)=MHE_uwb_(0,lopt_-1);

}

/**
 * @brief callback from vicon01 -- dynamic uav
 */
void dwe::pos_pp01_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){ // dynamic uav
    VICON_pos01_.leftCols(lopt_)=VICON_pos01_.rightCols(lopt_);
    VICON_pos01_(0,lopt_)=msg->pose.position.x;
    VICON_pos01_(1,lopt_)=msg->pose.position.y;
    VICON_pos01_(2,lopt_)=msg->pose.position.z;

    // VICON_pos_ : relative pos
    VICON_pos_ = VICON_pos01_ - VICON_pos02_;

    // VICON_uwb_ : relative dis
    VICON_uwb_.leftCols(lopt_)=VICON_uwb_.rightCols(lopt_);
    VICON_uwb_(0,lopt_) = sqrt(pow(VICON_pos01_(0,lopt_)-VICON_pos02_(0,lopt_),2)+pow(VICON_pos01_(1,lopt_)-VICON_pos02_(1,lopt_),2)+pow(VICON_pos01_(2,lopt_)-VICON_pos02_(2,lopt_),2));

    // takeoff condition dx or dy or dz >=0.1
    if(LOOP_num_ == count_j_ +1){
        double sum_x_ = VICON_pos01_.row(0).sum();
        double sum_y_ = VICON_pos01_.row(1).sum();
        double sum_z_ = VICON_pos01_.row(2).sum();
        VICON_inital_(0,0) = sum_x_ / VICON_pos01_.cols();
        VICON_inital_(1,0) = sum_y_ / VICON_pos01_.cols();
        VICON_inital_(2,0) = sum_z_ / VICON_pos01_.cols();
    }
    if ( !hasTakenOff_ && LOOP_num_ > count_j_ +1) {
        // if(VICON_pos01_(2, lopt_)>= 0.5){
        //     hasTakenOff_ = true;
        // }
        if(LOOP_num_>= 210){
            hasTakenOff_ = true;
        }
    }
}

void dwe::vel_pp01_cb(const nav_msgs::Odometry::ConstPtr &msg){       // dynamic uav
    VICON_vel01_.leftCols(lopt_)=VICON_vel01_.rightCols(lopt_);
    VICON_vel01_(0,lopt_)=msg->twist.twist.linear.x;
    VICON_vel01_(1,lopt_)=msg->twist.twist.linear.y;
    VICON_vel01_(2,lopt_)=msg->twist.twist.linear.z;

    // VICON_vel_ : relative vel
    VICON_vel_ = VICON_vel01_ - VICON_vel02_;

    // VICON_imu_ : relative acc 
    VICON_imu_.leftCols(lopt_)=VICON_imu_.rightCols(lopt_);
    if (dt_== 0) {
        // First message received, initialize previousTime
        VICON_imu_(0,lopt_)=0;
        VICON_imu_(1,lopt_)=0;
        VICON_imu_(2,lopt_)=0;
        
        return;
    }
    else{
        VICON_imu_(0,lopt_)=(VICON_vel_(0,lopt_)-VICON_vel_(0,lopt_-1))/dt_;
        VICON_imu_(1,lopt_)=(VICON_vel_(1,lopt_)-VICON_vel_(1,lopt_-1))/dt_;
        VICON_imu_(2,lopt_)=(VICON_vel_(2,lopt_)-VICON_vel_(2,lopt_-1))/dt_;
    }
}

/**
 * @brief callback from vicon02 -- static uav
 */
void dwe::pos_pp02_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){  // static uav
    VICON_pos02_.leftCols(lopt_)=VICON_pos02_.rightCols(lopt_);
    VICON_pos02_(0,lopt_)=msg->pose.position.x;
    VICON_pos02_(1,lopt_)=msg->pose.position.y;
    VICON_pos02_(2,lopt_)=msg->pose.position.z;

}

void dwe::vel_pp02_cb(const nav_msgs::Odometry::ConstPtr &msg){        // static uav
    VICON_vel02_.leftCols(lopt_)=VICON_vel02_.rightCols(lopt_);
    VICON_vel02_(0,lopt_)=msg->twist.twist.linear.x;
    VICON_vel02_(1,lopt_)=msg->twist.twist.linear.y;
    VICON_vel02_(2,lopt_)=msg->twist.twist.linear.z;
}

/**
 * @brief solveonce
 */

void dwe::solveonce()
{
    ROS_INFO("dt: %.4lf seconds", dt_);

    // Initialize matrix in solveonce
    x1_= xt_.rightCols(k_*lopt_+1);

    //imu from uav
    MatrixXd imu_sol = MHE_imu_.leftCols(k_*lopt_);
    MatrixXd flow_sol = MHE_flow_;
    MatrixXd uwb_sol = MHE_uwb_;

    if (flow_quality_!=255 || !enable_flow_)
    {
        paramR_ = 0;
    }
    else{
        paramR_ = 1;
    }

    tR << 1,paramR_,paramR_,paramR_*0.1; //z
    R = tR.asDiagonal();

    A << 1, 0, 0, dt_, 0, 0,
                0, 1, 0, 0, dt_, 0,
                0, 0, 1, 0, 0, dt_,
                0, 0, 0, 1-kappa_(0)*dt_, 0, 0,
                0, 0, 0, 0, 1-kappa_(1)*dt_, 0,
                0, 0, 0, 0, 0, 1-kappa_(2)*dt_;

    B << 0.5*pow(dt_,2), 0, 0,
                    0, 0.5*pow(dt_,2), 0,
                    0, 0, 0.5*pow(dt_,2),
                    dt_, 0,  0,  
                    0,  dt_, 0,
                    0,  0, dt_; 

    AA = A;
    for (int i = 1; i < k_; i++) {
        AA = AA*A;
    }

    for (int i=1;i<=num_state_;i++){

        Ex.block((i-1)*6,(i-1)*6,6,6)=Eigen::MatrixXd::Identity(6,6);

        Ex.block((i-1)*6+6*num_state_,(i-1)*6,6,6) = -AA;
        Ex.block((i-1)*6+6*num_state_,(i)*6,6,6) = Eigen::MatrixXd::Identity(6,6);

        Eigen::MatrixXd x0_loc=x1_.topRows(3).col(i);
        double rho=x0_loc.norm();
        Eigen::MatrixXd C(4,6);
        C.setZero(4,6);
        C.block(0,0,1,3) = x0_loc.transpose()/rho;
        C.block(1,3,3,3) = Eigen::MatrixXd::Identity(3,3);

        Ex.block(num_state_*6+6*num_state_+i*4-4,(i)*6,4,6) = C;

        Eu.block((i-1)*6,(i-1)*6,6,6) = Eigen::MatrixXd::Identity(6,6);

        for(int j=1;j<=k_;j++){
            Eigen::Matrix<double, 6, 6> Akj;
            Akj.setIdentity(6,6);
            int kj=k_-j;
            for (int q=0;q<kj;q++){
                Akj=Akj*A;
            }
            Eu.block(6*num_state_+(i-1)*6,6*num_state_+(i-1)*3*k_+(j-1)*3,6,3) = Akj*B;
        }

        Eu.block((num_state_)*12+4*i-4,num_state_*3*k_+6*num_state_+4*i-4,4,4) = Eigen::MatrixXd::Identity(4,4);
        ur(3*k_*num_state_+6*num_state_+i*4-4,0) = uwb_sol(0,i*k_);
        ur.block(3*k_*num_state_+6*num_state_+i*4-3,0,3,1) = flow_sol.col(i*k_);

        // W: 320x320 covariance matrix, diag param is set as P,Q,R
        W.block((i-1)*6,(i-1)*6,6,6) = P;
        W.block(6*num_state_+(i-1)*6,6*num_state_+(i-1)*6,6,6) = Q;
        W.block(6*num_state_+num_state_*6+i*4-4,6*num_state_+num_state_*6+i*4-4,4,4) = R;

        Eigen::MatrixXd tt(1,order_+1);
        for(int j=1;j<=(order_+1);j++){
            tt(0,j-1) = pow(dt_*i*k_,j-1);
        }
        for(int j=1;j<=6;j++){
            t_matrix.block((i)*6+j-1,(j-1)*(order_+1),1,order_+1) = tt;
        }
    }

    xtt=x1_.rightCols(x1_.cols()-1);

    for(int j=1;j<=xtt.cols();j=j+1){
        ur.block(0+xtt.rows()*(j-1),0,xtt.rows(),1) = xtt.col(j-1);
    }

    for(int j=1;j<=imu_sol.cols();j=j+1){
        ur.block(xtt.cols()*xtt.rows()+imu_sol.rows()*(j-1),0,imu_sol.rows(),1) = imu_sol.col(j-1);
    }

    Ex_new = Ex * t_matrix; 
    ar = ((Ex_new.transpose()*W*Ex_new).inverse())*(Ex_new.transpose())*W*Eu*ur;

    xxt_new = t_matrix * ar;

    for(int i=1;i<=num_state_;i++){
        xxe.block(0,i-1,6,1) = xxt_new.block(6*i,0,6,1);
    }

    xt_.conservativeResize(6, LOOP_num_);
    xt_.col(LOOP_num_-1)=xxe.rightCols(1);

    xt_real_.conservativeResize(6, LOOP_num_);
    xt_real_(0,LOOP_num_-1) = xt_(0,LOOP_num_-1) + VICON_pos02_(0,0);
    xt_real_(1,LOOP_num_-1) = xt_(1,LOOP_num_-1) + VICON_pos02_(1,0);
    xt_real_(2,LOOP_num_-1) = xt_(2,LOOP_num_-1) + VICON_pos02_(2,0);
    xt_real_(3,LOOP_num_-1) = xt_(3,LOOP_num_-1);
    xt_real_(4,LOOP_num_-1) = xt_(4,LOOP_num_-1);
    xt_real_(5,LOOP_num_-1) = xt_(5,LOOP_num_-1);

}
