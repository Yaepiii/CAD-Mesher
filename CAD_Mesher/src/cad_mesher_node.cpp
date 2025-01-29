/***
 * This file run a CAD-Mesher node.
 * Please use this abbreviation dictionary to help you understand our code:
 * param: parameter
 * g_data: global data
 * trf: transformation, 6dof
 * odom: odometry
 * dir: direction, the coordinate that serve as the prediction in gaussian
    process function, like z in z = f(x, y).
 * num: number
 * Idx: index
 * 3Dir: inside one cell, there can be 3 different gaussian process function
    to model complex local surfaces, they have different prediction directions,
    that is, x, y, or z. Functions without "3Dir" means they only allow one
    gaussian process function inside one cell, like paper [3].
 * glb: global map, means in the world frame
 * now: means the current scan
 * avg: average
 * thr: threshold
 * posi: position
 * ary: array
 *
    */

# include "cad_mesher_node.h"
Parameter param;//parameters
Log g_data;//global variables
std::atomic<bool> CAD_Mesher::abort_(false);

Log::Log(){
    log_length =  param.max_steps;
    num_cells_now = num_cells_glb = num_cells_new =
    time_cost = time_cost_draw_map = time_find_overlap = time_find_overlap_points = time_pub_odom =
    time_get_pcl = time_gp = time_compute_rt = time_update = time_down_sample = time_dynamic_removal = time_sliding_window =
    not_a_surface_cell_num = overlap_point_num = gp_times = rg_times =
        Eigen::MatrixXd::Zero(1, log_length);
    pose = Eigen::MatrixXd::Zero(3, log_length);
    //first_transf = lidar_install_transf = grt_first_transf = transf_odom_now = transf_odom_last = Eigen::MatrixXd::Identity(4,4);
    path.header.frame_id      = "map";
    path_odom.header.frame_id = "map";
    path_grt.header.frame_id  = "map";
    pcl_raw_accumulated.height = 1;
    pcl_raw_accumulated.width = 0;
    g << 0.0, 0.0, 9.82;

    pcl_submap_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_static_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_dynamic_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_new_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    pcl_pre_ptr.reset(new pcl::PointCloud<pcl::PointXYZ>());
    spaciousness = 5;

    q_wmap_wodom = Eigen::Quaterniond(1, 0, 0, 0);
    t_wmap_wodom = Eigen::Vector3d(0, 0, 0);
}
void Log::extendLog(){
    if(step >= log_length){ // 10000
        int extend_length = 5000;
        extendEigen1dVector(num_cells_now, extend_length);
        extendEigen1dVector(num_cells_glb, extend_length);
        extendEigen1dVector(num_cells_new, extend_length);

        extendEigen1dVector(time_cost, extend_length);
        extendEigen1dVector(time_cost_draw_map, extend_length);
        extendEigen1dVector(time_find_overlap, extend_length);
        extendEigen1dVector(time_find_overlap_points, extend_length);
        extendEigen1dVector(time_pub_odom, extend_length);
        extendEigen1dVector(time_get_pcl, extend_length);
        extendEigen1dVector(time_gp, extend_length);
        extendEigen1dVector(time_compute_rt, extend_length);
        extendEigen1dVector(time_update, extend_length);
        extendEigen1dVector(time_down_sample, extend_length);
        extendEigen1dVector(time_dynamic_removal, extend_length);
        extendEigen1dVector(time_sliding_window, extend_length);

        extendEigen1dVector(not_a_surface_cell_num, extend_length);
        extendEigen1dVector(overlap_point_num, extend_length);
        extendEigen1dVector(gp_times, extend_length);
        extendEigen1dVector(rg_times, extend_length);

        pose.conservativeResize(Eigen::NoChange_t(1), pose.cols() + extend_length);
        pose.rightCols(extend_length).fill(0);

        log_length += extend_length;
    }
}
void Log::initLog(std::string & log_file_path){
    //open file to record log
    ROS_DEBUG("Log::initLog");
    //init file path
    std::cout << "Try to save report in: " << log_file_path << std::endl;
    auto now = std::time(nullptr);
    char buf[sizeof("YYYY-MM-DD-HH:MM:SS")];
    std::string run_time (buf, buf + std::strftime(buf, sizeof(buf), "%F-%T", std::gmtime(&now)));
    std::string log_file_path_report   = log_file_path + "_" + run_time + "_report.txt";
    std::string log_file_path_odom     = log_file_path + "_" + run_time + "_path_odom.txt";
    std::string log_file_path_grt      = log_file_path + "_" + run_time + "_path_grt.txt";
    log_file_path_GP_map_points = log_file_path + "_" + run_time + "_map_point.pcd";
    log_file_path_raw_pcl       = log_file_path + "_" + run_time + "_raw_pcl.pcd";
    log_file_path.erase(log_file_path.end() - 5, log_file_path.end());
    //std::cout << log_file_path << std::endl;
    std::string log_file_path_path          = log_file_path + param.seq.substr(1, 2) + "_pred.txt";
    //open
    file_loc_report_wrt.open (log_file_path_report, std::ios::out);
    if(!file_loc_report_wrt){
        ROS_WARN("Can not open Report file");
    }
    file_loc_path_wrt.open(log_file_path_path, std::ios::out);
    if(!file_loc_path_wrt){
        ROS_WARN("Can not open Path file");
    }
    if(param.use_odom_prior){
        file_loc_path_grt_wrt.open(log_file_path_grt, std::ios::out);
        if(!file_loc_path_odom_wrt){
            ROS_WARN("Can not open Path Odom file");
        }
    }
}
void Log::saveResult(double code_whole_time, const PointMatrix & map_glb_point_filtered, Map &  map_glb){
    //save report
    ROS_DEBUG("saveResult");
    std::cout<<"Saving result" <<std::endl;
    double time_sum_all_step;
    time_sum_all_step = time_cost.leftCols(step).sum();//unit: ms, last step is not counted
    double sum_now_cells = num_cells_now.leftCols(step).sum();
    double raw_point_num_before_voxel_filter = -1;
    double steps_include_first = step+1 ;
    //print
    std::cout
            << "TIME ALL STEPS: " << (time_sum_all_step)/1000.0 << " s" << std::endl
            << "TRJ LENGTH    : " << trajectory_length << std::endl;
    std::cout << "average_time" << "\n"
              << "time_average        : " << time_cost.leftCols(step+1).sum()/steps_include_first << "\n"
              << "time_dynamic_removal: " << time_dynamic_removal.leftCols(step+1).sum()/steps_include_first << "\n"
              << "time_sliding_window : " << time_sliding_window.leftCols(step+1).sum()/steps_include_first << "\n"
              << "time_get_pcl        : " << time_get_pcl.leftCols(step+1).sum()/steps_include_first << "\n"
              << "time_down_sample    : " << time_down_sample.leftCols(step+1).sum()/steps_include_first << "\n"
              << "time_overlap_region : " << time_find_overlap.leftCols(step+1).sum()/steps_include_first <<"\n"
              << "time_overlap_points : " << time_find_overlap_points.leftCols(step+1).sum()/steps_include_first <<"\n"
              << "time_gp             : " << time_gp.leftCols(step+1).sum()/steps_include_first <<"\n"
              << "time_compute_rt     : " << time_compute_rt.leftCols(step+1).sum()/steps_include_first <<"\n"
              << "time_update         : " << time_update.leftCols(step+1).sum()/steps_include_first <<"\n"
              //<< "time_pub_odom      : " << time_pub_odom.leftCols(step+1).sum()/steps_include_first <<"\n"
              << "time_cost_draw_map  : " << time_cost_draw_map.leftCols(step+1).sum()/steps_include_first <<"\n" ;
    //save report
    if(file_loc_report_wrt){
        file_loc_report_wrt
            << "step: " << step << "\n"
            << "TIME PER STEP:  " <<(time_sum_all_step)/(step-1) << " ms" << "\n"
            << "TIME ALL STEPS: " <<(time_sum_all_step)/1000.0 << " s" << "\n"
            << "TIME TOTAL RUN: " <<(code_whole_time)/1000.0 << " s" << "\n"
            << "NOW_CELL AVERAGE: " << (sum_now_cells) / step << "\n"
            << "DISPLACEMENT  : " << sqrt(pow(pose(0,step),2)+pow(pose(1,step),2)+pow(pose(2,step),2)) << "\n"

            << "time_cost: " << time_cost.leftCols(step+1).sum()/steps_include_first << "\n" << time_cost.leftCols(step+1) << "\n"
            << "time_dynamic_removal: " << time_dynamic_removal.leftCols(step+1).sum()/steps_include_first << "\n" << time_dynamic_removal.leftCols(step+1) << "\n"
            << "time_sliding_window: " << time_sliding_window.leftCols(step+1).sum()/steps_include_first << "\n" << time_sliding_window.leftCols(step+1) << "\n"
            << "time_get_pcl: " << time_get_pcl.leftCols(step+1).sum()/steps_include_first << "\n" << time_get_pcl.leftCols(step+1) << "\n"
            << "time_down_sample: " << time_down_sample.leftCols(step+1).sum()/steps_include_first << "\n" << time_down_sample.leftCols(step+1) << "\n"
            << "time_gp: " << time_gp.leftCols(step+1).sum()/steps_include_first << "\n" << time_gp.leftCols(step+1) << "\n"
            << "time_find_overlap: " << time_find_overlap.leftCols(step+1).sum()/steps_include_first << "\n" << time_find_overlap.leftCols(step+1) << "\n"
            << "time_find_overlap_points: " << time_find_overlap_points.leftCols(step+1).sum()/steps_include_first << "\n" << time_find_overlap_points.leftCols(step+1) << "\n"
            << "time_compute_rt: " << time_compute_rt.leftCols(step+1).sum()/steps_include_first << "\n" << time_compute_rt.leftCols(step+1) << "\n"
            << "time_update: " << time_update.leftCols(step+1).sum()/steps_include_first << "\n" << time_update.leftCols(step+1) << "\n"
            << "time_cost_draw_map: " << time_cost_draw_map.leftCols(step+1).sum()/steps_include_first << "\n" << time_cost_draw_map.leftCols(step+1) << "\n"

            << "num_cells_glb:" << num_cells_glb.leftCols(step + 1).sum() / steps_include_first << "\n" << num_cells_glb.leftCols(step + 1) << "\n"
            << "num_cells_now:" << num_cells_now.leftCols(step + 1).sum() / steps_include_first << "\n" << num_cells_now.leftCols(step + 1) << "\n"
            << "num_cells_new:" << num_cells_new.leftCols(step + 1).sum() / steps_include_first << "\n" << num_cells_new.leftCols(step + 1) << "\n"
            << "not_a_surface_cell_num:" << not_a_surface_cell_num.leftCols(step + 1).sum() / steps_include_first << "\n" << not_a_surface_cell_num.leftCols(step + 1) << "\n"
            << "gp_times:" << gp_times.leftCols(step + 1).sum() / steps_include_first << "\n" << gp_times.leftCols(step + 1) << "\n"
            << "overlap_point_num:" << overlap_point_num.leftCols(step+1).sum()/steps_include_first <<"\n" << overlap_point_num.leftCols(step+1) << "\n"
            << "register_times:" << rg_times.leftCols(step+1).sum()/steps_include_first <<"\n" << rg_times.leftCols(step+1) << "\n"
            << "pose: " << "\n" << pose.leftCols(step+1) << "\n"
            << std::endl;
        file_loc_report_wrt << "raw_point_num_before_voxel_filter: "<<raw_point_num_before_voxel_filter;
        std::cout << "Report saved in: " << param.file_loc_report << std::endl;
        file_loc_report_wrt.close();
    }
    else{
        std::cout <<"Result not saved, did you creat the folder?" << std::endl;
    }
    //save path
    savePath2TxtKitti(file_loc_path_wrt, path);
    //save mesh map
    if(param.save_mesh_map){
        //at the end, publish global mesh map, may cost seconds
        std::string file_loc_mesh_ply = param.file_loc_report + param.seq + "_mesh.ply";
        if(map_glb.outputMeshAsPly(file_loc_mesh_ply, map_glb.mesh_msg)){
            std::cout << "Mesh map saved in: " << file_loc_mesh_ply << std::endl;
        }
    }
    //save raw pcl
    if(param.save_raw_point_clouds){
        double voxel = 0.3;
        raw_point_num_before_voxel_filter = pcl_raw_accumulated.width;
        pcl::PointCloud<pcl::PointXYZ> raw_pcl_store;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointer(new pcl::PointCloud<pcl::PointXYZ>);
        cloudPointer = pcl_raw_accumulated.makeShared();
        raw_pcl_store = pclVoxelFilter(cloudPointer, voxel);
        pcl::io::savePCDFileASCII(log_file_path_raw_pcl, raw_pcl_store);
    }
    g_data.file_loc_path_wrt.close();
    g_data.file_loc_path_grt_wrt.close();
}
void Log::pose_print(ros::Publisher & cloud_pub) const{//ok
    sensor_msgs::PointCloud cloud1 = matrix3DtoPclMsg(pose, step);
    cloud_pub.publish(cloud1);
}
void Log::savePath2TxtKitti(std::ofstream & file_out, nav_msgs::Path & path_msg){
    //write kitti format pose txt file, from path_msg, before write, transform to the camera frame using provided extrinsic
    if(file_out){
        for(int i = 0; i<= step-1; i++){
            Transf T_rectified;
            Transf T_velo2cam = Eigen::Matrix4d::Identity();
            if(param.seq == "/00" || param.seq == "/01" || param.seq == "/02" || param.seq == "/13" || param.seq == "/14" ||
               param.seq == "/15" || param.seq == "/16" || param.seq == "/17" || param.seq == "/18" || param.seq == "/19" ||
               param.seq == "/20" || param.seq == "/21" ) {
                T_velo2cam << 4.276802385584e-04,-9.999672484946e-01,-8.084491683471e-03,-1.198459927713e-02,
                              -7.210626507497e-03,8.081198471645e-03,-9.999413164504e-01,-5.403984729748e-02,
                              9.999738645903e-01,4.859485810390e-04,-7.206933692422e-03,-2.921968648686e-01,
                              0.0,0.0,0.0,1.0;
            }
            else if(param.seq == "/03") {
                T_velo2cam << 2.347736981471e-04, -9.999441545438e-01, -1.056347781105e-02, -2.796816941295e-03,
                              1.044940741659e-02, 1.056535364138e-02, -9.998895741176e-01, -7.510879138296e-02,
                              9.999453885620e-01, 1.243653783865e-04, 1.045130299567e-02, -2.721327964059e-01,
                              0.0,0.0,0.0,1.0;
            }
            else if(param.seq == "/04" || param.seq == "/05" || param.seq == "/06" || param.seq == "/07" || param.seq == "/08" ||
                    param.seq == "/09" || param.seq == "/10" || param.seq == "/11" || param.seq == "/12" ) {
                T_velo2cam << -1.857739385241e-03, -9.999659513510e-01, -8.039975204516e-03, -4.784029760483e-03,
                              -6.481465826011e-03, 8.051860151134e-03, -9.999466081774e-01, -7.337429464231e-02,
                              9.999773098287e-01, -1.805528627661e-03, -6.496203536139e-03, -3.339968064433e-01,
                              0.0,0.0,0.0,1.0;
            }
            T_rectified = T_velo2cam * T_seq[i] * T_velo2cam.inverse();
            //T_rectified = T_seq[i];
            file_out << T_rectified(0, 0) << " " << T_rectified(0, 1) << " " << T_rectified(0, 2) << " " << T_rectified(0, 3) << " "
                     << T_rectified(1, 0) << " " << T_rectified(1, 1) << " " << T_rectified(1, 2) << " " << T_rectified(1, 3) << " "
                     << T_rectified(2, 0) << " " << T_rectified(2, 1) << " " << T_rectified(2, 2) << " " << T_rectified(2, 3)
                     << "\n";
        }
    }
    else{
        std::cout<<"Can not open file: " <<"\n";
    }
}
void Log::accumulateRawPoint(pcl::PointCloud<pcl::PointXYZ> pcl_raw, Transf& transf_this_step){
    //accumulate raw point cloud in the world frame
    //std::cout<<"num_point_all_raw_point last: \n"<<pcl_raw_accumulated.width * pcl_raw_accumulated.height<<"\n";//std::endl;
    static int skip_i = 0;
    int skip_step = 1;
    skip_i ++;
    if(skip_i % skip_step == 0){
        pcl::transformPointCloud(pcl_raw, pcl_raw, transf_this_step.cast<float>());
        pcl_raw_accumulated = pcl_raw_accumulated + pcl_raw;
        std::cout << "num_point_all_raw_point: " << pcl_raw_accumulated.width * pcl_raw_accumulated.height << "\n";//std::endl;
    }
}
Transf Log::initFirstTransf(){
    // get the first initial guess transform, can use it to correct the whole map when the sensor is not horizontal installed
    // usually grt and imu odometry are gravity aligned, so we can use them to correct the whole map
    ROS_DEBUG("initFirstTransf");

    Transf first_odom = Eigen::MatrixXd::Identity(4, 4);
    if(step != 0){
        ROS_ERROR("Incorrect use of initFirstTransf!");
        return first_odom;
    }
    //two ways

    //2, use first odom
    else if(param.use_odom_prior){
        bool first = true;
        while(imu_msg_buf.empty() && odometry_msg_buf.empty() && ros::ok()){
            ros::spinOnce();//wait for first imu
            if(first) ROS_INFO("Waiting for first odom or imu...");
            usleep(100);
            first = false;
        }
        g_data.transf_odom_now = PoseWithCovariance2transf(g_data.odometry_msg_buf.back()->pose); //里程计类型转化为Eigen::Matrix4d类型
        transf_odom_now = g_data.transf_odom_now;
        // imu or odometry callback function will update transf_odom_now
    }
    
    //3, give a manual T
    else{transf_odom_now = createTrans(param.correction_x, param.correction_y, param.correction_z,
                                   param.correction_roll_degree, param.correction_pitch_degree,
                                   param.correction_yaw_degree);
    }

    if_first_trans_init = true;//stop update grt_first_transf

    //set odom
    g_data.recordPoseToPath(Odom, g_data.transf_odom_now);//save path

    //set imu start point
    imu_Bg.fill(0);
    imu_Ba.fill(0);
    imu_pos = transf_odom_now.block(0, 3, 3, 1);
    imu_rot = transf_odom_now.block(0, 0, 3, 3);
    imu_vel.fill(0);
    imu_init = true;

    if(param.dataset == 3){
        //newer college dataset
        transf_odom_now(0, 0) = 5.925493285036220747e-01;
        transf_odom_now(0, 1) = -8.038419275143061649e-01;
        transf_odom_now(0, 2) = 5.218676416200035417e-02;
        transf_odom_now(0, 3) = -2.422443415414985424e-01;
        transf_odom_now(1, 0) = 8.017167514002809803e-01;
        transf_odom_now(1, 1) = 5.948020209102693467e-01;
        transf_odom_now(1, 2) = 5.882863457495644127e-02;
        transf_odom_now(1, 3) = 3.667865561670570873e+00;
        transf_odom_now(2, 0) = -7.832971094540422397e-02;
        transf_odom_now(2, 1) = 6.980134849334420320e-03;
        transf_odom_now(2, 2) = 9.969030746023688216e-01;
        transf_odom_now(2, 3) = 6.809443654823238434e-01;
    }
    //result
    transf_odom_last = transf_odom_now;//once read
    std::cout << "first_transf:\n" << transf_odom_now << "\n";
    return transf_odom_now;
}
void Log::updatePose(Transf & now_slam_transf){
    //After scan registration, if use imu, here try to compensate the bias of imu. If no imu, just save transformation
    ROS_DEBUG("updatePose");
    static Transf last_imu_transf, now_imu_transf, last_slam_transf;
    static double bias_sum_x, bias_sum_y, update_count;
    static double last_time;
    static bool is_first = true;

    static std::queue<double> ba_x;
    static std::queue<double> ba_y;
    int fix_lag = 1000;//50Hz = 20s

    //store Tsep
    T_seq.push_back(now_slam_transf);
    Point tmp_pose;
    tmp_pose = trans3Dpoint(0, 0, 0, now_slam_transf);
    pose.col(step) = tmp_pose;
    if(step > 0){
        trajectory_length += sqrt(pow(tmp_pose(0, 0) - pose(0, step - 1), 2) +
                pow(tmp_pose(1, 0) - pose(1, step - 1), 2) +
                pow(tmp_pose(2, 0) - pose(2, step - 1), 2));
    }

    recordPoseToPath(Slam, now_slam_transf);
    //save path and grt_path to txt
    //savePathEveryStep2Txt(file_loc_path_gdt_wrt, path_grt);
    //savePathEveryStep2Txt(file_loc_path_wrt, path);

    //update imu translation and imu bias
    if(param.imu_feedback){

        now_imu_transf << imu_rot, imu_pos, 0, 0, 0, 1;
        if(is_first){
            last_time  = ros::Time::now().toSec();
            last_time  = g_data.pcl_msg_buff.header.stamp.toSec();
            //last_time = imu_msg_buf.back()->header.timestamp.toSec();
            last_imu_transf = now_imu_transf;
            last_slam_transf = now_slam_transf;
            bias_sum_x = bias_sum_y = update_count = 0;
            is_first = false;
            imu_vel.fill(0);
        }
        else{
            //update imu transformPoints
            imu_pos(0, 0) = tmp_pose(0, 0);
            imu_pos(1, 0) = tmp_pose(1, 0);
            //imu_vel.fill(0);

            //save imu pose every time update and predict
            Point tmp_point;
            tmp_point << g_data.imu_pos.topRows(2), 0;
            g_data.pose_imu.addPoint(tmp_point);

            //update imu bias
            double now_time  = g_data.pcl_msg_buff.header.stamp.toSec();
            double dt = now_time - last_time;
            if(dt == 0){
                dt = 0.1;
            }
            double ds_slam_x = now_slam_transf(0, 3) - last_slam_transf(0, 3),
                    ds_slam_y = now_slam_transf(1, 3) - last_slam_transf(1, 3);
            double ds_imu_x  = now_imu_transf(0, 3) - last_imu_transf(0, 3),
                    ds_imu_y  = now_imu_transf(1, 3) - last_imu_transf(1, 3);
            double ds_x = ds_slam_x - ds_imu_x,
                    ds_y = ds_slam_y - ds_imu_y;

            //imu_vel(0, 0) = ds_slam_x / dt;
            //imu_vel(1, 0) = ds_slam_y / dt;

            bool fix_lag_ba_estimate = false;//false true
            if(fix_lag_ba_estimate){
                if(ba_x.size() < fix_lag){
                    ba_x.push(- ds_x * 2 / pow(dt, 2));
                    ba_y.push(- ds_y * 2 / pow(dt, 2));
                    bias_sum_x += ba_x.back();
                    bias_sum_y += ba_y.back();
                }
                else{
                    while(ba_x.size() >= fix_lag){
                        bias_sum_x -= ba_x.front();
                        bias_sum_y -= ba_y.front();
                        ba_x.pop();
                        ba_y.pop();
                    }
                    ba_x.push(- ds_x * 2 / pow(dt, 2));
                    ba_y.push(- ds_y * 2 / pow(dt, 2));
                    bias_sum_x += ba_x.back();
                    bias_sum_y += ba_y.back();
                    imu_Ba(0, 0) = bias_sum_x / ba_x.size();
                    imu_Ba(1, 0) = bias_sum_y / ba_y.size();
                }
                std::cout << "bias: x " << imu_Ba(0, 0) << "   y: " << imu_Ba(1, 0) << "   dt: " << dt << "\n";
            }
            else{
                update_count ++;
                bias_sum_x += - ds_x * 2 / pow(dt, 2);
                bias_sum_y += - ds_y * 2 / pow(dt, 2);
                imu_Ba(0, 0) = bias_sum_x / update_count;
                imu_Ba(1, 0) = bias_sum_y / update_count;
                std::cout << "bias: x " << imu_Ba(0, 0) << "   y: " << imu_Ba(1, 0) << "   dt: " << dt << "\n";
            }

            last_time = now_time;
            last_imu_transf = now_imu_transf;
            last_slam_transf = now_slam_transf;
        }
    }
}

void Parameter::initParameter(ros::NodeHandle & nh){
    //<!--  read param  -->
    nh.param("cad_mesher/use_odom_prior", use_odom_prior, false);
    nh.param("cad_mesher/imu_feedback", imu_feedback, false);//? TO DO
    nh.param("cad_mesher/file_loc_dataset", file_loc_dataset, std::string("/not_set"));
    nh.param("cad_mesher/dataset", dataset, 6);
    nh.param("cad_mesher/seq", seq, std::string(""));
    nh.param("cad_mesher/max_steps", max_steps, 1);
    std::cout<<"max_steps: "<<max_steps<<std::endl;
    nh.param("cad_mesher/file_loc_report", file_loc_report, std::string("not_set"));
    nh.param("cad_mesher/save_mesh_map", save_mesh_map, false);
    nh.param("cad_mesher/save_raw_point_clouds", save_raw_point_clouds, true);

    //<!--  register param  -->-
    nh.param("cad_mesher/range_max",  range_max,  100.0);
    nh.param("cad_mesher/range_min",  range_min,  1.0);
    nh.param("cad_mesher/range_unit", range_unit, 1.0);
    nh.param("cad_mesher/register_times", register_times, 5);
    nh.param("cad_mesher/cross_overlap", cross_overlap, false);
    nh.param("cad_mesher/cross_cell_overlap_length", cross_cell_overlap_length, 0);
    nh.param("cad_mesher/num_margin_old_cell", num_margin_old_cell, -1);
    nh.param("cad_mesher/point2mesh", point2mesh, false);
    nh.param("cad_mesher/residual_combination", residual_combination, true);
    //<!--  visualize parameter  -->
    nh.param("cad_mesher/full_cover", full_cover, false);
    nh.param("cad_mesher/visualisation_type", visualisation_type, 1);

    //<!--  gp param  -->
    nh.param("cad_mesher/num_thread", num_thread, 1);
    nh.param("cad_mesher/grid", grid, 1.0);
    nh.param("cad_mesher/min_points_num_to_gp", min_points_num_to_gp, 8);
    nh.param("cad_mesher/num_test",  num_test, 10);
    //nh.param("cad_mesher/voxel_size", voxel_size, 0.05);
    voxel_size = grid*1.0/num_test;
    std::cout<<"voxel_size: "<<voxel_size<<std::endl;

    nh.param("cad_mesher/variance_register", variance_register, 0.1);
    nh.param("cad_mesher/variance_map_update", variance_map_update, 0.1);
    nh.param("cad_mesher/variance_map_show", variance_map_show, 0.1);
    nh.param("cad_mesher/variance_min", variance_min, 5.0);
    nh.param("cad_mesher/variance_sensor", variance_sensor, 0.1);

    nh.param("cad_mesher/sliding_window_size", sliding_window_size, 5);
    nh.param("cad_mesher/keyframe_adding_distance", keyframe_adding_distance, 0.5);
    nh.param("cad_mesher/keyframe_adding_angle", keyframe_adding_angle, 0.3);
    nh.param("cad_mesher/scan_number", scan_number, 64);
    nh.param("cad_mesher/use_adaptive_kd", use_adaptive_kd, true);
    nh.param("cad_mesher/use_curvature", use_curvature, true);
    nh.param("cad_mesher/use_curvature_weight", use_curvature_weight, true);
    nh.param("cad_mesher/skip_map_glb_pub", skip_map_glb_pub, 5);
    nh.param("cad_mesher/ground_number_threshold", ground_number_threshold, 5);
    nh.param("cad_mesher/p_stable", stable_threshold, 0.5);
    nh.param("cad_mesher/p_hit", p_hit, 0.7);
    nh.param("cad_mesher/p_miss", p_miss, 0.4);
    nh.param("cad_mesher/use_dynamic_removal", use_dynamic_removal, true);

    //    nh.param("cad_mesher/correction_x", correction_x, 0.0);
//    nh.param("cad_mesher/correction_y", correction_y, 0.0);
//    nh.param("cad_mesher/correction_z", correction_z, 0.0);
//    nh.param("cad_mesher/correction_roll_degree",   correction_roll_degree, 0.0);
//    nh.param("cad_mesher/correction_pitch_degree", correction_pitch_degree, 0.0);
//    nh.param("cad_mesher/correction_yaw_degree",     correction_yaw_degree, 0.0);
    eigen_1 = 48;
    eigen_2 = 0.95;
    eigen_3 = 0.2;
    converge_thr = 0.00001;

    if(scan_number == 64){
        image_res = 3.0;    //10.0
        V_FOV_UP = 5;
        V_FOV_DOWN = -25;
        H_FOV = 360;    //100
    }
    else if(scan_number == 32){
        image_res = 2.0;
        V_FOV_UP = 10;
        V_FOV_DOWN = -30;
        H_FOV = 360;
    }
    else if(scan_number == 16){
        image_res = 1.0;
        V_FOV_UP = 15;
        V_FOV_DOWN = -15;
        H_FOV = 360;
    }
    V_FOV = std::abs(V_FOV_UP) + std::abs(V_FOV_DOWN);
    std::cout<<"=========ROS INIT DONE=========="<<std::endl;
}

//获得初始猜测
Transf CAD_Mesher::getOdom(){
    //before scan registration, obtain initial guess of transformation from motion prior or odometry msg
    Transf odom, dT, odom_now, odom_pre;
    if(param.use_odom_prior){
        //use odometry from topic
        ros::spinOnce();
        odom_now = g_data.transf_odom_now;
        odom_pre = g_data.transf_odom_last;
        dT = odom_pre.inverse() * odom_now;
        // odom = PoseWithCovariance2transf(g_data.odometry_msg_buf.front()->pose);
        odom.block(0, 0, 3, 3) = g_data.q_wmap_wodom.toRotationMatrix() * odom_now.block(0, 0, 3, 3);
        odom.block(0, 3, 3, 1) = g_data.q_wmap_wodom.toRotationMatrix() * odom_now.block(0, 3, 3, 1) + g_data.t_wmap_wodom;
        // odom = g_data.T_seq[g_data.step - 1] * dT;

        //store odom_offline path
        g_data.recordPoseToPath(Odom, odom);
        g_data.transf_odom_last = g_data.transf_odom_now;//once the odom_offline was read, the odom_buff_last will be updated
    }
    else{
        if(g_data.step == 1){
            odom = g_data.T_seq[g_data.step - 1];
        }
        else if (g_data.step > 1){
            //const motion prior
            odom = g_data.T_seq[g_data.step - 1] * g_data.T_seq[g_data.step - 2].inverse() * g_data.T_seq[g_data.step - 1];
            // no motion prior
            // odom = g_data.T_seq[g_data.step-1];
        }
    }
    std::cout << "Pose Odom:" << "x: " << odom(0, 3) << "  y: " << odom(1, 3) << "  z: " << odom(2, 3) << "\n";
    return odom;//use as initial guess
}

void CAD_Mesher::imuIntegration(const sensor_msgs::ImuConstPtr & imu_msg){
    //a very simple integration of imu to provide odometry, neglect this part if no imu is used
    //ROS_INFO("imuIntegration");
    static double time_last;

    double time_now = imu_msg->header.stamp.toSec();
    if( !g_data.receive_first_imu){
        //initialize
        g_data.imu_pos.fill(0);
        g_data.imu_vel.fill(0);
        g_data.imu_Ba << param.bias_acc_x, param.bias_acc_y, 0;

        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        g_data.acc_0 = linear_acceleration;

        tf::Quaternion temp_quaternion;
        tf::quaternionMsgToTF(imu_msg->orientation, temp_quaternion);
        tf::Matrix3x3 matrix(temp_quaternion);

        g_data.imu_rot << matrix[0][0], matrix[0][1], matrix[0][2],
                matrix[1][0], matrix[1][1], matrix[1][2],
                matrix[2][0], matrix[2][1], matrix[2][2];

        time_last = time_now;
        g_data.receive_first_imu = true;
    }
    else{
        double dt = time_now - time_last;
        time_last = time_now;

        double dx = imu_msg->linear_acceleration.x;
        double dy = imu_msg->linear_acceleration.y;
        double dz = imu_msg->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        Eigen::Vector3d _un_acc_0;
        _un_acc_0 = g_data.imu_rot * (g_data.acc_0 - g_data.imu_Ba) - g_data.g;

        tf::Quaternion quat;
        tf::quaternionMsgToTF(imu_msg->orientation, quat);
        tf::Matrix3x3 matrix(quat);
        g_data.imu_rot << matrix[0][0], matrix[0][1], matrix[0][2],
                matrix[1][0], matrix[1][1], matrix[1][2],
                matrix[2][0], matrix[2][1], matrix[2][2];

        Eigen::Vector3d _un_acc_1;
        _un_acc_1 = g_data.imu_rot * (linear_acceleration - g_data.imu_Ba) - g_data.g;


        Eigen::Vector3d _un_acc = 0.5 * (_un_acc_0 + _un_acc_1);

        g_data.imu_pos = g_data.imu_pos + dt * g_data.imu_vel + 0.5 * dt * dt * _un_acc;
        g_data.imu_vel = g_data.imu_vel + dt * _un_acc;
        g_data.acc_0 = linear_acceleration;
    }

    //save imu pose every time update and predict
    Point tmp_point;
    tmp_point << g_data.imu_pos.topRows(2), 0;
    g_data.pose_imu .addPoint(tmp_point);

    g_data.transf_odom_now.block(0, 0, 3, 3) = g_data.imu_rot;
    g_data.transf_odom_now(0, 3) = g_data.imu_pos(0, 0);
    g_data.transf_odom_now(1, 3) = g_data.imu_pos(1, 0);
}
void CAD_Mesher::imuCallback(const sensor_msgs::Imu::ConstPtr & imu_msg){
    //receive imu data
    mBuf.lock();
    g_data.imu_msg_buf.push(imu_msg);
    mBuf.unlock();
}
void CAD_Mesher::odomCallback(const nav_msgs::Odometry::ConstPtr & odom_msg){
    //receive odometry data
    mBuf.lock();
    g_data.odometry_msg_buf.push_back(odom_msg);
    mBuf.unlock();

    //high frequence publish
    Eigen::Quaterniond q_wodom_now_tmp;
    Eigen::Vector3d t_wodom_now_tmp;
    q_wodom_now_tmp.x() = odom_msg->pose.pose.orientation.x;
    q_wodom_now_tmp.y() = odom_msg->pose.pose.orientation.y;
    q_wodom_now_tmp.z() = odom_msg->pose.pose.orientation.z;
    q_wodom_now_tmp.w() = odom_msg->pose.pose.orientation.w;
    t_wodom_now_tmp.x() = odom_msg->pose.pose.position.x;
    t_wodom_now_tmp.y() = odom_msg->pose.pose.position.y;
    t_wodom_now_tmp.z() = odom_msg->pose.pose.position.z;

    Eigen::Quaterniond q_w_new = g_data.q_wmap_wodom * q_wodom_now_tmp;
    Eigen::Vector3d t_w_new = g_data.q_wmap_wodom * t_wodom_now_tmp + g_data.t_wmap_wodom;

    nav_msgs::Odometry aft_mapping_highfrec_odom;
    aft_mapping_highfrec_odom.header.frame_id = "map";
    aft_mapping_highfrec_odom.child_frame_id = "cad_mesher_aft_mapped_highfrec_odom";
    aft_mapping_highfrec_odom.header.stamp = odom_msg->header.stamp;
    aft_mapping_highfrec_odom.pose.pose.orientation.x = q_w_new.x();
    aft_mapping_highfrec_odom.pose.pose.orientation.y = q_w_new.y();
    aft_mapping_highfrec_odom.pose.pose.orientation.z = q_w_new.z();
    aft_mapping_highfrec_odom.pose.pose.orientation.w = q_w_new.w();
    aft_mapping_highfrec_odom.pose.pose.position.x = t_w_new.x();
    aft_mapping_highfrec_odom.pose.pose.position.y = t_w_new.y();
    aft_mapping_highfrec_odom.pose.pose.position.z = t_w_new.z();
    aft_mapping_odom_highfrec_pub.publish(aft_mapping_highfrec_odom);

    geometry_msgs::PoseStamped aft_mapping_highfrec_odom_pose;
    aft_mapping_highfrec_odom_pose.header.stamp = odom_msg->header.stamp;
    aft_mapping_highfrec_odom_pose.header.frame_id = "map";
    aft_mapping_highfrec_odom_pose.pose.orientation.x = q_w_new.x();
    aft_mapping_highfrec_odom_pose.pose.orientation.y = q_w_new.y();
    aft_mapping_highfrec_odom_pose.pose.orientation.z = q_w_new.z();
    aft_mapping_highfrec_odom_pose.pose.orientation.w = q_w_new.w();
    aft_mapping_highfrec_odom_pose.pose.position.x = t_w_new.x();
    aft_mapping_highfrec_odom_pose.pose.position.y = t_w_new.y();
    aft_mapping_highfrec_odom_pose.pose.position.z = t_w_new.z();
    aft_mapping_highfrec_odom_pose.header.stamp = odom_msg->header.stamp;
    aft_mapping_highfrec_odom_pose.header.frame_id = "map";
    aft_mapping_highfrec_odom_path.poses.push_back(aft_mapping_highfrec_odom_pose);
    aft_mapping_highfrec_odom_path_pub.publish(aft_mapping_highfrec_odom_path);
}
void CAD_Mesher::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr & pcl_msg){
    //receive point cloud
    
    mBuf.lock();
    g_data.pcl_msg_buff_deque.push_back(*pcl_msg);
    if (g_data.pcl_msg_buff_deque.size() > 1){
        g_data.pcl_msg_buff_deque.pop_front();
    }
    mBuf.unlock();
}

bool CAD_Mesher::visualize(Map & map_glb, Map & map_now, int option){
    //visualization. some types may be a heavy load for CAD_Mesher or rviz.
    TicToc t_visualize;
    // publish path
    posePrint(pose_pub, g_data.pose, g_data.step);
    //posePrint(pose_imu_pub, g_data.pose_imu.point, g_data.pose_imu.num_point);
    path_pub.     publish(g_data.path);
    path_odom_pub.publish(g_data.path_odom);
    path_grt_pub. publish(g_data.path_grt);

    // 0 no output
    // 1 publish vertices as point cloud, every current scan and skipped map glb
    // 2 use mesh_tools to visualize mesh, only mesh inside updated cells
    // 3 use mesh_tools to visualize mesh, updated cells and skipped map mesh glb

    if(option == 0){
        return true;
    }
    //pub current scan
    //pub aligned raw points in the world frame
    scanPrint3D(raw_points_in_world_pub, map_now.points_turned, 1);

    //pub current scan vertices as pcl
    map_now.filterVerticesByVariance(param.variance_register);
    scanPrint3D(map_vertices_now_pub, map_now.vertices_filted, 1);

    //pub overlapped vertices in registration, for debug
    //only overlap, for debug
    //map_glb.filterVerticesByVarianceOverlap(param.variance_map_show, map_now);//needed?
    //scanPrint3D(overlap_point_glb, map_glb.vertices_filted, 1);
    //scanPrint3D(overlap_point_now, map_now.vertices_filted, 1);

    if(option == 2 || option == 3){
        // mesh_tool updated cells
        map_now.filterMeshLocal();
        mesh_pub_local.publish(map_now.mesh_msg);
    }

    static int pub_map_glb_count = 0;
    int skip_map_glb_point = 1;
    pub_map_glb_count ++;
    if(pub_map_glb_count % param.skip_map_glb_pub == 0 ){//&& g_data.trajectory_length > 0
        if(option == 1){
            //all map_glb pcl
            map_glb.filterVerticesByVariance(param.variance_map_show);
            scanPrint3D(map_vertices_glb_pub, map_glb.vertices_filted, skip_map_glb_point);
        }
        if(option == 3){
            // mesh_tool total map
            map_glb.filterMeshGlb();
            mesh_pub.publish(map_glb.mesh_msg);
        }
        if(option == 4){
            // mesh_tool total map
            map_glb.filterMeshGlb();
            mesh_pub.publish(map_glb.mesh_msg);
            map_glb.filterVerticesByVariance(param.variance_map_show);
            scanPrint3D(map_vertices_glb_pub, map_glb.vertices_filted, skip_map_glb_point);
        }

        //for debug: normal
        //scanPrint3D(overlap_point_glb, map_glb.ary_overlap_vertices[1], 1);
        pub_map_glb_count = 0;
    }
    std::cout << "t_visualize: " << t_visualize.toc() << "ms" << std::endl;
    return true;
}

void CAD_Mesher::map2odomUpdate(){
    Eigen::Matrix3d R_wmap_now = g_data.T_seq[g_data.step].block(0, 0, 3, 3);
    Eigen::Matrix3d R_wodom_now = g_data.T_seq[g_data.step].block(0, 0, 3, 3);
    Eigen::Quaterniond q_wmap_now(R_wmap_now);
    Eigen::Quaterniond q_wodom_now(R_wodom_now);
    Eigen::Vector3d t_wmap_now(g_data.T_seq[g_data.step].block(0, 3, 3, 1));
    Eigen::Vector3d t_wodom_now(g_data.transf_odom_now.block(0, 3, 3, 1));
    g_data.q_wmap_wodom = q_wmap_now * q_wodom_now.inverse();
    g_data.t_wmap_wodom = t_wmap_now - g_data.q_wmap_wodom * t_wodom_now;
}

void CAD_Mesher::pubTf(){
    static double first_time = 0;
    //publish tf and odometry message from
    Transf transf_now = g_data.T_seq[g_data.step];
    //pub odometry msg
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = g_data.pcl_msg_buff.header.stamp;
    odom_msg.pose = transf2PoseWithCovariance(transf_now);
    odom_pub.publish(odom_msg);
    //pub tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odom_msg.pose.pose.position.x,
                                    odom_msg.pose.pose.position.y,
                                    odom_msg.pose.pose.position.z));
    q.setW(odom_msg.pose.pose.orientation.w);
    q.setX(odom_msg.pose.pose.orientation.x);
    q.setY(odom_msg.pose.pose.orientation.y);
    q.setZ(odom_msg.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odom_msg.header.stamp, "/map", "/cad_mesher_odom"));

    if (first_time == 0 && param.dataset == 1){ //kitti dataset or mapping evaluation dataset maicity and newer college
      first_time = g_data.pcl_msg_buff.header.stamp.toSec();
    }
    std::cout.flush(); 
    f_save_pose_evo << std::fixed << std::setprecision(6) << (g_data.pcl_msg_buff.header.stamp.toSec() - first_time) << " " 
                    <<  std::setprecision(9) << odom_msg.pose.pose.position.x << " " << odom_msg.pose.pose.position.y 
                    << " " << odom_msg.pose.pose.position.z << " " 
				    << odom_msg.pose.pose.orientation.x << " " << odom_msg.pose.pose.orientation.y << " " 
                    << odom_msg.pose.pose.orientation.z << " " << odom_msg.pose.pose.orientation.w << std::endl;

}

//Compute Spaciousness of Current Scan
void CAD_Mesher::computeSpaciousness(){
    // compute range of points
    std::vector<double> ds;
    pcl::PointCloud<pcl::PointXYZ> sliding_window_first;
    pcl::fromROSMsg(g_data.pcl_msg_buff, sliding_window_first);
    if(sliding_window_first.points.empty()){
        return;
    }
    #pragma omp parallel for num_threads(param.num_thread) default(shared)
    for (int i = 0; i <= sliding_window_first.points.size(); i++){
        double d = std::sqrt(pow(sliding_window_first.points[i].x, 2) + pow(sliding_window_first.points[i].y, 2) + pow(sliding_window_first.points[i].z, 2));
        #pragma omp critical
        ds.push_back(d);
    }

    // median
    std::nth_element(ds.begin(), ds.begin() + ds.size()/2, ds.end());
    double median_curr = ds[ds.size()/2];
    static double median_prev = median_curr;
    g_data.spaciousness = 0.95*median_prev + 0.05*median_curr;
    median_prev = g_data.spaciousness;

}

//Set Adaptive Downsample Parameters
void CAD_Mesher::setDSAdaptiveParams(){
    // computeSpaciousness();
    // Set Downsample Size from Spaciousness Metric
    if(g_data.spaciousness >= 20.0){
        param.voxel_size = 0.1;
        param.keyframe_adding_distance = 0.0;
        param.keyframe_adding_angle = 0.0;
    }
    else if(g_data.spaciousness >= 15.0 && g_data.spaciousness < 20.0){
        param.voxel_size = 0.15;
        param.keyframe_adding_distance = 0.3;
        param.keyframe_adding_angle = 0.2;
    }
    else if(g_data.spaciousness >= 10.0 && g_data.spaciousness < 15.0){
        param.voxel_size = 0.2;
        param.keyframe_adding_distance = 0.5;
        param.keyframe_adding_angle = 0.3;
    }
    else if(g_data.spaciousness >= 5.0 && g_data.spaciousness < 10.0){
        param.voxel_size = 0.25;
        param.keyframe_adding_distance = 0.8;
        param.keyframe_adding_angle = 0.5;
    }
    else if(g_data.spaciousness < 5.0){
        param.voxel_size = 0.3;
        param.keyframe_adding_distance = 1.0;
        param.keyframe_adding_angle = 0.8;
    }
    // param.voxel_size = 1 / g_data.spaciousness;
    // ROS_WARN("-------------------------------------------");
    // std::cout << "spaciousness: " << g_data.spaciousness << std::endl;
    // ROS_WARN("-------------------------------------------");
}

//project pointcloud to range image
cv::Mat CAD_Mesher::projectPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in, cv::Mat & range_image_idx){
    int kNumRimgRow = std::round(param.V_FOV * param.image_res);
    int kNumRimgCol = std::round(param.H_FOV * param.image_res);
    
    cv::Mat range_image = cv::Mat(kNumRimgRow, kNumRimgCol, CV_32FC1, cv::Scalar::all(param.range_max));

    int num_points = pcl_in->points.size();
    #pragma omp parallel for num_threads(param.num_thread) default(shared)
    for(int idx = 0; idx < num_points; idx++){
        const auto& x = pcl_in->points[idx].x;
        const auto& y = pcl_in->points[idx].y;
        const auto& z = pcl_in->points[idx].z;

        const float range = std::sqrt(x * x + y * y + z * z);
        const float yaw = std::atan2(y, x);
        const float pitch = std::atan2(z, std::sqrt(x * x + y * y));
    
        float proj_row = (1.0f - (pitch * 180.0 / M_PI + std::abs(param.V_FOV_DOWN)) / (param.V_FOV- float(0.0))) * kNumRimgRow;
        float proj_col = ((yaw * 180.0 / M_PI + (param.H_FOV / float(2.0))) / (param.H_FOV - float(0.0))) * kNumRimgCol;
        proj_row = std::round(proj_row);
        proj_col = std::round(proj_col);
        const int v = clamp<int>(static_cast<int>(proj_row), 0, kNumRimgRow - 1);
        const int u = clamp<int>(static_cast<int>(proj_col), 0, kNumRimgCol - 1);
    
        if(range < range_image.at<float>(v, u)){
            range_image.at<float>(v, u) = range;
            range_image_idx.at<int>(v, u) = idx;
        }
    }
    return range_image;
}

std::vector<int> CAD_Mesher::getStaticIdxFromDynamicIdx(const std::vector<int>& dynamic_point_idx, int all_points_number){
    std::vector<int> pt_idx_all = linspace<int>(0, all_points_number, all_points_number);

    std::set<int> pt_idx_all_set(pt_idx_all.begin(), pt_idx_all.end());
    for(auto& dyna_pt_idx: dynamic_point_idx){
        pt_idx_all_set.erase(dyna_pt_idx);
    }
    std::vector<int> static_point_indexes(pt_idx_all_set.begin(), pt_idx_all_set.end());
    
    return static_point_indexes;
}

void CAD_Mesher::parseStaticMapPointcloudUsingPtIdx(std::vector<int>& point_idx){
    // extractor
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(point_idx);
    extractor.setInputCloud(g_data.pcl_new_ptr); 
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse 
    g_data.pcl_static_ptr->clear();
    extractor.filter(*g_data.pcl_static_ptr);
}

void CAD_Mesher::parseDynamicMapPointcloudUsingPtIdx(std::vector<int>& point_idx){
    // extractor
    pcl::ExtractIndices<pcl::PointXYZ> extractor;
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(point_idx);
    extractor.setInputCloud(g_data.pcl_new_ptr); 
    extractor.setIndices(index_ptr);
    extractor.setNegative(false); // If set to true, you can extract point clouds outside the specified index

    // parse 
    g_data.pcl_dynamic_ptr->clear();
    extractor.filter(*g_data.pcl_dynamic_ptr);
}

void CAD_Mesher::process(){
    static bool start_flag = true;
    while(g_data.pcl_msg_buff_deque.empty()){
        if(start_flag){
            // printf("\033[2J\033[1;1H"); // ANSI的Esc屏幕控制码： \033[2J -- 清屏 \033[y;xH -- 设置光标位置
            std::cout << std::endl << "========== CAD-Mesher Start ==========" << std::endl;
            ROS_INFO("Waiting for the point cloud...");
        }
        usleep(100);
        start_flag = false;
    }
    //main process
    TicToc t_whole;
    TicToc t_first_map;
    int sliding_window_cont = 0;
    g_data.pcl_static_ptr->clear();
    pcl::fromROSMsg(g_data.pcl_msg_buff_deque.front(), *g_data.pcl_static_ptr);
    g_data.sliding_window.push_back(std::make_pair(Eigen::MatrixXd::Identity(4, 4), g_data.pcl_static_ptr));
    g_data.pcl_submap_ptr->clear();
    *g_data.pcl_submap_ptr += *g_data.pcl_static_ptr;
    sliding_window_cont++;
    //initialize map
    g_data.extendLog();
    Transf Tguess = g_data.initFirstTransf();
    Map map_glb(Tguess);    // init global map
    g_data.updatePose(Tguess);
    std::cout<<"=========FIRST GLB MAP READY=========TIME:"<<t_first_map.toc()<<std::endl;
    double max_rg_time = 5000; //max time for registration (in millisecond)
    Map map_now;
    while(nh.ok()){
        while(!g_data.odometry_msg_buf.empty() && !g_data.pcl_msg_buff_deque.empty()){
            mBuf.lock();
            // time align
            while(!g_data.odometry_msg_buf.empty() && g_data.odometry_msg_buf.front()->header.stamp.toSec() < g_data.pcl_msg_buff_deque.front().header.stamp.toSec()){
                std::cout << "odom: " << g_data.odometry_msg_buf.front()->header.stamp.toSec() << " cloud: " << g_data.pcl_msg_buff_deque.front().header.stamp.toSec() << std::endl;
                g_data.odometry_msg_buf.pop_front();
            }
            
            if(g_data.odometry_msg_buf.empty()){
                mBuf.unlock();
                break;
            }
            if(g_data.odometry_msg_buf.front()->header.stamp.toSec() != g_data.pcl_msg_buff_deque.front().header.stamp.toSec()){
                ROS_ERROR("--CAD-Mesher: unsync messeage!\n");
                printf("time odom %f pointcloud %f \n", g_data.odometry_msg_buf.front()->header.stamp.toSec(), g_data.pcl_msg_buff_deque.front().header.stamp.toSec());
                mBuf.unlock();
                break;
            }
            g_data.transf_odom_now = PoseWithCovariance2transf(g_data.odometry_msg_buf.front()->pose);
            g_data.pcl_msg_buff = g_data.pcl_msg_buff_deque.front();
            g_data.pcl_msg_buff_deque.pop_front();
            g_data.pcl_new_ptr->clear();
            pcl::fromROSMsg(g_data.pcl_msg_buff, *g_data.pcl_new_ptr);
            mBuf.unlock();
            if(!g_data.sliding_window.empty()){
                Transf T_b = g_data.sliding_window.back().first;
                Eigen::Vector3d ypr_b = R2ypr(T_b.block(0, 0, 3, 3));
                Transf T_c = g_data.transf_odom_now;
                Eigen::Vector3d ypr_c = R2ypr(T_c.block(0, 0, 3, 3));
                double dx = T_b(0, 3) - T_c(0, 3);
                double dy = T_b(1, 3) - T_c(1, 3);
                double dz = T_b(2, 3) - T_c(2, 3);
                double dyaw = ypr_b.x() - ypr_c.x();
                double dpitch = ypr_b.y() - ypr_c.y();
                double droll = ypr_b.z() - ypr_c.z();
                
                if (dyaw > M_PI) dyaw = dyaw - M_PI * 2; 
                if (dyaw < -M_PI) dyaw = dyaw + M_PI * 2;
                if (abs(droll) > param.keyframe_adding_angle || 
                    abs(dpitch) > param.keyframe_adding_angle || 
                    abs(dyaw) > param.keyframe_adding_angle || 
                    sqrt(dx * dx + dy * dy + dz * dz) > param.keyframe_adding_distance || sliding_window_cont < param.sliding_window_size - 1){
                    Eigen::Matrix3d R_b = T_b.block(0, 0, 3, 3);
                    Eigen::Quaterniond q_b(R_b);
                    Eigen::Vector3d t_b(T_b.block(0, 3, 1, 3));

                    Eigen::Matrix3d R_c = T_c.block(0, 0, 3, 3);
                    Eigen::Quaterniond q_c(R_c);
                    Eigen::Vector3d t_c(T_c.block(0, 3, 1, 3));

                    if(param.use_dynamic_removal && sliding_window_cont > param.sliding_window_size - 1){
                        TicToc t_dynamic_removal;
    
                        cv::Mat rangeImage_now_idx = cv::Mat(param.V_FOV * param.image_res, param.H_FOV * param.image_res, CV_32SC1, cv::Scalar::all(0));
                        cv::Mat rangeImage_submap_idx = cv::Mat(param.V_FOV * param.image_res, param.H_FOV * param.image_res, CV_32SC1, cv::Scalar::all(0));
                        cv::Mat rangeImage_now = projectPointCloud(g_data.pcl_new_ptr, rangeImage_now_idx);
                        cv::Mat rangeImage_submap = projectPointCloud(g_data.pcl_submap_ptr, rangeImage_submap_idx);
                        cv::Mat rangeImage_diff = cv::Mat(param.V_FOV * param.image_res, param.H_FOV * param.image_res, CV_32FC1, cv::Scalar::all(0.0));
                        cv::absdiff(rangeImage_now, rangeImage_submap, rangeImage_diff);
                        
                        std::vector<int> dynamic_point_idx;
                        for (int row_idx = 0; row_idx < rangeImage_diff.rows; row_idx++) {
                            for (int col_idx = 0; col_idx < rangeImage_diff.cols; col_idx++) {
                                float this_diff = rangeImage_diff.at<float>(row_idx, col_idx);
                                float this_range = rangeImage_now.at<float>(row_idx, col_idx);
                                float adaptive_coeff = 0.5;
                                float adaptive_dynamic_descrepancy_threshold = adaptive_coeff * this_range; // adaptive descrepancy threshold 
                                if(this_diff > adaptive_dynamic_descrepancy_threshold && this_diff < param.range_max){
                                    dynamic_point_idx.emplace_back(rangeImage_now_idx.at<int>(row_idx, col_idx));
                                }
                            }
                        }
                        // std::cout << "raw pointcloud number is: " << g_data.pcl_new_ptr->points.size() << std::endl;
                        // std::cout << "dynamic count number is: " << dynamic_point_idx.size() << std::endl;
                        std::vector<int> static_point_idx = getStaticIdxFromDynamicIdx(dynamic_point_idx, g_data.pcl_new_ptr->points.size());
                        // std::cout << "static count number is: " << static_point_idx.size() << std::endl;
                        parseStaticMapPointcloudUsingPtIdx(static_point_idx);
                                                
                        g_data.time_dynamic_removal(0, g_data.step) = t_dynamic_removal.toc();
                        if (param.visualisation_type >= 3){
                            //publish range image
                            cv::Mat normalized_range, u8_range, color_map;
                            cv::normalize(rangeImage_now, normalized_range, 255, 0, cv::NORM_MINMAX);
                            normalized_range.convertTo(u8_range, CV_8UC1);
                            cv::applyColorMap(u8_range, color_map, cv::COLORMAP_JET);
                            rangeImage_now_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_map).toImageMsg();
                            range_image_now_pub.publish(rangeImage_now_msg);
                            cv::normalize(rangeImage_submap, normalized_range, 255, 0, cv::NORM_MINMAX);
                            normalized_range.convertTo(u8_range, CV_8UC1);
                            cv::applyColorMap(u8_range, color_map, cv::COLORMAP_JET);
                            rangeImage_submap_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", color_map).toImageMsg();
                            range_image_submap_pub.publish(rangeImage_submap_msg);
                        }
                    }
                    else{
                        g_data.pcl_static_ptr->clear();
                        *g_data.pcl_static_ptr = *g_data.pcl_new_ptr;
                    }
                    *g_data.pcl_pre_ptr = *g_data.pcl_new_ptr;
                    pcl::transformPointCloud(*g_data.pcl_static_ptr, *g_data.pcl_static_ptr, T_c);
                    g_data.sliding_window.push_back(std::make_pair(T_c, g_data.pcl_static_ptr));
                    
                    if(g_data.sliding_window.size() > param.sliding_window_size){
                        g_data.sliding_window.pop_front();
                    }
                    TicToc t_sliding_window;
                    g_data.pcl_submap_ptr->clear();
                    if(g_data.sliding_window.size() > 1){
                        for(int i = 0; i < g_data.sliding_window.size(); i++){
                            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pre_2_now;
                            pcl_pre_2_now = (transformPointCloud(g_data.sliding_window[i].second, q_c.conjugate(), (-1*(q_c.conjugate()*t_c))));    //变换到当前帧坐标系
                            *g_data.pcl_submap_ptr += *pcl_pre_2_now;
                        }
                    }
                    g_data.time_sliding_window(0, g_data.step) = t_sliding_window.toc();
                }
                else{
                    // g_data.odometry_msg_buf.pop_front();
                    // g_data.pcl_msg_buff_deque.pop_front();
                    // mBuf.unlock();
                    break;
                }
            } 
            sliding_window_cont++;
            printf("--CAD-Mesher: add %d framekey.\n", sliding_window_cont);

            //begin
            g_data.step++;
            g_data.extendLog();
            TicToc t_step;
            g_data.t_gp = g_data.t_compute_rt = 0;
            //new scan
            Tguess = getOdom();
            //adaptive downsampling
            if(param.use_adaptive_kd){
                setDSAdaptiveParams();
            }
            if(!map_now.processNewScan(Tguess, g_data.step, map_glb)){
                std::cout<<"break"<<std::endl;
                break;
            }
            //register
            map_now.registerToMap(map_glb, Tguess, max_rg_time);
            map2odomUpdate();
            pubTf();
            //map update
            TicToc t_update;
            if(param.use_dynamic_removal){
                map_now.dynamicRemoval(map_glb);
            }
            map_glb.updateMap(map_now);
            g_data.time_update(0, g_data.step) = t_update.toc();
            //draw map
            TicToc t_draw_map;
            visualize(map_glb, map_now, param.visualisation_type);
            g_data.time_cost_draw_map(0, g_data.step) = t_draw_map.toc();
            //the average time, exclude time to read pcd files or wait for a msg (downsample included), and time to pub mesh msg
            g_data.time_cost(0, g_data.step) = t_step.toc() + g_data.time_down_sample(0, g_data.step) -
                                            g_data.time_get_pcl(0, g_data.step) - t_draw_map.toc();
            //report
            std::cout<<"t_overlap_region: "<<g_data.time_find_overlap(0, g_data.step) << "ms"<< std::endl;
            std::cout<<"t_match_points  : "<<g_data.time_find_overlap_points(0, g_data.step) << "ms"<< std::endl;
            std::cout<<"t_gp            : "<<g_data.time_gp          (0, g_data.step) << "ms"<< std::endl;
            std::cout<<"t_rt            : "<<g_data.time_compute_rt  (0,g_data.step) << "ms"<< std::endl;
            std::cout<<"t_update        : "<<g_data.time_update(0, g_data.step) << "ms"<< std::endl;
            std::cout<<"t_draw_map      : "<<g_data.time_cost_draw_map(0, g_data.step) << "ms"<< std::endl;
            std::cout<<"===STEP "<<g_data.step<<"===Time used: "<< g_data.time_cost(0, g_data.step) <<" ms==="<< std::endl;
        }
        if(abort_){
            //some final process, such as report
            if(g_data.step == param.max_steps){
                std::cout << "Reach Max Step, exit" << std::endl;
            }
            if (SpaciousnessThread.joinable()){
                SpaciousnessThread.join();
            }
            map_glb.filterMeshGlb();
            mesh_pub.publish(map_glb.mesh_msg);
            g_data.saveResult(t_whole.toc(), map_glb.vertices_filted, map_glb);
            ros::shutdown();
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

CAD_Mesher::CAD_Mesher(ros::NodeHandle & nh_, Parameter & param_, Log & g_data_) : nh(nh_), param(param_), g_data(g_data_){
    odom_pub = nh.advertise<nav_msgs::Odometry>("/cad_mesher/lidar_odometry_low_frequency", 1);

    map_vertices_glb_pub = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/map_vertices_global", 1);
    map_vertices_now_pub = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/map_vertices_now", 1);
    raw_points_in_world_pub = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/raw_points_in_world", 1);
    overlap_point_glb = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/overlap_point_global", 1);
    overlap_point_now = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/overlap_point_now", 1);

    pose_pub = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/pose_pub", 1);
    pose_imu_pub = nh.advertise<sensor_msgs::PointCloud>("/cad_mesher/pose_imu_pub", 1);
    path_pub = nh.advertise<nav_msgs::Path>("/cad_mesher/path_pub", 1);
    path_odom_pub = nh.advertise<nav_msgs::Path>("/cad_mesher/path_odom_pub", 1);
    path_grt_pub = nh.advertise<nav_msgs::Path>("/cad_mesher/path_grt_pub", 1);

    mesh_pub = nh.advertise<mesh_msgs::MeshGeometryStamped>("/cad_mesher/mesh_global", 1);
    mesh_pub_local = nh.advertise<mesh_msgs::MeshGeometryStamped>("/cad_mesher/mesh_now", 1);

    aft_mapping_odom_highfrec_pub = nh.advertise<nav_msgs::Odometry>("/cad_mesher/lidar_odometry_high_frequency", 10);
    aft_mapping_highfrec_odom_path_pub = nh.advertise<nav_msgs::Path>("/cad_mesher/lidar_odometry_path", 10);
    image_transport::ImageTransport it(nh);
    range_image_now_pub = it.advertise("/cad_mesher/range_image_now", 10);
    range_image_submap_pub = it.advertise("/cad_mesher/range_image_submap", 10);

    param.initParameter(nh);

    //odometry may be provided by those ways, choose one from odom_sub and imu_sub
    odom_sub = nh.subscribe("/mapping_odom", 1, & CAD_Mesher::odomCallback, this);
    //imu_sub = nh.subscribe("/imu/data", 500, & CAD_Mesher::imuCallback, this);

    pointcloud_sub = nh.subscribe("/mapping_cloud", 100, &CAD_Mesher::pointCloudCallback, this);
    g_data.initLog(param.file_loc_report);

    MappingThread = std::thread(&CAD_Mesher::process, this);
    if(!param.use_curvature && param.use_adaptive_kd){
        SpaciousnessThread = std::thread(&CAD_Mesher::computeSpaciousness, this);
    }
    boost::format fmt_pose("%s/%s");
    f_save_pose_evo.open((fmt_pose % param.file_loc_report % "cad-mesher.txt").str(), std::fstream::out);
    if (!f_save_pose_evo.is_open()){
        printf("The file failed to open! Please check the directory!");
        ros::shutdown();
    }
}

CAD_Mesher::~CAD_Mesher(){
    f_save_pose_evo.close();
}

void controlC(int sig) {
  CAD_Mesher::abort();
}

int main(int argc, char **argv){
    std::cout<<"====START===="<<std::endl;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout<<std::setprecision(5)<<setiosflags(std::ios::fixed);
    ros::init(argc, argv, "cad_mesher");

    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);// Debug Info
    signal(SIGINT, controlC);
    sleep(0.5);

    CAD_Mesher cad_mesher(nh, param, g_data);
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
