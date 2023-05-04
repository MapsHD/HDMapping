#include <laszip/laszip_api.h>
#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <Fusion.h>
#include <map>

#include <imgui.h>
#include <imgui_impl_glut.h>
#include <imgui_impl_opengl2.h>
#include <ImGuizmo.h>
#include <imgui_internal.h>

#include <glew.h>
#include <GL/freeglut.h>

#include <structures.h>
#include <ndt.h>

#include <transformations.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian.h>
#include <python-scripts/constraints/relative_pose_tait_bryan_wc_jacobian.h>
#include <chrono>

#define SAMPLE_PERIOD (1.0 / 200.0)

std::vector<Eigen::Vector3d> all_points;
std::vector<Point3D> initial_points;
NDT ndt;
std::vector<Eigen::Vector3d> means;
std::vector<Eigen::Matrix3d> covs;

std::vector<Point3D> intermediate_points;
std::vector<Eigen::Affine3d> intermediate_trajectory;
std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
NDT::GridParameters in_out_params;
std::vector<NDT::PointBucketIndexPair> index_pair;
std::vector<NDT::Bucket> buckets;

bool show_all_points = true;
bool show_initial_points = true;
bool show_intermadiate_points = false;
bool show_covs = false;
int dec_covs = 10;

struct PPoint {
    double timestamp;
    float intensity;
    Eigen::Vector3d point;
};

float rotate_x = 0.0, rotate_y = 0.0;
float translate_x, translate_y = 0.0;
float translate_z = -50.0;
const unsigned int window_width = 800;
const unsigned int window_height = 600;
static ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
int mouse_old_x, mouse_old_y;
bool gui_mouse_down{ false };
int mouse_buttons = 0; 
float mouse_sensitivity = 1.0;

void optimize(std::vector<Point3D> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model, 
    NDT::GridParameters& rgd_params, std::vector<NDT::PointBucketIndexPair>& index_pair, std::vector<NDT::Bucket>& buckets);

void draw_ellipse(const Eigen::Matrix3d& covar, Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd  = 3)
{
    Eigen::LLT<Eigen::Matrix<double,3,3> > cholSolver(covar);
    Eigen::Matrix3d transform = cholSolver.matrixL();

    const double pi = 3.141592;
    const double di = 0.02;
    const double dj = 0.04;
    const double du = di*2*pi;
    const double dv = dj*pi;
    glColor3f(color.x(), color.y(),color.z());

    for (double i = 0; i < 1.0; i+=di)  //horizonal
    {
        for (double j = 0; j < 1.0; j+=dj)  //vertical
        {
            double u = i*2*pi;      //0     to  2pi
            double v = (j-0.5)*pi;  //-pi/2 to pi/2

            const Eigen::Vector3d pp0( cos(v)* cos(u),cos(v) * sin(u),sin(v));
            const Eigen::Vector3d pp1(cos(v) * cos(u + du) ,cos(v) * sin(u + du) ,sin(v));
            const Eigen::Vector3d pp2(cos(v + dv)* cos(u + du) ,cos(v + dv)* sin(u + du) ,sin(v + dv));
            const Eigen::Vector3d pp3( cos(v + dv)* cos(u),cos(v + dv)* sin(u),sin(v + dv));
            Eigen::Vector3d tp0 = transform * (nstd*pp0) + mean;
            Eigen::Vector3d tp1 = transform * (nstd*pp1) + mean;
            Eigen::Vector3d tp2 = transform * (nstd*pp2) + mean;
            Eigen::Vector3d tp3 = transform * (nstd*pp3) + mean;

            glBegin(GL_LINE_LOOP);
            glVertex3dv(tp0.data());
            glVertex3dv(tp1.data());
            glVertex3dv(tp2.data());
            glVertex3dv(tp3.data());
            glEnd();
        }
    }
}

void lidar_odometry_gui() {
    if(ImGui::Begin("lidar_odometry_gui")){
        ImGui::Checkbox("show_all_points", &show_all_points);
        ImGui::Checkbox("show_initial_points", &show_initial_points);
        ImGui::Checkbox("show_covs", &show_covs);
        ImGui::SameLine();
        ImGui::InputInt("dec_covs" , &dec_covs);
        ImGui::Checkbox("show_intermadiate_points", &show_intermadiate_points);

        if(ImGui::Button("optimize")){
            optimize(intermediate_points, intermediate_trajectory, intermediate_trajectory_motion_model, in_out_params, index_pair, buckets);
        }
        

        ImGui::End();
    }
}

void mouse(int glut_button, int state, int x, int y) {
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);
    int button = -1;
    if (glut_button == GLUT_LEFT_BUTTON) button = 0;
    if (glut_button == GLUT_RIGHT_BUTTON) button = 1;
    if (glut_button == GLUT_MIDDLE_BUTTON) button = 2;
    if (button != -1 && state == GLUT_DOWN)
        io.MouseDown[button] = true;
    if (button != -1 && state == GLUT_UP)
        io.MouseDown[button] = false;

    if (!io.WantCaptureMouse)
    {
        if (state == GLUT_DOWN) {
            mouse_buttons |= 1 << glut_button;
        }
        else if (state == GLUT_UP) {
            mouse_buttons = 0;
        }
        mouse_old_x = x;
        mouse_old_y = y;
    }
}

void wheel(int button, int dir, int x, int y)
{
    if (dir > 0)
    {
        translate_z -= 0.05f * translate_z;
    }
    else
    {
        translate_z += 0.05f * translate_z;
    }
    return;
}

void reshape(int w, int h) {
    glViewport(0, 0, (GLsizei)w, (GLsizei)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)w / (GLfloat)h, 0.01, 10000.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void motion(int x, int y) {
    ImGuiIO& io = ImGui::GetIO();
    io.MousePos = ImVec2((float)x, (float)y);

    if (!io.WantCaptureMouse)
    {
        float dx, dy;
        dx = (float)(x - mouse_old_x);
        dy = (float)(y - mouse_old_y);

        gui_mouse_down = mouse_buttons > 0;
        if (mouse_buttons & 1) {
            rotate_x += dy * 0.2f * mouse_sensitivity;
            rotate_y += dx * 0.2f * mouse_sensitivity;
        }
        if (mouse_buttons & 4) {
            translate_x += dx * 0.5f * mouse_sensitivity;
            translate_y -= dy * 0.5f * mouse_sensitivity;
        }
       
        mouse_old_x = x;
        mouse_old_y = y;
    }
    glutPostRedisplay();
}

void display() {
    ImGuiIO& io = ImGui::GetIO();
    glViewport(0, 0, (GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    float ratio = float(io.DisplaySize.x) / float(io.DisplaySize.y);
    
    glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);
    
    reshape((GLsizei)io.DisplaySize.x, (GLsizei)io.DisplaySize.y);
    glTranslatef(translate_x, translate_y, translate_z);
    glRotatef(rotate_x, 1.0, 0.0, 0.0);
    glRotatef(rotate_y, 0.0, 0.0, 1.0);
      
    glBegin(GL_LINES);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(100, 0.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 100, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 100);
    glEnd();
  
    if(show_all_points){
        glColor3d(1.0, 0.0, 0.0);
        glBegin(GL_POINTS);
        for(const auto &p:all_points){
            glVertex3d(p.x(), p.y(), p.z());
        }
        glEnd();
    }
    if(show_initial_points){
        glColor3d(0.0, 1.0, 0.0);
        glBegin(GL_POINTS);
        for(const auto &p:initial_points){
            glVertex3d(p.x, p.y, p.z);
        }
        glEnd();
    }
    if(show_covs){
        for(int i = 0; i < means.size(); i += dec_covs){
            draw_ellipse(covs[i], means[i], Eigen::Vector3f(0.0f, 0.0f, 1.0f), 3);
        }
        //draw_ellipse(const Eigen::Matrix3d& covar, Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd  = 3)
    }
    if(show_intermadiate_points){
        glPointSize(2);
        glColor3d(0.0, 0.0, 1.0);
        glBegin(GL_POINTS);
        for(const auto &p:intermediate_points){
            Eigen::Vector3d pt = intermediate_trajectory[p.index_pose] * Eigen::Vector3d(p.x, p.y, p.z);
            //std::vector<Eigen::Affine3d> intermediate_trajectory;

            glVertex3d(pt.x(), pt.y(), pt.z());
        }
        glEnd();
        glPointSize(1);
    }

    ImGui_ImplOpenGL2_NewFrame();
    ImGui_ImplGLUT_NewFrame();

    lidar_odometry_gui();
       
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    
    glutSwapBuffers();
    glutPostRedisplay();
}

bool initGL(int* argc, char** argv) {
    glutInit(argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowSize(window_width, window_height);
    glutCreateWindow("lidar_odometry");
    glutDisplayFunc(display);
    glutMotionFunc(motion);

    // default initialization
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glEnable(GL_DEPTH_TEST);

    // viewport
    glViewport(0, 0, window_width, window_height);

    // projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60.0, (GLfloat)window_width / (GLfloat)window_height, 0.01, 10000.0);
    glutReshapeFunc(reshape);
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls

    ImGui::StyleColorsDark();
    ImGui_ImplGLUT_Init();
    ImGui_ImplGLUT_InstallFuncs();
    ImGui_ImplOpenGL2_Init();
    return true;
}

std::vector<PPoint> load_point_cloud(const std::string& lazFile)
{
    std::vector<PPoint> points;
    laszip_POINTER laszip_reader;
    if (laszip_create(&laszip_reader))
    {
        fprintf(stderr, "DLL ERROR: creating laszip reader\n");
        std::abort();
    }

    laszip_BOOL is_compressed = 0;
    if (laszip_open_reader(laszip_reader, lazFile.c_str(), &is_compressed))
    {
        fprintf(stderr, "DLL ERROR: opening laszip reader for '%s'\n", lazFile.c_str());
        std::abort();
    }
    std::cout << "compressed : " << is_compressed << std::endl;
    laszip_header *header;

    if (laszip_get_header_pointer(laszip_reader, &header))
    {
        fprintf(stderr, "DLL ERROR: getting header pointer from laszip reader\n");
        std::abort();
    }
    fprintf(stderr, "file '%s' contains %u points\n", lazFile.c_str(), header->number_of_point_records);
    laszip_point *point;
    if (laszip_get_point_pointer(laszip_reader, &point))
    {
        fprintf(stderr, "DLL ERROR: getting point pointer from laszip reader\n");
        std::abort();
    }

    int counter_ts0 = 0;
    for (laszip_U32 j = 0; j < header->number_of_point_records; j++)
    {
        if (laszip_read_point(laszip_reader))
        {
            fprintf(stderr, "DLL ERROR: reading point %u\n", j);
            std::abort();
        }
        PPoint p;
        p.point.x() = header->x_offset + header->x_scale_factor * static_cast<double>(point->X);
        p.point.y() = header->y_offset + header->y_scale_factor * static_cast<double>(point->Y);
        p.point.z() = header->z_offset + header->z_scale_factor * static_cast<double>(point->Z);
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;

        if(p.timestamp == 0){
            counter_ts0 ++;
        }
        if(p.timestamp > 0){
            points.emplace_back(p);
        }//else{
           // std::cout << "timestamp == 0!!!" << std::endl;
        //}
    }

    std::cout << "number points with ts == 0: " << counter_ts0 << std::endl;
    std::cout << "total number points: " << points.size() << std::endl;
    return points;
}

std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string &imu_file)
{
    std::vector<std::tuple<double, FusionVector, FusionVector>> all_data;
    std::ifstream myfile(imu_file);
    if (myfile.is_open()) {
        while (myfile) {
            double data[7];
            myfile >> data[0] >> data[1] >> data[2] >> data[3] >> data[4] >> data[5] >> data[6];
            //std::cout << data[0] << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4] << " " << data[5] << " " << data[6] << std::endl;
            if (data[0] > 0) {
                FusionVector gyr;
                gyr.axis.x = data[1];
                gyr.axis.y = data[2];
                gyr.axis.z = data[3];

                FusionVector acc;
                acc.axis.x = data[4];
                acc.axis.y = data[5];
                acc.axis.z = data[6];

                all_data.emplace_back(data[0] / 1e9, gyr, acc);
            }
        }
        myfile.close();
    }
    return all_data;
}

Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time)
{
    Eigen::Matrix4d ret(Eigen::Matrix4d::Zero());
    auto it_lower = trajectory.lower_bound(query_time);
    auto it_next = it_lower;
    if (it_lower == trajectory.begin()){
        return ret;
    }
    if (it_lower->first > query_time) {
        it_lower = std::prev(it_lower);
    }
    if (it_lower == trajectory.begin()){
        return ret;
    }
    if (it_lower == trajectory.end()){
        return ret;
    }

    double t1 = it_lower->first;
    double t2 = it_next->first;
    double difft1 = t1- query_time;
    double difft2 = t2- query_time;
    if (t1 == t2 && std::fabs(difft1)< 0.1){
        ret = Eigen::Matrix4d::Identity();
        ret.col(3).head<3>() = it_next->second.col(3).head<3>();
        ret.topLeftCorner(3,3) = it_lower->second.topLeftCorner(3,3);
        return ret;
    }
    if (std::fabs(difft1)< 0.15 && std::fabs(difft2)< 0.15 )
    {
        assert(t2>t1);
        assert(query_time>t1);
        assert(query_time<t2);
        ret = Eigen::Matrix4d::Identity();
        double res = (query_time-t1)/(t2-t1);
        Eigen::Vector3d diff = it_next->second.col(3).head<3>() - it_lower->second.col(3).head<3>();
        ret.col(3).head<3>() = it_next->second.col(3).head<3>() + diff*res;
        Eigen::Matrix3d r1 = it_lower->second.topLeftCorner(3, 3).matrix();
        Eigen::Matrix3d r2 = it_next->second.topLeftCorner(3, 3).matrix();
        Eigen::Quaterniond q1(r1);
        Eigen::Quaterniond q2(r2);
        Eigen::Quaterniond qt = q1.slerp(res, q2);
        ret.topLeftCorner(3,3) =  qt.toRotationMatrix();
        return ret;
    }
    std::cout << "Problem with : " <<  difft1 << " " << difft2 << "  q : " << query_time<< " t1 :"<<t1 <<" t2: "<<t2 << std::endl;
    return ret;
}

int main(int argc, char *argv[]){
    std::vector<std::tuple<double, FusionVector, FusionVector>> imu_data1 = load_imu("C:/data/mandeye_360/StopScan+straightGoing/imu0000.csv");
    std::vector<std::tuple<double, FusionVector, FusionVector>> imu_data2 = load_imu("C:/data/mandeye_360/StopScan+straightGoing/imu0001.csv");

    auto im_data = imu_data1;
    im_data.insert(std::end(im_data), std::begin(imu_data2), std::end(imu_data2));

    auto points1 = load_point_cloud("C:/data/mandeye_360/StopScan+straightGoing/lidar0000.laz");
    auto points2 = load_point_cloud("C:/data/mandeye_360/StopScan+straightGoing/lidar0001.laz");

    auto point_data = points1;
    point_data.insert(std::end(point_data), std::begin(points2), std::end(points2));

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    std::map<double, Eigen::Matrix4d> trajectory;

    for(const auto& [timestamp, gyr, acc]:im_data){
        const FusionVector gyroscope = {static_cast<float>(gyr.axis.x * 180.0 / M_PI), static_cast<float>(gyr.axis.y * 180.0 / M_PI), static_cast<float>(gyr.axis.z * 180.0 / M_PI)};
        const FusionVector accelerometer = {acc.axis.x, acc.axis.y, acc.axis.z};

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);

        Eigen::Quaterniond d {quat.element.w,quat.element.x,quat.element.y,quat.element.z};
        Eigen::Affine3d t {Eigen::Matrix4d::Identity()};
        t.rotate(d);
        trajectory[timestamp] = t.matrix();
        const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
    }

    std::cout << "number of points: " << point_data.size() << std::endl;
    for (auto &p : point_data) {
        Eigen::Matrix4d t = getInterpolatedPose(trajectory, p.timestamp);
        if (!t.isZero()) {
            Eigen::Affine3d tt(t);
            Eigen::Vector3d tp = tt * p.point;
            all_points.push_back(tp);
        }
    }

    for(int i = 0; i < 1000000; i++){
        auto p = point_data[i];
        Point3D pp;
        pp.x = p.point.x();
        pp.y = p.point.y();
        pp.z = p.point.z();

        initial_points.push_back(pp);
    }

    std::vector<double> timestamps;
    for(int i = 2600000; i < 2600000 + 100000; i++){
        auto p = point_data[i];
        Point3D pp;
        pp.x = p.point.x();
        pp.y = p.point.y();
        pp.z = p.point.z();
        //std::cout << p.timestamp << std::endl;

        if(p.timestamp > 0){
            intermediate_points.push_back(pp);
            timestamps.push_back(p.timestamp);
        }
        //std::cout << " ts " << p.timestamp;
    }

    //std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<double> trj_ts;

    for(const auto &t:trajectory){
        if(t.first >= timestamps[0] - 0.1 && t.first <= timestamps[timestamps.size() - 1] + 0.1){
            Eigen::Affine3d m;
            m.matrix() = t.second;
            intermediate_trajectory.push_back(m);
            trj_ts.push_back(t.first);
        }
    }



    std::vector<Point3D> intermediate_points_temp;

    //intermediate_trajectory_motion_model = intermediate_trajectory;
    
    for(int i = 0; i < timestamps.size(); i++){
        auto lower = std::lower_bound(trj_ts.begin(), trj_ts.end(), timestamps[i]);

        if(std::distance(trj_ts.begin(), lower) < 0 || std::distance(trj_ts.begin(), lower) >= trj_ts.size()){
            std::cout << "PROBLEM" << std::endl;
            std::cout << timestamps[i] << " " << trj_ts[0] << " " << trj_ts[trj_ts.size() - 1] << std::endl; 
        }else{
            Point3D p = intermediate_points[i];
            p.index_pose = std::distance(trj_ts.begin(), lower);
            intermediate_points_temp.push_back(p);
        }
        //intermediate_points[i].index_pose = std::distance(trj_ts.begin(), lower);
        //std::cout << timestamps[i] << " " << std::distance(trj_ts.begin(), lower) << std::endl;
    }

    //intermediate_trajectory_motion_model = intermediate_points_temp;
    //intermediate_trajectory = intermediate_points_temp;
    intermediate_points = intermediate_points_temp;
    intermediate_trajectory_motion_model = intermediate_trajectory;
    


    /////////////////////////////////////////////////////////////////////////
    in_out_params.resolution_X = 0.3;
    in_out_params.resolution_Y = 0.3;
    in_out_params.resolution_Z = 0.3;
    in_out_params.bounding_box_extension = 1.0;
    
    ndt.compute_cov_mean(initial_points, index_pair, buckets, in_out_params);

    for(int i = 0; i < buckets.size(); i++){
		if(buckets[i].number_of_points > 5){
			//std::cout << i << " " << buckets[i].cov << std::endl;
            covs.push_back(buckets[i].cov);
            means.push_back(buckets[i].mean);
		}
	}
    
    initGL(&argc, argv);
    glutDisplayFunc(display);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheel);
    glutMainLoop();

    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplGLUT_Shutdown();

    ImGui::DestroyContext();
    return 0;
}

void optimize(std::vector<Point3D> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, 
    std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params, std::vector<NDT::PointBucketIndexPair>& index_pair, std::vector<NDT::Bucket>& buckets)
{
    auto start = chrono::steady_clock::now();

    std::cout << "optimize" << std::endl;
    std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

    for(int i = 0; i < intermediate_points.size(); i++){
        //if(intermediate_points[i].)
        Eigen::Vector3d point_local(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);
        if(point_local.norm() < 2.0){
            continue;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points[i].index_pose] * Eigen::Vector3d(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);

        if (point_global.x() < rgd_params.bounding_box_min_X)
		{
			continue;
		}
		if (point_global.x() > rgd_params.bounding_box_max_X)
		{
			continue;
		}
		if (point_global.y() < rgd_params.bounding_box_min_Y)
		{
			continue;
		}
		if (point_global.y() > rgd_params.bounding_box_max_Y)
		{
			continue;
		}
		if (point_global.z() < rgd_params.bounding_box_min_Z)
		{
			continue;
		}
		if (point_global.z() > rgd_params.bounding_box_max_Z)
		{
			continue;
		}

        //check bb
        long long unsigned int ix = (point_global.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (point_global.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (point_global.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

        

		auto index_of_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
										   static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) +
									   iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;
        if(index_of_bucket > buckets.size()){
            continue;
        }
        //std::cout << index_of_bucket << std::endl;

        if(buckets[index_of_bucket].number_of_points >= 5){
            Eigen::Matrix3d infm = buckets[index_of_bucket].cov.inverse();

            if (!(infm(0, 0) == infm(0, 0)))
                continue;
            if (!(infm(0, 1) == infm(0, 1)))
                continue;
            if (!(infm(0, 2) == infm(0, 2)))
                continue;

            if (!(infm(1, 0) == infm(1, 0)))
                continue;
            if (!(infm(1, 1) == infm(1, 1)))
                continue;
            if (!(infm(1, 2) == infm(1, 2)))
                continue;

            if (!(infm(2, 0) == infm(2, 0)))
                continue;
            if (!(infm(2, 1) == infm(2, 1)))
                continue;
            if (!(infm(2, 2) == infm(2, 2)))
                continue;

            double threshold = 10000.0;

            if (infm(0, 0) > threshold)
                continue;
            if (infm(0, 1) > threshold)
                continue;
            if (infm(0, 2) > threshold)
                continue;
            if (infm(1, 0) > threshold)
                continue;
            if (infm(1, 1) > threshold)
                continue;
            if (infm(1, 2) > threshold)
                continue;
            if (infm(2, 0) > threshold)
                continue;
            if (infm(2, 1) > threshold)
                continue;
            if (infm(2, 2) > threshold)
                continue;

            if (infm(0, 0) < -threshold)
                continue;
            if (infm(0, 1) < -threshold)
                continue;
            if (infm(0, 2) < -threshold)
                continue;
            if (infm(1, 0) < -threshold)
                continue;
            if (infm(1, 1) < -threshold)
                continue;
            if (infm(1, 2) < -threshold)
                continue;
            if (infm(2, 0) < -threshold)
                continue;
            if (infm(2, 1) < -threshold)
                continue;
            if (infm(2, 2) < -threshold)
                continue;

            //std::cout << "jojo";

            double delta_x;
            double delta_y;
            double delta_z;

            Eigen::Affine3d m_pose = intermediate_trajectory[intermediate_points[i].index_pose];
            Eigen::Vector3d p_s(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);
            Eigen::Vector3d p_t(buckets[index_of_bucket].mean.x(), buckets[index_of_bucket].mean.y(), buckets[index_of_bucket].mean.z());

            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
					
			TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);

			point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
														pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
														p_s.x(), p_s.y(), p_s.z(), p_t.x(), p_t.y(), p_t.z());

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                    p_s.x(), p_s.y(), p_s.z());
            int ir = tripletListB.size();
            int c = intermediate_points[i].index_pose * 6;
            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 6; col++)
                {
                    if (jacobian(row, col) != 0.0)
                    {
                        tripletListA.emplace_back(ir + row, c + col, -jacobian(row, col));
                    }
                }
            }
            tripletListB.emplace_back(ir, 0, delta_x);
            tripletListB.emplace_back(ir + 1, 0, delta_y);
            tripletListB.emplace_back(ir + 2, 0, delta_z);

            tripletListP.emplace_back(ir, ir, infm(0, 0));
            tripletListP.emplace_back(ir, ir + 1, infm(0, 1));
            tripletListP.emplace_back(ir, ir + 2, infm(0, 2));
            tripletListP.emplace_back(ir + 1, ir, infm(1, 0));
            tripletListP.emplace_back(ir + 1, ir + 1, infm(1, 1));
            tripletListP.emplace_back(ir + 1, ir + 2, infm(1, 2));
            tripletListP.emplace_back(ir + 2, ir, infm(2, 0));
            tripletListP.emplace_back(ir + 2, ir + 1, infm(2, 1));
            tripletListP.emplace_back(ir + 2, ir + 2, infm(2, 2));
        }
    }
    std::cout << "ndt finished" << std::endl;

    //
    std::vector<std::pair<int, int>> odo_edges;
    for(size_t i = 1; i < intermediate_trajectory.size(); i++){
		odo_edges.emplace_back(i-1,i);
	}

    std::vector<TaitBryanPose> poses;
    std::vector<TaitBryanPose> poses_desired;

    for(size_t i = 0 ; i < intermediate_trajectory.size(); i++){
        poses.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]));
    }
    for(size_t i = 0 ; i < intermediate_trajectory_motion_model.size(); i++){
        poses_desired.push_back(pose_tait_bryan_from_affine_matrix(intermediate_trajectory_motion_model[i]));
    }

    for(size_t i = 0 ; i < odo_edges.size(); i++){
        Eigen::Matrix<double, 6, 1> relative_pose_measurement_odo;
        relative_pose_tait_bryan_wc_case1(relative_pose_measurement_odo,
                poses_desired[odo_edges[i].first].px,
                poses_desired[odo_edges[i].first].py,
                poses_desired[odo_edges[i].first].pz,
                poses_desired[odo_edges[i].first].om,
                poses_desired[odo_edges[i].first].fi,
                poses_desired[odo_edges[i].first].ka,
                poses_desired[odo_edges[i].second].px,
                poses_desired[odo_edges[i].second].py,
                poses_desired[odo_edges[i].second].pz,
                poses_desired[odo_edges[i].second].om,
                poses_desired[odo_edges[i].second].fi,
                poses_desired[odo_edges[i].second].ka);

        Eigen::Matrix<double, 6, 1> delta;
        relative_pose_obs_eq_tait_bryan_wc_case1(
                delta,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka,
                relative_pose_measurement_odo(0,0),
                relative_pose_measurement_odo(1,0),
                relative_pose_measurement_odo(2,0),
                relative_pose_measurement_odo(3,0),
                relative_pose_measurement_odo(4,0),
                relative_pose_measurement_odo(5,0));

        Eigen::Matrix<double, 6, 12, Eigen::RowMajor> jacobian;
        relative_pose_obs_eq_tait_bryan_wc_case1_jacobian(jacobian,
                poses[odo_edges[i].first].px,
                poses[odo_edges[i].first].py,
                poses[odo_edges[i].first].pz,
                poses[odo_edges[i].first].om,
                poses[odo_edges[i].first].fi,
                poses[odo_edges[i].first].ka,
                poses[odo_edges[i].second].px,
                poses[odo_edges[i].second].py,
                poses[odo_edges[i].second].pz,
                poses[odo_edges[i].second].om,
                poses[odo_edges[i].second].fi,
                poses[odo_edges[i].second].ka);

        int ir = tripletListB.size();

        int ic_1 = odo_edges[i].first * 6;
        int ic_2 = odo_edges[i].second * 6;

        for(size_t row = 0 ; row < 6; row ++){
            tripletListA.emplace_back(ir + row, ic_1    , -jacobian(row,0));
            tripletListA.emplace_back(ir + row, ic_1 + 1, -jacobian(row,1));
            tripletListA.emplace_back(ir + row, ic_1 + 2, -jacobian(row,2));
            tripletListA.emplace_back(ir + row, ic_1 + 3, -jacobian(row,3));
            tripletListA.emplace_back(ir + row, ic_1 + 4, -jacobian(row,4));
            tripletListA.emplace_back(ir + row, ic_1 + 5, -jacobian(row,5));

            tripletListA.emplace_back(ir + row, ic_2    , -jacobian(row,6));
            tripletListA.emplace_back(ir + row, ic_2 + 1, -jacobian(row,7));
            tripletListA.emplace_back(ir + row, ic_2 + 2, -jacobian(row,8));
            tripletListA.emplace_back(ir + row, ic_2 + 3, -jacobian(row,9));
            tripletListA.emplace_back(ir + row, ic_2 + 4, -jacobian(row,10));
            tripletListA.emplace_back(ir + row, ic_2 + 5, -jacobian(row,11));
        }

        tripletListB.emplace_back(ir,     0, delta(0,0));
        tripletListB.emplace_back(ir + 1, 0, delta(1,0));
        tripletListB.emplace_back(ir + 2, 0, delta(2,0));
        tripletListB.emplace_back(ir + 3, 0, delta(3,0));
        tripletListB.emplace_back(ir + 4, 0, delta(4,0));
        tripletListB.emplace_back(ir + 5, 0, delta(5,0));

        tripletListP.emplace_back(ir ,    ir,     1000000);
        tripletListP.emplace_back(ir + 1, ir + 1, 1000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 1000000);
        tripletListP.emplace_back(ir + 3, ir + 3, 1000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 1000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }

    Eigen::SparseMatrix<double> matA(tripletListB.size(), intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> matP(tripletListB.size(), tripletListB.size());
    Eigen::SparseMatrix<double> matB(tripletListB.size(), 1);

    matA.setFromTriplets(tripletListA.begin(), tripletListA.end());
    matP.setFromTriplets(tripletListP.begin(), tripletListP.end());
    matB.setFromTriplets(tripletListB.begin(), tripletListB.end());

    Eigen::SparseMatrix<double> AtPA(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPB(intermediate_trajectory.size() * 6, 1);

    {
    Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
    AtPA = (AtP) * matA;
    AtPB = (AtP) * matB;
    }

    tripletListA.clear();
    tripletListP.clear();
    tripletListB.clear();

    std::cout << "AtPA.size: " << AtPA.size() << std::endl;
    std::cout << "AtPB.size: " << AtPB.size() << std::endl;

    std::cout << "start solving AtPA=AtPB" << std::endl;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

    std::cout << "x = solver.solve(AtPB)" << std::endl;
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);

    std::vector<double> h_x;

    for (int k=0; k<x.outerSize(); ++k){
        for (Eigen::SparseMatrix<double>::InnerIterator it(x,k); it; ++it){
            h_x.push_back(it.value());
        }
    }
    std::cout << "h_x.size(): " << h_x.size() << std::endl;
    std::cout << "AtPA=AtPB SOLVED" << std::endl;

    for(size_t i = 0 ; i < h_x.size(); i++){
        std::cout << h_x[i] << std::endl;
    }

    if(h_x.size() == 6 * intermediate_trajectory.size()){
        int counter = 0;

        for(size_t i = 0; i < intermediate_trajectory.size(); i++){
            TaitBryanPose pose = pose_tait_bryan_from_affine_matrix(intermediate_trajectory[i]);
            pose.px += h_x[counter++];
            pose.py += h_x[counter++];
            pose.pz += h_x[counter++];
            pose.om += h_x[counter++];
            pose.fi += h_x[counter++];
            pose.ka += h_x[counter++];
            intermediate_trajectory[i] = affine_matrix_from_pose_tait_bryan(pose);
        }
        std::cout << "optimizing with tait bryan finished" << std::endl;
    }else{
        std::cout << "optimizing with tait bryan FAILED" << std::endl;
    }

    auto end = chrono::steady_clock::now();
    std::cout << "Elapsed time in milliseconds: "
        << chrono::duration_cast<chrono::milliseconds>(end - start).count()
        << " ms" << endl;
return;
}