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

#define SAMPLE_PERIOD (1.0 / 200.0)

std::vector<Eigen::Vector3d> all_points;
std::vector<Point3D> initial_points;
NDT ndt;
std::vector<Eigen::Vector3d> means;
std::vector<Eigen::Matrix3d> covs;

std::vector<Point3D> intermediate_points;


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
        glColor3d(0.0, 0.0, 1.0);
        glBegin(GL_POINTS);
        for(const auto &p:intermediate_points){
            glVertex3d(p.x, p.y, p.z);
        }
        glEnd();
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
        points.emplace_back(p);
    }
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

    for(int i = 5500000; i < 5800000; i++){
        auto p = point_data[i];
        Point3D pp;
        pp.x = p.point.x();
        pp.y = p.point.y();
        pp.z = p.point.z();
        //std::cout << p.timestamp << std::endl;
        intermediate_points.push_back(pp);
    }


    NDT::GridParameters in_out_params;
    in_out_params.resolution_X = 0.3;
    in_out_params.resolution_Y = 0.3;
    in_out_params.resolution_Z = 0.3;
    in_out_params.bounding_box_extension = 1.0;
    

    std::vector<NDT::PointBucketIndexPair> index_pair;
    std::vector<NDT::Bucket> buckets;

    ndt.compute_cov_mean(initial_points, index_pair, buckets, in_out_params);

    for(int i = 0; i < buckets.size(); i++){
		if(buckets[i].number_of_points > 5){
			//std::cout << i << " " << buckets[i].cov << std::endl;
            covs.push_back(buckets[i].cov);
            means.push_back(buckets[i].mean);
		}
	}

    //ndt.build_rgd(initial_points, index_pair, buckets, in_out_params);
    //std::cout << "buckets.size() " << buckets.size() << std::endl;
    //bool optimize(std::vector<PointCloud> &point_clouds, bool compute_only_mahalanobis_distance, bool compute_only_mean_and_cov);
    
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