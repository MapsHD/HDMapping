#include <portable-file-dialogs.h>

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
#include <python-scripts/constraints/smoothness_tait_bryan_wc_jacobian.h>
#include <python-scripts/point-to-point-metrics/point_to_point_source_to_target_tait_bryan_wc_jacobian_simplified.h>
//

#define SAMPLE_PERIOD (1.0 / 200.0)
//#define NR_ITER 100
namespace fs = std::filesystem;



std::vector<Eigen::Vector3d> all_points;
std::vector<Point3Di> initial_points;
NDT ndt;
std::vector<Eigen::Vector3d> means;
std::vector<Eigen::Matrix3d> covs;

NDT::GridParameters in_out_params;

std::vector<NDT::PointBucketIndexPair> index_pair;
std::vector<NDT::Bucket> buckets;

Eigen::Matrix4d getInterpolatedPose(const std::map<double, Eigen::Matrix4d> &trajectory, double query_time);
std::vector<Point3Di> decimate(std::vector<Point3Di> points, double bucket_x, double bucket_y, double bucket_z);
void update_rgd(NDT::GridParameters& rgd_params, std::vector<NDT::Bucket>& buckets,
                std::vector<Point3Di>& points_global);

bool show_all_points = false;
bool show_initial_points = true;
//bool show_intermadiate_points = false;
bool show_covs = false;
int dec_covs = 10;
double filter_threshold_xy = 1.5;
int nr_iter = 100;

bool fusionConventionNwu = true;
bool fusionConventionEnu = false;
bool fusionConventionNed = false;

//struct PPoint {
//    double timestamp;
//    float intensity;
//    Eigen::Vector3d point;
//};



struct WorkerData{
    std::vector<Point3Di> intermediate_points;
    std::vector<Point3Di> original_points;
    std::vector<Eigen::Affine3d> intermediate_trajectory;
    std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    bool show = false;
};

std::vector<WorkerData> worker_data;



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
std::string working_directory = "";
double decimation = 0.01;

std::vector<std::tuple<double, FusionVector, FusionVector>> load_imu(const std::string &imu_file);
std::vector<Point3Di> load_point_cloud(const std::string& lazFile);
void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model, 
    NDT::GridParameters& rgd_params, std::vector<NDT::Bucket>& buckets);

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

bool update_sliding_window_rgd(Eigen::Vector3d offset, 
                        std::vector<WorkerData>& worker_data, 
                        const NDT::GridParameters &rgd_params,
                        std::vector<NDT::Bucket>& buckets)
{
    std::cout << "update_sliding_window_rgd NOT IMPLEMENTED" << std::endl;
    exit(1);
   
    #if 0
    long long int ix_prev = (0.0 - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
    long long int iy_prev = (0.0 - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
    long long int iz_prev = (0.0 - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

    long long int ix_curr = (offset.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
    long long int iy_curr = (offset.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
    long long int iz_curr = (offset.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

    if(ix_prev != ix_curr || iy_prev != iy_curr || iz_prev != iz_curr){
        std::cout << "update_sliding_window_rgd" << std::endl;

        std::cout << "ix_prev: " << ix_prev << std::endl;
        std::cout << "iy_prev: " << iy_prev << std::endl;
        std::cout << "iz_prev: " << iz_prev << std::endl;

        std::cout << "ix_curr: " << ix_curr << std::endl;
        std::cout << "iy_curr: " << iy_curr << std::endl;
        std::cout << "iz_curr: " << iz_curr << std::endl;
        //exit(1);
        //NDT::GridParameters in_out_params;
        //std::vector<NDT::PointBucketIndexPair> index_pair;
        //std::vector<NDT::Bucket> buckets;

        std::vector<NDT::Bucket> buckets_new(buckets.size());
        for(int i = 0; i < buckets_new.size(); i++){
            buckets_new[i].number_of_points = 0;
            buckets_new[i].index_begin = 0;
            buckets_new[i].index_end = 0;
            buckets_new[i].cov = Eigen::Matrix3d::Zero();
            buckets_new[i].mean = {0,0,0};
        }

        for(int ix = 0; ix < in_out_params.number_of_buckets_X; ix++){
            for(int iy = 0; iy < in_out_params.number_of_buckets_Y; iy++){
                for(int iz = 0; iz < in_out_params.number_of_buckets_Z; iz++){
                    int new_ix = ix - (ix_curr - ix_prev);
                    int new_iy = iy - (iy_curr - iy_prev);
                    int new_iz = iz - (iz_curr - iz_prev);

                    if(new_ix < 0 || new_ix >= in_out_params.number_of_buckets_X){
                        continue;
                    }
                    if(new_iy < 0 || new_iy >= in_out_params.number_of_buckets_Y){
                        continue;
                    }
                    if(new_iz < 0 || new_iz >= in_out_params.number_of_buckets_Z){
                        continue;
                    }

                    long long int new_index_of_bucket = new_ix * static_cast<long long int>(rgd_params.number_of_buckets_Y) *
										   static_cast<long long int>(rgd_params.number_of_buckets_Z) +
									   new_iy * static_cast<long long int>(rgd_params.number_of_buckets_Z) + new_iz;

                    long long int prev_index_of_bucket = ix * static_cast<long long int>(rgd_params.number_of_buckets_Y) *
										   static_cast<long long int>(rgd_params.number_of_buckets_Z) +
									   iy * static_cast<long long int>(rgd_params.number_of_buckets_Z) + iz;       
                    
                    buckets_new[new_index_of_bucket] = buckets[prev_index_of_bucket];   
                    buckets_new[new_index_of_bucket].mean.x() -= (double(ix_curr - ix_prev)) * rgd_params.resolution_X;
                    buckets_new[new_index_of_bucket].mean.y() -= (double(ix_curr - ix_prev)) * rgd_params.resolution_Y;
                    buckets_new[new_index_of_bucket].mean.z() -= (double(ix_curr - ix_prev)) * rgd_params.resolution_Z;

                }
            }
        }
        buckets = buckets_new;

        for(int index = 0 ; index < worker_data.size(); index++){
            //for(auto &it:worker_data[index].intermediate_trajectory){
            for(int ii=0; ii < worker_data[index].intermediate_trajectory.size(); ii++){
                //std::cout << it.translation() << " x" << std::endl;
                worker_data[index].intermediate_trajectory[ii].translation().x() -= (double(ix_curr - ix_prev)) * rgd_params.resolution_X;
                worker_data[index].intermediate_trajectory[ii].translation().y() -= (double(iy_curr - iy_prev)) * rgd_params.resolution_Y;
                worker_data[index].intermediate_trajectory[ii].translation().z() -= (double(iz_curr - iz_prev)) * rgd_params.resolution_Z;
                //worker_data[index].intermediate_trajectory_motion_model[ii].translation().z() -= (iz_curr - iz_prev) * rgd_params.resolution_Z;

                //std::cout << (iz_curr - iz_prev)*rgd_params.resolution_Z << std::endl;
                //std::cout << it.translation() << " xx" << std::endl;

                //it.translation() -= offset;
            }
        }
       
        std::cout << "offset " << offset << std::endl;
        

        return true;
    }
    #endif
    return false;
}

void shift_rgd(const NDT::GridParameters &rgd_params, std::vector<NDT::Bucket>& buckets){

    std::cout << "shift_rgd NOT IMPLEMENTED" << std::endl;
    exit(1);

    #if 0
    std::vector<NDT::Bucket> buckets_new(buckets.size());
    for(int i = 0; i < buckets.size(); i++){
        buckets_new[i].number_of_points = 0;
        buckets_new[i].index_begin = 0;
        buckets_new[i].index_end = 0;
        buckets_new[i].cov = Eigen::Matrix3d::Zero();
        buckets_new[i].mean = {0,0,0};
    }

    for(int ix = 0; ix < in_out_params.number_of_buckets_X; ix++){
        for(int iy = 0; iy < in_out_params.number_of_buckets_Y; iy++){
            for(int iz = 0; iz < in_out_params.number_of_buckets_Z; iz++){
                int new_ix = ix - 0;
                int new_iy = iy - 0;
                int new_iz = iz - 1;

                bool isok = true;
                if(new_ix < 0 || new_ix >= in_out_params.number_of_buckets_X){
                    isok = false;
                }
                if(new_iy < 0 || new_iy >= in_out_params.number_of_buckets_Y){
                    isok = false;
                }
                if(new_iz < 0 || new_iz >= in_out_params.number_of_buckets_Z){
                    isok = false;
                }

                if(isok){
                    long long int new_index_of_bucket = new_ix * static_cast<long long int>(rgd_params.number_of_buckets_Y) *
                                            static_cast<long long int>(rgd_params.number_of_buckets_Z) +
                                        new_iy * static_cast<long long int>(rgd_params.number_of_buckets_Z) + new_iz;

                    long long int prev_index_of_bucket = ix * static_cast<long long int>(rgd_params.number_of_buckets_Y) *
                                            static_cast<long long int>(rgd_params.number_of_buckets_Z) +
                                        iy * static_cast<long long int>(rgd_params.number_of_buckets_Z) + iz;       
                    

                    buckets_new[new_index_of_bucket] = buckets[prev_index_of_bucket]; 
                    buckets_new[new_index_of_bucket].mean.z() -= rgd_params.resolution_Z;
                }   
            }
        }
    }
    buckets = buckets_new;  
    #endif
}

bool saveLaz(const std::string& filename, const WorkerData &data)
{

	constexpr float scale = 0.0001f; // one tenth of milimeter
	// find max
	double max_x{std::numeric_limits<double>::lowest()};
	double max_y{std::numeric_limits<double>::lowest()};
	double max_z{std::numeric_limits<double>::lowest()};

	//double min_x{std::numeric_limits<double>::max() };
	//double min_y{std::numeric_limits<double>::max() };
	//double min_z{std::numeric_limits<double>::max() };
    double min_x = 1000000000000.0;
	double min_y = 1000000000000.0;
	double min_z = 1000000000000.0;

    //struct WorkerData{
    //std::vector<Point3D> intermediate_points;
    //std::vector<Point3D> original_points;
    //std::vector<Eigen::Affine3d> intermediate_trajectory;
    //std::vector<Eigen::Affine3d> intermediate_trajectory_motion_model;
    //bool show = false;
    //};

    std::vector<Point3Di> points;
    Eigen::Affine3d m_pose = data.intermediate_trajectory[0].inverse();
    for(const auto &org_p:data.original_points){
        Point3Di p = org_p;
        p.point =  m_pose * (data.intermediate_trajectory[org_p.index_pose] * org_p.point);
        //Eigen::Vector3d pp(org_p.x, org_p.y, org_p.z);
        //Eigen::Vector3d pt = m_pose * (data.intermediate_trajectory[org_p.index_pose] * pp);
        //Eigen::Vector3d pt = pp;
        //p.x = pt.x();
        //p.y = pt.y();
        //p.z = pt.z();
        points.push_back(p);
    }

	for(auto& p : points)
	{
        if(p.point.x() < min_x){
            min_x = p.point.x();
        }
        if(p.point.x() > max_x){
            max_x = p.point.x();
        }

        if(p.point.y() < min_y){
            min_y = p.point.y();
        }
        if(p.point.y() > max_y){
            max_y = p.point.y();
        }

        if(p.point.z() < min_z){
            min_z = p.point.z();
        }
        if(p.point.z() > max_z){
            max_z = p.point.z();
        }

		/*double x = 0.001 * p.point.x;
		double y = 0.001 * p.point.y;
		double z = 0.001 * p.point.z;

		max_x = std::max(max_x, x);
		max_y = std::max(max_y, y);
		max_z = std::max(max_z, z);

		min_x = std::min(min_x, x);
		min_y = std::min(min_y, y);
		min_z = std::min(min_z, z);*/
	}

	std::cout << "processing: " << filename << "points " << points.size() << std::endl;

	laszip_POINTER laszip_writer;
	if(laszip_create(&laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: creating laszip writer\n");
		return false;
	}

	// get a pointer to the header of the writer so we can populate it

	laszip_header* header;

	if(laszip_get_header_pointer(laszip_writer, &header))
	{
		fprintf(stderr, "DLL ERROR: getting header pointer from laszip writer\n");
		return false;
	}

	// populate the header

	header->file_source_ID = 4711;
	header->global_encoding = (1 << 0); // see LAS specification for details
	header->version_major = 1;
	header->version_minor = 2;
	//    header->file_creation_day = 120;
	//    header->file_creation_year = 2013;
	header->point_data_format = 1;
	header->point_data_record_length = 0;
	header->number_of_point_records = points.size();
	header->number_of_points_by_return[0] = points.size();
	header->number_of_points_by_return[1] = 0;
	header->point_data_record_length = 28;
	header->x_scale_factor = scale;
	header->y_scale_factor = scale;
	header->z_scale_factor = scale;

	header->max_x = max_x;
	header->min_x = min_x;
	header->max_y = max_y;
	header->min_y = min_y;
	header->max_z = max_z;
	header->min_z = min_z;

	// optional: use the bounding box and the scale factor to create a "good" offset
	// open the writer
	laszip_BOOL compress = (strstr(filename.c_str(), ".laz") != 0);

	if(laszip_open_writer(laszip_writer, filename.c_str(), compress))
	{
		fprintf(stderr, "DLL ERROR: opening laszip writer for '%s'\n", filename.c_str());
		return false;
	}

	fprintf(stderr, "writing file '%s' %scompressed\n", filename.c_str(), (compress ? "" : "un"));

	// get a pointer to the point of the writer that we will populate and write

	laszip_point* point;
	if(laszip_get_point_pointer(laszip_writer, &point))
	{
		fprintf(stderr, "DLL ERROR: getting point pointer from laszip writer\n");
		return false;
	}

	laszip_I64 p_count = 0;
	laszip_F64 coordinates[3];

	for(int i = 0; i < points.size(); i++)
	{

		const auto& p = points[i];
		point->intensity = p.intensity;
		//point->gps_time = 0;//p.timestamp * 1e-9;
		//point->user_data = 0;//p.line_id;
		//point->classification = p.point.tag;
		p_count++;
		coordinates[0] = p.point.x();
		coordinates[1] = p.point.y();
		coordinates[2] = p.point.z();
		if(laszip_set_coordinates(laszip_writer, coordinates))
		{
			fprintf(stderr, "DLL ERROR: setting coordinates for point %I64d\n", p_count);
			return false;
		}

		if(laszip_write_point(laszip_writer))
		{
			fprintf(stderr, "DLL ERROR: writing point %I64d\n", p_count);
			return false;
		}
	}

	if(laszip_get_point_count(laszip_writer, &p_count))
	{
		fprintf(stderr, "DLL ERROR: getting point count\n");
		return false;
	}

	fprintf(stderr, "successfully written %I64d points\n", p_count);

	// close the writer

	if(laszip_close_writer(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: closing laszip writer\n");
		return false;
	}

	// destroy the writer

	if(laszip_destroy(laszip_writer))
	{
		fprintf(stderr, "DLL ERROR: destroying laszip writer\n");
		return false;
	}

	std::cout << "exportLaz DONE" << std::endl;
	return true;
}

bool save_poses(const std::string file_name, std::vector<Eigen::Affine3d> m_poses, std::vector<std::string> filenames)
{
	std::ofstream outfile;
	outfile.open(file_name);
	if (!outfile.good())
	{
		std::cout << "can not save file: " << file_name << std::endl;
		return false;
	}

	outfile << m_poses.size() << std::endl;
	for (size_t i = 0; i < m_poses.size(); i++)
	{
		outfile << filenames[i] << std::endl;
		outfile << m_poses[i](0, 0) << " " << m_poses[i](0, 1) << " " << m_poses[i](0, 2) << " " << m_poses[i](0, 3) << std::endl;
		outfile << m_poses[i](1, 0) << " " << m_poses[i](1, 1) << " " << m_poses[i](1, 2) << " " << m_poses[i](1, 3) << std::endl;
		outfile << m_poses[i](2, 0) << " " << m_poses[i](2, 1) << " " << m_poses[i](2, 2) << " " << m_poses[i](2, 3) << std::endl;
		outfile << "0 0 0 1" << std::endl;
	}
	outfile.close();

	return true;
}

void lidar_odometry_gui() {
    if(ImGui::Begin("lidar_odometry_gui v0.9")){
        ImGui::Checkbox("show_all_points", &show_all_points);
        ImGui::Checkbox("show_initial_points", &show_initial_points);
        ImGui::Checkbox("show_covs", &show_covs);
        ImGui::SameLine();
        ImGui::InputInt("dec_covs" , &dec_covs);

        ImGui::InputDouble("resolution_X" , &in_out_params.resolution_X);
        ImGui::InputDouble("resolution_Y" , &in_out_params.resolution_Y);
        ImGui::InputDouble("resolution_Z" , &in_out_params.resolution_Z);

        ImGui::InputDouble("filter_threshold_xy (all local points inside lidar xy_circle radius[m] will be removed)" , &filter_threshold_xy);

        ImGui::InputDouble("decimation (larger value of decimation better performance, but worse accuracy)" , &decimation);
        ImGui::InputInt("number iterations", &nr_iter);

        ImGui::Checkbox("fusionConventionNwu", &fusionConventionNwu);
        if(fusionConventionNwu){
            //fusionConventionNwu
            fusionConventionEnu = false;
            fusionConventionNed = false;
        }
        ImGui::Checkbox("fusionConventionEnu", &fusionConventionEnu);
        if(fusionConventionEnu){
            fusionConventionNwu = false;
            //fusionConventionEnu
            fusionConventionNed = false;
        }
        ImGui::Checkbox("fusionConventionNed", &fusionConventionNed);
        if(fusionConventionNed){
            fusionConventionNwu = false;
            fusionConventionEnu = false;
            //fusionConventionNed
        }


        if(!fusionConventionNwu && !fusionConventionEnu && !fusionConventionNed){
            fusionConventionNwu = true;
        }




        //ImGui::Checkbox("show_intermadiate_points", &show_intermadiate_points);

        /*if(ImGui::Button("shift rgd")){
            std::cout << "shift rgd" << std::endl;
            shift_rgd(in_out_params, buckets);
            covs.clear();
            means.clear();
            for(int j = 0; j < buckets.size(); j++){
                if(buckets[j].number_of_points > 5){
                    //std::cout << i << " " << buckets[i].cov << std::endl;
                    covs.push_back(buckets[j].cov);
                    means.push_back(buckets[j].mean);
                }
            }
        }*/

        if(ImGui::Button("load data")){
            static std::shared_ptr<pfd::open_file> open_file;
            std::vector<std::string> input_file_names;
            ImGui::PushItemFlag(ImGuiItemFlags_Disabled, (bool)open_file);
            const auto t = [&]() {
                std::vector<std::string> filters;
                auto sel = pfd::open_file("Load las files", "C:\\", filters, true).result();
                for (int i = 0; i < sel.size(); i++)
                {
                    input_file_names.push_back(sel[i]);
                    //std::cout << "las file: '" << input_file_name << "'" << std::endl;
                }
            };
            std::thread t1(t);
            t1.join();

            if (input_file_names.size() > 0) {
                if(input_file_names.size() % 2 == 0){
                    working_directory = fs::path(input_file_names[0]).parent_path().string();
                    for(size_t i = 0; i < input_file_names.size(); i++){
                        std::cout << input_file_names[i] << std::endl;
                    }
                    std::cout << "loading imu" << std::endl;
                    std::vector<std::tuple<double, FusionVector, FusionVector>> imu_data;
                    for(size_t i = 0; i < input_file_names.size() / 2; i++){
                        auto imu = load_imu(input_file_names[i].c_str());
                        imu_data.insert(std::end(imu_data), std::begin(imu), std::end(imu));
                    }

                    std::cout << "loading points" << std::endl;
                    std::vector<Point3Di> points; 
                    for(size_t i = input_file_names.size() / 2; i < input_file_names.size(); i++){
                        auto pp = load_point_cloud(input_file_names[i].c_str());
                        points.insert(std::end(points), std::begin(pp), std::end(pp));
                    }
                    
                    //
                    FusionAhrs ahrs;
                    FusionAhrsInitialise(&ahrs);

                    if(fusionConventionNwu){
                        ahrs.settings.convention = FusionConventionNwu;
                    } 
                    if(fusionConventionEnu){
                        ahrs.settings.convention = FusionConventionEnu;
                    } 
                    if(fusionConventionNed){
                        ahrs.settings.convention = FusionConventionNed;
                    } 

                    
                    
                    //ahrs.convention = FusionConventionNwu,
                    //ahrs.settings.convention = FusionConventionNwu;
                    //FusionConventionEnu
                    

                    std::map<double, Eigen::Matrix4d> trajectory;

                    for(const auto& [timestamp, gyr, acc]:imu_data){
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

                    std::cout << "number of points: " << points.size() << std::endl;
                    for (auto &p : points) {
                        Eigen::Matrix4d t = getInterpolatedPose(trajectory, p.timestamp);
                        if (!t.isZero()) {
                            Eigen::Affine3d tt(t);
                            Eigen::Vector3d tp = tt * p.point;
                            all_points.push_back(tp);
                        }
                    }

                    for(int i = 0; i < 1000000; i++){
                        auto p = points[i];
                        //Point3D pp;
                        //pp.x = p.point.x();
                        //pp.y = p.point.y();
                        //pp.z = p.point.z();

                        initial_points.push_back(p);
                    }

                    double timestamp_begin = points[1000000-1].timestamp;
                    std::cout << "timestamp_begin: " << timestamp_begin << std::endl; 

                    std::vector<double> timestamps;
                    std::vector<Eigen::Affine3d> poses;
                    for(const auto &t:trajectory){
                        if(t.first >= timestamp_begin){
                            timestamps.push_back(t.first);
                            Eigen::Affine3d m;
                            m.matrix() = t.second;
                            poses.push_back(m);
                        }
                    }

                    std::cout << "poses.size(): " << poses.size() << std::endl;

                    int thershold = 20;
                    WorkerData wd;
                    std::vector<double> temp_ts;
                    for(size_t i = 0; i < poses.size(); i++){
                        std::cout << "preparing data " << i + 1 << " of " << poses.size() << std::endl;

                        wd.intermediate_trajectory.push_back(poses[i]);
                        wd.intermediate_trajectory_motion_model.push_back(poses[i]);
                        temp_ts.push_back(timestamps[i]);

                        if(wd.intermediate_trajectory.size() >= thershold){
                            for(int k = 0; k < points.size(); k++){
                                if(points[k].timestamp > temp_ts[0] && points[k].timestamp < temp_ts[temp_ts.size() - 1]){
                                    //std::cout << point_data[k].timestamp << " " << temp_ts[0] << " " <<  temp_ts[temp_ts.size() - 1] << std::endl;
                                    auto p = points[k];
                                    //Point3Di pp;
                                    //pp.x = p.point.x();
                                    //pp.y = p.point.y();
                                    //pp.z = p.point.z();
                                    //pp.intensity = p.intensity;
                                    //pp.timestamp = p.timestamp;
                                    //std::cout << pp.x <<" " << pp.y << " " << pp.z << std::endl;
                                  
                                    auto lower = std::lower_bound(temp_ts.begin(), temp_ts.end(), p.timestamp);
                                    p.index_pose = std::distance(temp_ts.begin(), lower);
                                    //std::cout << pp.index_pose << " ";
                                    wd.intermediate_points.push_back(p);
                                    wd.original_points.push_back(p);
                                    //timestamps.push_back(p.timestamp);
                                }
                            }

                            wd.intermediate_points = decimate(wd.intermediate_points, decimation, decimation, decimation);

                            worker_data.push_back(wd);
                            wd.intermediate_points.clear();
                            wd.original_points.clear();
                            wd.intermediate_trajectory.clear();
                            wd.intermediate_trajectory_motion_model.clear();
                            temp_ts.clear();
                        }
                    }
                    std::cout << "XXX compute_cov_mean start" << std::endl;

                    //
                    std::cout << "initial_points.size(): " << initial_points.size() << std::endl;
                    ndt.compute_cov_mean(initial_points, index_pair, buckets, in_out_params);

                    std::cout << "XXX compute_cov_mean done" << std::endl;
                    int counter_active_buckets = 0; 
                    for(int i = 0; i < buckets.size(); i++){
                        if(buckets[i].number_of_points > 5){
                            covs.push_back(buckets[i].cov);
                            means.push_back(buckets[i].mean);
                            counter_active_buckets ++;
                        }
                    }
                    std::cout << ": counter_active_buckets: " << counter_active_buckets << std::endl;
                }else{
                    std::cout << "please select files correctly" << std::endl;
                }
            } 
        }
        if(ImGui::Button("compute_all")){
            std::chrono::time_point<std::chrono::system_clock> start, end;
            start = std::chrono::system_clock::now();
            double acc_distance = 0.0;
            std::vector<Point3Di> points_global;

            for(int i = 0; i < worker_data.size(); i++){
                if(i > 0){
                    Eigen::Vector3d mean_shift(0.0f, 0.0f, 0.0f);
                    for(int tr = 1; tr < worker_data[i-1].intermediate_trajectory.size(); tr++){
                        //acc_distance += ((worker_data[i].intermediate_trajectory[tr-1].inverse()) * worker_data[i].intermediate_trajectory[tr]).translation().norm();
                        Eigen::Affine3d m_relative = worker_data[i].intermediate_trajectory[tr-1].inverse() * worker_data[i].intermediate_trajectory[tr];
                        mean_shift += m_relative.translation();
                    }
                    mean_shift /= (worker_data[i-1].intermediate_trajectory.size() - 1);

                    Eigen::Affine3d m_mean_shift = Eigen::Affine3d::Identity();
                    m_mean_shift.translation() = mean_shift;
                    //std::cout << "mean_shift " << mean_shift << std::endl;

                    std::vector<Eigen::Affine3d> new_trajectory;
                    Eigen::Affine3d current_node = worker_data[i].intermediate_trajectory[0];
                    new_trajectory.push_back(current_node);

                    for(int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++){
                        current_node = current_node * (worker_data[i].intermediate_trajectory[tr-1].inverse() * worker_data[i].intermediate_trajectory[tr]);
                        current_node = current_node * m_mean_shift;
                        new_trajectory.push_back(current_node);
                    }

                    worker_data[i].intermediate_trajectory = new_trajectory;
                    ////////////////////////////////////////////////////////////////////////
                    std::vector<Eigen::Affine3d> new_trajectory_motion_model;
                    Eigen::Affine3d current_node_motion_model = worker_data[i].intermediate_trajectory_motion_model[0];
                    new_trajectory_motion_model.push_back(current_node_motion_model);

                    for(int tr = 1; tr < worker_data[i].intermediate_trajectory_motion_model.size(); tr++){
                        current_node_motion_model = current_node_motion_model * (worker_data[i].intermediate_trajectory_motion_model[tr-1].inverse() * worker_data[i].intermediate_trajectory_motion_model[tr]);
                        current_node_motion_model = current_node_motion_model * m_mean_shift;
                        new_trajectory_motion_model.push_back(current_node_motion_model);
                    }

                    worker_data[i].intermediate_trajectory_motion_model = new_trajectory_motion_model;
                }

                for(int iter = 0; iter < nr_iter; iter++){
                    std::cout << "computing: [" << i + 1 << "] of " << worker_data.size() << std::endl; 
                    std::cout << "computing iter: [" << iter + 1 << "] of " << nr_iter << std::endl; 
                    std::cout << "acc_distance: " << acc_distance << std::endl;

                    optimize(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model, 
                        in_out_params, buckets);
                }

                for(int tr = 1; tr < worker_data[i].intermediate_trajectory.size(); tr++){
                    acc_distance += ((worker_data[i].intermediate_trajectory[tr-1].inverse()) * worker_data[i].intermediate_trajectory[tr]).translation().norm();
                }

                //update
                for(int j = i + 1; j < worker_data.size(); j++){
                    Eigen::Affine3d m_last = worker_data[j - 1].intermediate_trajectory[worker_data[j - 1].intermediate_trajectory.size() - 1];
                    auto tmp = worker_data[j].intermediate_trajectory;

                    worker_data[j].intermediate_trajectory[0] = m_last;
                    for(int k = 1; k < tmp.size(); k++){
                        Eigen::Affine3d m_update = tmp[k-1].inverse() * tmp[k];
                        m_last = m_last * m_update;
                        worker_data[j].intermediate_trajectory[k] = m_last;
                    }
                }

                for(int j = 0; j < worker_data[i].intermediate_points.size(); j++){
                    Point3Di pp =worker_data[i].intermediate_points[j];
                    pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                    points_global.push_back(pp);
                    //Eigen::Vector3d pt = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * 
                    //    Eigen::Vector3d(worker_data[i].intermediate_points[j].x(), worker_data[i].intermediate_points[j].y(), worker_data[i].intermediate_points[j].z());

                    //Point3Di pp = ;
                    //pp.x = pt.x();
                    //pp.y = pt.y();
                    //pp.z = pt.z();
                    //pp.index_pose = worker_data[i].intermediate_points[j].index_pose;
                    //points_global.push_back(pp);
                }

                if(acc_distance > 10.0){
                    index_pair.clear();
                    buckets.clear();
                    ndt.compute_cov_mean(points_global, index_pair, buckets, in_out_params);
                    //points_global.clear();
                    
                    std::vector<Point3Di> points_global_new;
                    for(int k = points_global.size() /2; k < points_global.size(); k++){
                        points_global_new.push_back(points_global[k]);
                    }

                    acc_distance = 0;
                    //update_rgd(in_out_params, buckets, points_global);
                    points_global = points_global_new;
                }else{
                    std::vector<Point3Di> pg;
                    for(int j = 0; j < worker_data[i].intermediate_points.size(); j++){
                        Point3Di pp = worker_data[i].intermediate_points[j];
                        pp.point = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * pp.point;
                        pg.push_back(pp);

                        /*Eigen::Vector3d pt = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * 
                            Eigen::Vector3d(worker_data[i].intermediate_points[j].x, worker_data[i].intermediate_points[j].y, worker_data[i].intermediate_points[j].z);

                        Point3D pp;
                        pp.x = pt.x();
                        pp.y = pt.y();
                        pp.z = pt.z();
                        pp.index_pose = worker_data[i].intermediate_points[j].index_pose;
                        pg.push_back(pp);*/
                    }
                    update_rgd(in_out_params, buckets, pg);
                }
                /**/
                //std::cout << "computed: [" << i + 1 << "] of " << worker_data.size() << std::endl; 
                //update_rgd(in_out_params, buckets, points_global);

                //covs.clear();
                //means.clear();
                //for(int j = 0; j < buckets.size(); j++){
                //    if(buckets[j].number_of_points > 5){
                //        //std::cout << i << " " << buckets[i].cov << std::endl;
                //        covs.push_back(buckets[j].cov);
                //        means.push_back(buckets[j].mean);
                //    }
                //}
                //std::cout << "computed: [" << i + 1 << "] of " << worker_data.size() << std::endl;
                //std::vector<Eigen::Affine3d> intermediate_trajectory_curr = worker_data[i].intermediate_trajectory;
                //Eigen::Vector3d curr_posiotion = worker_data[i].intermediate_trajectory[ worker_data[i].intermediate_trajectory.size() - 1].translation();
                //auto offset = worker_data[i].intermediate_trajectory[ worker_data[i].intermediate_trajectory.size() - 1].translation();
                //std::cout << "curr offset " << offset << std::endl;
                //update_sliding_window_rgd(offset, worker_data, in_out_params, buckets);
            }
            end = std::chrono::system_clock::now();
 
            std::chrono::duration<double> elapsed_seconds = end - start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
        
            std::cout << "finished computation at " << std::ctime(&end_time)
                    << "elapsed time: " << elapsed_seconds.count() << "s\n";

        }
        if(ImGui::Button("save result")){
            //concatenate data

            std::vector<WorkerData> worker_data_concatenated;
            
            WorkerData wd;
            int counter = 0;
            int pose_offset = 0;
            for(int i = 0 ; i < worker_data.size(); i++){
                auto tmp_data = worker_data[i].original_points;

                for(auto &t:tmp_data){
                    t.index_pose += pose_offset;
                }

                wd.intermediate_trajectory.insert(std::end(wd.intermediate_trajectory), 
                    std::begin(worker_data[i].intermediate_trajectory), std::end(worker_data[i].intermediate_trajectory));

                wd.original_points.insert(std::end(wd.original_points), 
                    std::begin(tmp_data), std::end(tmp_data));
                
                pose_offset += worker_data[i].intermediate_trajectory.size();
                
                counter ++;
                if(counter > 50){
                    worker_data_concatenated.push_back(wd); 
                    wd.intermediate_trajectory.clear();
                    wd.original_points.clear();
                    counter = 0; 
                    pose_offset = 0; 
                }
            }

            if(counter > 10){
                worker_data_concatenated.push_back(wd);   
            }    

            std::vector<Eigen::Affine3d> m_poses;
            std::vector<std::string> file_names;
            for(int i = 0 ; i < worker_data_concatenated.size(); i++){
                fs::path path(working_directory);
                std::string filename = ("scan_lio_" + std::to_string(i) + ".laz");
                path /= filename;
                std::cout << "saving to: " << path << std::endl;
                saveLaz(path.string(), worker_data_concatenated[i]);
                m_poses.push_back(worker_data_concatenated[i].intermediate_trajectory[0]);
                file_names.push_back(filename);
            }
            fs::path path(working_directory);
            path /= "lio_poses.reg";
            save_poses(path.string(), m_poses, file_names);
        }

        for(int i = 0; i < worker_data.size(); i++){
            std::string text = "show[" + std::to_string(i) + "]";
            ImGui::Checkbox(text.c_str() , &worker_data[i].show);
            //ImGui::SameLine();
            //std::string text2 = "optimize[" + std::to_string(i) + "]";
            /*if(ImGui::Button(text2.c_str())){
                //Eigen::Vector3d last_position = worker_data[0].intermediate_trajectory[0].translation();

                //std::vector<Eigen::Affine3d> intermediate_trajectory_prev = worker_data[i].intermediate_trajectory;

                for(int iter = 0; iter < NR_ITER; iter++){
                    optimize(worker_data[i].intermediate_points, worker_data[i].intermediate_trajectory, worker_data[i].intermediate_trajectory_motion_model, 
                        in_out_params, buckets);
                }
                //update

                for(int j = i + 1; j < worker_data.size(); j++){
                    Eigen::Affine3d m_last = worker_data[j - 1].intermediate_trajectory[worker_data[j - 1].intermediate_trajectory.size() - 1];
                    auto tmp = worker_data[j].intermediate_trajectory;

                    worker_data[j].intermediate_trajectory[0] = m_last;
                    for(int k = 1; k < tmp.size(); k++){
                        Eigen::Affine3d m_update = tmp[k-1].inverse() * tmp[k];
                        m_last = m_last * m_update;
                        worker_data[j].intermediate_trajectory[k] = m_last;
                    }
                }

                std::vector<Point3D> points_global;

                for(int j = 0; j < worker_data[i].intermediate_points.size(); j++){
                    Eigen::Vector3d pt = worker_data[i].intermediate_trajectory[worker_data[i].intermediate_points[j].index_pose] * 
                        Eigen::Vector3d(worker_data[i].intermediate_points[j].x, worker_data[i].intermediate_points[j].y, worker_data[i].intermediate_points[j].z);

                    Point3D pp;
                    pp.x = pt.x();
                    pp.y = pt.y();
                    pp.z = pt.z();
                    pp.index_pose = worker_data[i].intermediate_points[j].index_pose;
                    points_global.push_back(pp);
                }

                update_rgd(in_out_params, buckets, points_global);

                covs.clear();
                means.clear();
                for(int j = 0; j < buckets.size(); j++){
                    if(buckets[j].number_of_points > 5){
                        //std::cout << i << " " << buckets[i].cov << std::endl;
                        covs.push_back(buckets[j].cov);
                        means.push_back(buckets[j].mean);
                    }
                }

                //std::vector<Eigen::Affine3d> intermediate_trajectory_curr = worker_data[i].intermediate_trajectory;

                //sliding_window_rgd(intermediate_trajectory_prev, intermediate_trajectory_curr, worker_data, in_out_params, index_pair, buckets);
            }*/
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
            glVertex3d(p.point.x(), p.point.y(), p.point.z());
        }
        glEnd();
    }
    if(show_covs){
        for(int i = 0; i < means.size(); i += dec_covs){
            draw_ellipse(covs[i], means[i], Eigen::Vector3f(0.0f, 0.0f, 1.0f), 3);
        }
        //draw_ellipse(const Eigen::Matrix3d& covar, Eigen::Vector3d& mean, Eigen::Vector3f color, float nstd  = 3)
    }

    for(int i = 0; i < worker_data.size(); i++){
        if(worker_data[i].show){
            glPointSize(2);
            glColor3d(0.0, 0.0, 1.0);
            glBegin(GL_POINTS);
            for(const auto &p:worker_data[i].intermediate_points){
                //std::cout << "kk";
                //std::cout << p.index_pose;
                Eigen::Vector3d pt = worker_data[i].intermediate_trajectory[p.index_pose] * p.point;
                glVertex3d(pt.x(), pt.y(), pt.z());
            }
            glEnd();
            glPointSize(1);
        }
    }


    //if(show_intermadiate_points){
        /*glPointSize(2);
        glColor3d(0.0, 0.0, 1.0);
        glBegin(GL_POINTS);
        for(const auto &p:intermediate_points){
            Eigen::Vector3d pt = intermediate_trajectory[p.index_pose] * Eigen::Vector3d(p.x, p.y, p.z);
            //std::vector<Eigen::Affine3d> intermediate_trajectory;

            glVertex3d(pt.x(), pt.y(), pt.z());
        }
        glEnd();
        glPointSize(1);*/
    //}

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

std::vector<Point3Di> load_point_cloud(const std::string& lazFile)
{
    std::vector<Point3Di> points;
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
        Point3Di p;

        double cal_x = 11.0 / 1000.0; //ToDo change if lidar differen than livox mid 360
        double cal_y = 23.29 / 1000.0;
        double cal_z = -44.12 / 1000.0;

        Eigen::Vector3d pf(header->x_offset + header->x_scale_factor * static_cast<double>(point->X), header->y_offset + header->y_scale_factor * static_cast<double>(point->Y), header->z_offset + header->z_scale_factor * static_cast<double>(point->Z));
        p.point.x() = pf.x() - cal_x;
        p.point.y() = pf.y() - cal_y;
        p.point.z() = pf.z() - cal_z;
        p.timestamp = point->gps_time;
        p.intensity = point->intensity;

        if(p.timestamp == 0){
            counter_ts0 ++;
        }else{
            if( sqrt(pf.x() * pf.x() + pf.y() * pf.y()) > filter_threshold_xy){
                points.emplace_back(p);
            }
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

//std::vector<Point3D> intermediate_points;
std::vector<Point3Di> decimate(std::vector<Point3Di> points, double bucket_x, double bucket_y, double bucket_z)
{
    std::vector<Point3Di> out;
   
    PointCloud pc;
    PointCloud::GridParameters params;

	params.resolution_X = bucket_x;
	params.resolution_Y = bucket_y;
	params.resolution_Z = bucket_z;
	params.bounding_box_extension = 1.0;

    std::vector<Eigen::Vector3d> ppoints;
    for(int i = 0; i < points.size(); i++){
        ppoints.emplace_back(points[i].point);
    }

	pc.grid_calculate_params(ppoints, params);
    std::vector<PointCloud::PointBucketIndexPair> ip;
	pc.reindex(ip, ppoints, params);

    for (int i = 1; i < ip.size(); i++) {
		if (ip[i - 1].index_of_bucket != ip[i].index_of_bucket) {
			out.emplace_back(points[ip[i].index_of_point]);
        }
    }

    return out;
}

int main(int argc, char *argv[]){
    in_out_params.resolution_X = 0.3;
    in_out_params.resolution_Y = 0.3;
    in_out_params.resolution_Z = 0.3;
    in_out_params.bounding_box_extension = 20.0;
    
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

#if 0
void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, 
    std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params, std::vector<NDT::Bucket>& buckets)
{
    //auto start = chrono::steady_clock::now();

    //std::cout << "optimize" << std::endl;
    std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

    for(int i = 0; i < intermediate_points.size(); i += 1){
        //if(intermediate_points[i].)
        //Eigen::Vector3d point_local(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);
        if(intermediate_points[i].point.norm() < 1.0){
            continue;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points[i].index_pose] * intermediate_points[i].point;

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

            double delta_x;
            double delta_y;
            double delta_z;

            Eigen::Affine3d m_pose = intermediate_trajectory[intermediate_points[i].index_pose];
            //Eigen::Vector3d p_s(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);
            Eigen::Vector3d &p_s = intermediate_points[i].point;
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
    //std::cout << "ndt finished" << std::endl;

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
        tripletListP.emplace_back(ir + 1, ir + 1, 100000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 100000000);
        tripletListP.emplace_back(ir + 3, ir + 3, 100000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 100000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }

    //smoothness
    for(size_t i = 1; i < poses.size() - 1; i++){
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                poses[i-1].px,
                poses[i-1].py,
                poses[i-1].pz,
                poses[i-1].om,
                poses[i-1].fi,
                poses[i-1].ka,
                poses[i].px,
                poses[i].py,
                poses[i].pz,
                poses[i].om,
                poses[i].fi,
                poses[i].ka,
                poses[i+1].px,
                poses[i+1].py,
                poses[i+1].pz,
                poses[i+1].om,
                poses[i+1].fi,
                poses[i+1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                poses[i-1].px,
                poses[i-1].py,
                poses[i-1].pz,
                poses[i-1].om,
                poses[i-1].fi,
                poses[i-1].ka,
                poses[i].px,
                poses[i].py,
                poses[i].pz,
                poses[i].om,
                poses[i].fi,
                poses[i].ka,
                poses[i+1].px,
                poses[i+1].py,
                poses[i+1].pz,
                poses[i+1].om,
                poses[i+1].fi,
                poses[i+1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i-1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i+1) * 6;

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

            tripletListA.emplace_back(ir + row, ic_3    , -jacobian(row,12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row,13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row,14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row,15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row,16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row,17));
        }
        tripletListB.emplace_back(ir,     0, delta(0,0));
        tripletListB.emplace_back(ir + 1, 0, delta(1,0));
        tripletListB.emplace_back(ir + 2, 0, delta(2,0));
        tripletListB.emplace_back(ir + 3, 0, delta(3,0));
        tripletListB.emplace_back(ir + 4, 0, delta(4,0));
        tripletListB.emplace_back(ir + 5, 0, delta(5,0));

        tripletListP.emplace_back(ir ,    ir,     10000);
        tripletListP.emplace_back(ir + 1, ir + 1, 10000);
        tripletListP.emplace_back(ir + 2, ir + 2, 10000);
        tripletListP.emplace_back(ir + 3, ir + 3, 10000);
        tripletListP.emplace_back(ir + 4, ir + 4, 10000);
        tripletListP.emplace_back(ir + 5, ir + 5, 10000);
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

    //std::cout << "AtPA.size: " << AtPA.size() << std::endl;
    //std::cout << "AtPB.size: " << AtPB.size() << std::endl;

    //std::cout << "start solving AtPA=AtPB" << std::endl;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

    //std::cout << "x = solver.solve(AtPB)" << std::endl;
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);

    std::vector<double> h_x;

    for (int k=0; k<x.outerSize(); ++k){
        for (Eigen::SparseMatrix<double>::InnerIterator it(x,k); it; ++it){
            h_x.push_back(it.value());
        }
    }
    //std::cout << "h_x.size(): " << h_x.size() << std::endl;
    //std::cout << "AtPA=AtPB SOLVED" << std::endl;

    //for(size_t i = 0 ; i < h_x.size(); i++){
    //    std::cout << h_x[i] << std::endl;
    //}

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
        //std::cout << "optimizing with tait bryan finished" << std::endl;
    }else{
        //std::cout << "optimizing with tait bryan FAILED" << std::endl;
    }

    //auto end = chrono::steady_clock::now();
    //std::cout << "single iteration Elapsed time in milliseconds: "
    //    << chrono::duration_cast<chrono::milliseconds>(end - start).count()
    //    << " ms" << endl;

return;
}

#else


void optimize(std::vector<Point3Di> &intermediate_points, std::vector<Eigen::Affine3d> &intermediate_trajectory, 
    std::vector<Eigen::Affine3d> &intermediate_trajectory_motion_model,
    NDT::GridParameters& rgd_params, std::vector<NDT::Bucket>& buckets)
{
    //auto start = chrono::steady_clock::now();

    //std::cout << "optimize" << std::endl;
    std::vector<Eigen::Triplet<double>> tripletListA;
	std::vector<Eigen::Triplet<double>> tripletListP;
	std::vector<Eigen::Triplet<double>> tripletListB;

    Eigen::SparseMatrix<double> AtPAndt(intermediate_trajectory.size() * 6, intermediate_trajectory.size() * 6);
    Eigen::SparseMatrix<double> AtPBndt(intermediate_trajectory.size() * 6, 1);



    for(int i = 0; i < intermediate_points.size(); i += 1){
        //if(intermediate_points[i].)
        //Eigen::Vector3d point_local(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);
        if(intermediate_points[i].point.norm() < 1.0){
            continue;
        }

        Eigen::Vector3d point_global = intermediate_trajectory[intermediate_points[i].index_pose] * intermediate_points[i].point;

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
           
            Eigen::Affine3d m_pose = intermediate_trajectory[intermediate_points[i].index_pose];
            //Eigen::Vector3d p_s(intermediate_points[i].x, intermediate_points[i].y, intermediate_points[i].z);
            Eigen::Vector3d &p_s = intermediate_points[i].point;
            //Eigen::Vector3d p_t(buckets[index_of_bucket].mean.x(), buckets[index_of_bucket].mean.y(), buckets[index_of_bucket].mean.z());
            TaitBryanPose pose_s = pose_tait_bryan_from_affine_matrix(m_pose);
            //
            Eigen::Matrix<double, 6, 6, Eigen::RowMajor> AtPA;
            point_to_point_source_to_target_tait_bryan_wc_AtPA_simplified(
                AtPA, 
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(),
                infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2));

            Eigen::Matrix<double, 6, 1> AtPB;
            point_to_point_source_to_target_tait_bryan_wc_AtPB_simplified(
                AtPB, 
                pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                p_s.x(), p_s.y(), p_s.z(),
                infm(0, 0), infm(0, 1), infm(0, 2), infm(1, 0), infm(1, 1), infm(1, 2), infm(2, 0), infm(2, 1), infm(2, 2),
                buckets[index_of_bucket].mean.x(), buckets[index_of_bucket].mean.y(), buckets[index_of_bucket].mean.z());
            
            int c = intermediate_points[i].index_pose * 6;

            //double aa[6][6];
            //double bb[6][1];

            for(int row = 0; row < 6; row++){
                for(int col = 0; col < 6; col++){
                    AtPAndt.coeffRef(c + row, c + col) += AtPA(row, col);
                    //aa[row][col] = AtPA(row, col);
                    //AtPAndt.coeffRef(c + row, c + col) += AtPA(col, row);
                }
            }

            for(int row = 0; row < 6; row++){
                AtPBndt.coeffRef(c + row, 0) -= AtPB(row, 0);
                //bb[row][0] = AtPB(row, 0);
            }

            //////////////////////////sanity check///////////////////////////////////////
            /*
            Eigen::Matrix<double, 3, 6, Eigen::RowMajor> jacobian;
					
			double delta_x;
            double delta_y;
            double delta_z;

			point_to_point_source_to_target_tait_bryan_wc(delta_x, delta_y, delta_z,
														pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
														p_s.x(), p_s.y(), p_s.z(), buckets[index_of_bucket].mean.x(), buckets[index_of_bucket].mean.y(), buckets[index_of_bucket].mean.z());

            point_to_point_source_to_target_tait_bryan_wc_jacobian(jacobian,
                                                                    pose_s.px, pose_s.py, pose_s.pz, pose_s.om, pose_s.fi, pose_s.ka,
                                                                    p_s.x(), p_s.y(), p_s.z());

            std::vector<Eigen::Triplet<double>> tripletListAtmp;
	        std::vector<Eigen::Triplet<double>> tripletListPtmp;
	        std::vector<Eigen::Triplet<double>> tripletListBtmp;

            for (int row = 0; row < 3; row++)
            {
                for (int col = 0; col < 6; col++)
                {
                    if (jacobian(row, col) != 0.0)
                    {
                        tripletListAtmp.emplace_back(row, col, -jacobian(row, col));
                    }
                }
            }

            tripletListBtmp.emplace_back(0    , 0, delta_x);
            tripletListBtmp.emplace_back(0 + 1, 0, delta_y);
            tripletListBtmp.emplace_back(0 + 2, 0, delta_z);

            tripletListPtmp.emplace_back(0, 0, infm(0, 0));
            tripletListPtmp.emplace_back(0, 0 + 1, infm(0, 1));
            tripletListPtmp.emplace_back(0, 0 + 2, infm(0, 2));
            tripletListPtmp.emplace_back(0 + 1, 0, infm(1, 0));
            tripletListPtmp.emplace_back(0 + 1, 0 + 1, infm(1, 1));
            tripletListPtmp.emplace_back(0 + 1, 0 + 2, infm(1, 2));
            tripletListPtmp.emplace_back(0 + 2, 0, infm(2, 0));
            tripletListPtmp.emplace_back(0 + 2, 0 + 1, infm(2, 1));
            tripletListPtmp.emplace_back(0 + 2, 0 + 2, infm(2, 2));

            Eigen::SparseMatrix<double> matA(tripletListBtmp.size(), 6);
            Eigen::SparseMatrix<double> matP(tripletListBtmp.size(), tripletListBtmp.size());
            Eigen::SparseMatrix<double> matB(tripletListBtmp.size(), 1);

            matA.setFromTriplets(tripletListAtmp.begin(), tripletListAtmp.end());
            matP.setFromTriplets(tripletListPtmp.begin(), tripletListPtmp.end());
            matB.setFromTriplets(tripletListBtmp.begin(), tripletListBtmp.end());

            Eigen::SparseMatrix<double> AtPAtmp(6, 6);
            Eigen::SparseMatrix<double> AtPBtmp(6, 1);

            {
            Eigen::SparseMatrix<double> AtP = matA.transpose() * matP;
            AtPAtmp = (AtP) * matA;
            AtPBtmp = (AtP) * matB;
            }

            std::cout << "-------------------------------" << std::endl;
            std::cout << "-------------------------------" << std::endl;
            for(int row = 0; row < 6; row++){
                for(int col = 0; col < 6; col++){
                    std::cout << aa[row][col] << " " << AtPAtmp.coeffRef(row, col) << std::endl;
                    
                    //AtPAndt.coeffRef(c + row, c + col) += AtPA(row, col);
                    //aa[row][col] = AtPA(row, col);
                    //AtPAndt.coeffRef(c + row, c + col) += AtPA(col, row);
                }
            }
            std::cout << "-------------------------------" << std::endl;
            for(int row = 0; row < 6; row++){
                    std::cout << bb[row][0] << " " << AtPBtmp.coeffRef(row,0) << std::endl;
            }
            */


            ////////////////////////////////////////////////////////////////
        }
    }

    std::cout << "done ..." << std::endl;
    //std::cout << "ndt finished" << std::endl;

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
        tripletListP.emplace_back(ir + 1, ir + 1, 100000000);
        tripletListP.emplace_back(ir + 2, ir + 2, 100000000);
        tripletListP.emplace_back(ir + 3, ir + 3, 100000000);
        tripletListP.emplace_back(ir + 4, ir + 4, 100000000);
        tripletListP.emplace_back(ir + 5, ir + 5, 1000000);
    }

    //smoothness
    for(size_t i = 1; i < poses.size() - 1; i++){
        Eigen::Matrix<double, 6, 1> delta;
        smoothness_obs_eq_tait_bryan_wc(delta,
                poses[i-1].px,
                poses[i-1].py,
                poses[i-1].pz,
                poses[i-1].om,
                poses[i-1].fi,
                poses[i-1].ka,
                poses[i].px,
                poses[i].py,
                poses[i].pz,
                poses[i].om,
                poses[i].fi,
                poses[i].ka,
                poses[i+1].px,
                poses[i+1].py,
                poses[i+1].pz,
                poses[i+1].om,
                poses[i+1].fi,
                poses[i+1].ka);

        Eigen::Matrix<double, 6, 18, Eigen::RowMajor> jacobian;
        smoothness_obs_eq_tait_bryan_wc_jacobian(jacobian,
                poses[i-1].px,
                poses[i-1].py,
                poses[i-1].pz,
                poses[i-1].om,
                poses[i-1].fi,
                poses[i-1].ka,
                poses[i].px,
                poses[i].py,
                poses[i].pz,
                poses[i].om,
                poses[i].fi,
                poses[i].ka,
                poses[i+1].px,
                poses[i+1].py,
                poses[i+1].pz,
                poses[i+1].om,
                poses[i+1].fi,
                poses[i+1].ka);

        int ir = tripletListB.size();

        int ic_1 = (i-1) * 6;
        int ic_2 = i * 6;
        int ic_3 = (i+1) * 6;

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

            tripletListA.emplace_back(ir + row, ic_3    , -jacobian(row,12));
            tripletListA.emplace_back(ir + row, ic_3 + 1, -jacobian(row,13));
            tripletListA.emplace_back(ir + row, ic_3 + 2, -jacobian(row,14));
            tripletListA.emplace_back(ir + row, ic_3 + 3, -jacobian(row,15));
            tripletListA.emplace_back(ir + row, ic_3 + 4, -jacobian(row,16));
            tripletListA.emplace_back(ir + row, ic_3 + 5, -jacobian(row,17));
        }
        tripletListB.emplace_back(ir,     0, delta(0,0));
        tripletListB.emplace_back(ir + 1, 0, delta(1,0));
        tripletListB.emplace_back(ir + 2, 0, delta(2,0));
        tripletListB.emplace_back(ir + 3, 0, delta(3,0));
        tripletListB.emplace_back(ir + 4, 0, delta(4,0));
        tripletListB.emplace_back(ir + 5, 0, delta(5,0));

        tripletListP.emplace_back(ir ,    ir,     10000);
        tripletListP.emplace_back(ir + 1, ir + 1, 10000);
        tripletListP.emplace_back(ir + 2, ir + 2, 10000);
        tripletListP.emplace_back(ir + 3, ir + 3, 10000);
        tripletListP.emplace_back(ir + 4, ir + 4, 10000);
        tripletListP.emplace_back(ir + 5, ir + 5, 10000);
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

    AtPA += AtPAndt; 
    AtPB += AtPBndt; 
    //std::cout << "AtPA.size: " << AtPA.size() << std::endl;
    //std::cout << "AtPB.size: " << AtPB.size() << std::endl;

    //std::cout << "start solving AtPA=AtPB" << std::endl;
    Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>> solver(AtPA);

    //std::cout << "x = solver.solve(AtPB)" << std::endl;
    Eigen::SparseMatrix<double> x = solver.solve(AtPB);

    std::vector<double> h_x;

    for (int k=0; k<x.outerSize(); ++k){
        for (Eigen::SparseMatrix<double>::InnerIterator it(x,k); it; ++it){
            h_x.push_back(it.value());
        }
    }
    //std::cout << "h_x.size(): " << h_x.size() << std::endl;
    //std::cout << "AtPA=AtPB SOLVED" << std::endl;

    //for(size_t i = 0 ; i < h_x.size(); i++){
    //    std::cout << h_x[i] << std::endl;
    //}

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
        //std::cout << "optimizing with tait bryan finished" << std::endl;
    }else{
        //std::cout << "optimizing with tait bryan FAILED" << std::endl;
    }

    //auto end = chrono::steady_clock::now();
    //std::cout << "single iteration Elapsed time in milliseconds: "
    //    << chrono::duration_cast<chrono::milliseconds>(end - start).count()
    //    << " ms" << endl;

return;
}

#endif

void update_rgd(NDT::GridParameters& rgd_params, std::vector<NDT::Bucket>& buckets,
                std::vector<Point3Di>& points_global)
{
    std::cout << "update_rgd" << std::endl;

    for(int i = 0; i < points_global.size(); i++){
        if (points_global[i].point.x() < rgd_params.bounding_box_min_X)
		{
			continue;
		}
		if (points_global[i].point.x() > rgd_params.bounding_box_max_X)
		{
			continue;
		}
		if (points_global[i].point.y() < rgd_params.bounding_box_min_Y)
		{
			continue;
		}
		if (points_global[i].point.y() > rgd_params.bounding_box_max_Y)
		{
			continue;
		}
		if (points_global[i].point.z() < rgd_params.bounding_box_min_Z)
		{
			continue;
		}
		if (points_global[i].point.z() > rgd_params.bounding_box_max_Z)
		{
			continue;
		}

		long long unsigned int ix = (points_global[i].point.x() - rgd_params.bounding_box_min_X) / rgd_params.resolution_X;
		long long unsigned int iy = (points_global[i].point.y() - rgd_params.bounding_box_min_Y) / rgd_params.resolution_Y;
		long long unsigned int iz = (points_global[i].point.z() - rgd_params.bounding_box_min_Z) / rgd_params.resolution_Z;

		auto index_of_bucket = ix * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Y) *
										   static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) +
									   iy * static_cast<long long unsigned int>(rgd_params.number_of_buckets_Z) + iz;

        //std::cout << "index_of_bucket " << index_of_bucket << " " << buckets[index_of_bucket].number_of_points << std::endl;
        //mean_acc      += (cur_acc - mean_acc) / N;
        //mean_gyr      += (cur_gyr - mean_gyr) / N;

        //cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
        //cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

        if(buckets[index_of_bucket].number_of_points == 0){
            buckets[index_of_bucket].mean = points_global[i].point;//Eigen::Vector3d(points_global[i].x, points_global[i].y, points_global[i].z);
            buckets[index_of_bucket].cov = Eigen::Matrix3d::Zero();
            buckets[index_of_bucket].cov(0,0) = 0.03 * 0.03;
            buckets[index_of_bucket].cov(1,1) = 0.03 * 0.03;
            buckets[index_of_bucket].cov(2,2) = 0.03 * 0.03;

            buckets[index_of_bucket].number_of_points = 1;
        }else{
            buckets[index_of_bucket].number_of_points ++;
            auto curr_mean = points_global[i].point;//Eigen::Vector3d(points_global[i].x, points_global[i].y, points_global[i].z);
            auto mean = buckets[index_of_bucket].mean;
            buckets[index_of_bucket].mean += (mean - curr_mean) / buckets[index_of_bucket].number_of_points;
            
            Eigen::Matrix3d cov_update;
            cov_update(0, 0) = (mean.x() - curr_mean.x()) * (mean.x() - curr_mean.x());
            cov_update(0, 1) = (mean.x() - curr_mean.x()) * (mean.y() - curr_mean.y());
            cov_update(0, 2) = (mean.x() - curr_mean.x()) * (mean.z() - curr_mean.z());
            cov_update(1, 0) = (mean.y() - curr_mean.y()) * (mean.x() - curr_mean.x());
            cov_update(1, 1) = (mean.y() - curr_mean.y()) * (mean.y() - curr_mean.y());
            cov_update(1, 2) = (mean.y() - curr_mean.y()) * (mean.z() - curr_mean.z());
            cov_update(2, 0) = (mean.z() - curr_mean.z()) * (mean.x() - curr_mean.x());
            cov_update(2, 1) = (mean.z() - curr_mean.z()) * (mean.y() - curr_mean.y());
            cov_update(2, 2) = (mean.z() - curr_mean.z()) * (mean.z() - curr_mean.z());

            buckets[index_of_bucket].cov = buckets[index_of_bucket].cov * (buckets[index_of_bucket].number_of_points - 1) / buckets[index_of_bucket].number_of_points +
                cov_update * (buckets[index_of_bucket].number_of_points - 1)/ (buckets[index_of_bucket].number_of_points * buckets[index_of_bucket].number_of_points);
            
            //std::cout << "---------before " << std::endl;
            //std::cout <<  buckets[index_of_bucket].cov << std::endl;
            
            //buckets[index_of_bucket].cov += cov_update * (1.0 / buckets[index_of_bucket].number_of_points * buckets[index_of_bucket].number_of_points);
        
            //std::cout << (mean - curr_mean).norm() << " ";
            //std::cout << "---------after " << std::endl;
            //std::cout <<  buckets[index_of_bucket].cov << std::endl;
        }
    }
}