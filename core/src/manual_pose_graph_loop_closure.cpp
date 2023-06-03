#include <manual_pose_graph_loop_closure.h>

void ManualPoseGraphLoopClosure::Gui(PointClouds &point_clouds_container, int &index_loop_closure_source, int &index_loop_closure_target)
{
    if(point_clouds_container.point_clouds.size() > 0){
        ImGui::InputInt("index_loop_closure_source", &index_loop_closure_source);
        if(index_loop_closure_source < 0){
            index_loop_closure_source = 0;
        }
        if (index_loop_closure_source >= point_clouds_container.point_clouds.size() - 1){
            index_loop_closure_source = point_clouds_container.point_clouds.size() - 1;
        }
        ImGui::InputInt("index_loop_closure_target", &index_loop_closure_target);
        if (index_loop_closure_target < 0)
        {
            index_loop_closure_target = 0;
        }
        if (index_loop_closure_target >= point_clouds_container.point_clouds.size() - 1)
        {
            index_loop_closure_target = point_clouds_container.point_clouds.size() - 1;
        }

        if (ImGui::Button("Add edge"))
        {
            Edge edge;
            edge.index_from = index_loop_closure_source;
            edge.index_to = index_loop_closure_target;
            //

            edge.relative_pose_tb = pose_tait_bryan_from_affine_matrix(
                point_clouds_container.point_clouds[index_loop_closure_source].m_pose.inverse() *
                point_clouds_container.point_clouds[index_loop_closure_target].m_pose);

            edge.relative_pose_tb_weights.px = 1.0;
            edge.relative_pose_tb_weights.py = 1.0;
            edge.relative_pose_tb_weights.pz = 1.0;
            edge.relative_pose_tb_weights.om = 1.0;
            edge.relative_pose_tb_weights.fi = 1.0;
            edge.relative_pose_tb_weights.ka = 1.0;


        }
    }
}

void ManualPoseGraphLoopClosure::Render(PointClouds &point_clouds_container,
                                        int index_loop_closure_source, int index_loop_closure_target)
{
    ObservationPicking observation_picking;
    //point_clouds_container.point_clouds[index_loop_closure_source].render(false, observation_picking, 1);
    point_clouds_container.point_clouds.at(index_loop_closure_source).render(false, observation_picking, 1);
    point_clouds_container.point_clouds.at(index_loop_closure_target).render(false, observation_picking, 1);

    //render trajectory
    //glColor3f(1.0f, 0.0f, 0.0f);
    //glBegin(GL_LINE_STRIP);
    //    for(const auto &pc:point_clouds_container.point_clouds){
    //        glVertex3f(pc.m_pose.translation().x(), pc.m_pose.translation().y(), pc.m_pose.translation().z());
    //    }
    //glEnd();
}