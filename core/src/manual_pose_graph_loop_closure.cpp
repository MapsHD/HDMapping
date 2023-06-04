#include <manual_pose_graph_loop_closure.h>

void ManualPoseGraphLoopClosure::Gui(PointClouds &point_clouds_container, int &index_loop_closure_source, int &index_loop_closure_target)
{
    if(point_clouds_container.point_clouds.size() > 0){

        if (!manipulate_active_edge)
        {
            ImGui::InputInt("index_loop_closure_source", &index_loop_closure_source);
            if (index_loop_closure_source < 0)
            {
                index_loop_closure_source = 0;
            }
            if (index_loop_closure_source >= point_clouds_container.point_clouds.size() - 1)
            {
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

                edges.push_back(edge);

                index_active_edge = edges.size() - 1;
            }
        }

        std::string number_active_edges = "number_edges: " + std::to_string(edges.size());
        ImGui::Text(number_active_edges.c_str());

        ImGui::Checkbox("manipulate_active_edge", &manipulate_active_edge);

        if (edges.size() > 0 && manipulate_active_edge)
        {
            //ImGui::SameLine();
            int remove_edge_index = -1;
            if (ImGui::Button("remove active edge"))
            {
                remove_edge_index = index_active_edge;
                // program_params.is_edge_gizmo = false;
            }

            int prev_index_active_edge = index_active_edge;
                ImGui::InputInt("index_active_edge", &index_active_edge);

            if (index_active_edge < 0)
            {
                index_active_edge = 0;
            }
            if (index_active_edge >= (int)edges.size())
            {
                index_active_edge = (int)edges.size() - 1;
            }

            ImGui::Checkbox("gizmo", &gizmo);

            //if (gizmo && (prev_index_active_edge != index_active_edge))
            //{
            //    int index_src = edges[index_active_edge].index_from;
            //    int index_trg = edges[index_active_edge].index_to;

            //    Eigen::Affine3d m_from = point_clouds_container.point_clouds.at(index_src).m_pose;
            //    Eigen::Affine3d m_to = m_from * affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

                //m_gizmo[0] = (float)m_to(0, 0);
                //m_gizmo[1] = (float)m_to(1, 0);
               // m_gizmo[2] = (float)m_to(2, 0);
                //m_gizmo[3] = (float)m_to(3, 0);
                //m_gizmo[4] = (float)m_to(0, 1);
                //m_gizmo[5] = (float)m_to(1, 1);
               // m_gizmo[6] = (float)m_to(2, 1);
               // m_gizmo[7] = (float)m_to(3, 1);
               // m_gizmo[8] = (float)m_to(0, 2);
               // m_gizmo[9] = (float)m_to(1, 2);
               // m_gizmo[10] = (float)m_to(2, 2);
               // m_gizmo[11] = (float)m_to(3, 2);
               // m_gizmo[12] = (float)m_to(0, 3);
               // m_gizmo[13] = (float)m_to(1, 3);
               // m_gizmo[14] = (float)m_to(2, 3);
                //m_gizmo[15] = (float)m_to(3, 3);
           // }

            ////////////////////////////////////////
            if (remove_edge_index != -1)
            {
                std::vector<Edge> new_edges;
                for (int i = 0; i < edges.size(); i++)
                {
                    if (remove_edge_index != i)
                    {
                        new_edges.push_back(edges[i]);
                    }
                }
                edges = new_edges;

                index_active_edge = remove_edge_index - 1;
            }
        }
    }
}

void ManualPoseGraphLoopClosure::Render(PointClouds &point_clouds_container,
                                        int index_loop_closure_source, int index_loop_closure_target)
{
    point_clouds_container.point_clouds.at(index_loop_closure_source).visible = true;
    point_clouds_container.point_clouds.at(index_loop_closure_target).visible = true;

    if (!manipulate_active_edge){
        ObservationPicking observation_picking;
        point_clouds_container.point_clouds.at(index_loop_closure_source).render(false, observation_picking, 1);
        point_clouds_container.point_clouds.at(index_loop_closure_target).render(false, observation_picking, 1);
    }else{
        if (edges.size() > 0){
            int index_src = edges[index_active_edge].index_from;
            int index_trg = edges[index_active_edge].index_to;

            Eigen::Affine3d m_src = point_clouds_container.point_clouds.at(index_src).m_pose;
            Eigen::Affine3d m_trg = m_src * affine_matrix_from_pose_tait_bryan(edges[index_active_edge].relative_pose_tb);

            point_clouds_container.point_clouds.at(index_src).render(m_src, 1);
            point_clouds_container.point_clouds.at(index_trg).render(m_trg, 1);
        }
    }

    


}