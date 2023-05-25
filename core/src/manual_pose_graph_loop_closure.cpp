#include <manual_pose_graph_loop_closure.h>

void ManualPoseGraphLoopClosure::Gui(){

}

void ManualPoseGraphLoopClosure::Render(const PointClouds &point_clouds_container){
    //render trajectory

    glColor3f(1.0f, 0.0f, 0.0f);
    glBegin(GL_LINE_STRIP);
        for(const auto &pc:point_clouds_container.point_clouds){
            glVertex3f(pc.m_pose.translation().x(), pc.m_pose.translation().y(), pc.m_pose.translation().z());
        }
    glEnd();
}