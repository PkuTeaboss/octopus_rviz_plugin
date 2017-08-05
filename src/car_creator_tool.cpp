// Cheng Zhang

#include "car_creator_tool.h"



#include <rviz/geometry.h>
#include <rviz/mesh_loader.h>

#include <ros/ros.h>
#include <ros/console.h>
#include <OgreMath.h>

namespace octopus_rviz_plugin
{
	CarCreatorTool::CarCreatorTool()
    : rviz::Tool()
    {
        moving_node_ = NULL;
    }

    void CarCreatorTool::onInitialize() {

        std::string car_resource = "package://octopus_rviz_plugin/media/body_cog.dae";

        if( rviz::loadMeshFromResource( car_resource ).isNull() )
        {
            ROS_ERROR( "[CarCreatorTool] failed to load model resource '%s'.", car_resource.c_str() );
            return;
        }

        moving_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
        Ogre::Entity* entity = scene_manager_->createEntity( car_resource );
        moving_node_->attachObject( entity );
        moving_node_->setVisible( false );
        moving_node_->setScale( Ogre::Vector3(0.2, 0.2, 0.2));
    }


    void CarCreatorTool::activate() {
        if( moving_node_ ){
            //moving_node_->setVisible( true );
        }
    }

    void CarCreatorTool::deactivate() {
        if( moving_node_ ){
            moving_node_->setVisible( false );
        }
    }


    int CarCreatorTool::processKeyEvent(QKeyEvent* event, rviz::RenderPanel* panel)
    {
        Ogre::Radian leftTurn(0.1);
        Ogre::Radian rightTurn(-0.1);
        if (event->type() == QEvent::KeyPress) {

            if (event->key() == Qt::Key_A || event->key() == Qt::Key_Left) {
                moving_node_->roll(leftTurn);
            } else if (event->key() == Qt::Key_D || event->key() == Qt::Key_Right) {
                moving_node_->roll(rightTurn);
            }
        } 
        return 0;
    }

    int CarCreatorTool::processMouseEvent( rviz::ViewportMouseEvent& event ) {
        if( !moving_node_ )
        {
            return Render;
        }


        Ogre::Vector3 intersection;
        Ogre::Plane ground_plane( Ogre::Vector3::UNIT_Z, 0.0f );

        if( rviz::getPointOnPlaneFromWindowXY( event.viewport, ground_plane, event.x, event.y, intersection) ) {
            moving_node_->setVisible( true );
            moving_node_->setPosition( intersection );

            if( event.leftDown() )
            {
                Ogre::Quaternion tmp = moving_node_->getOrientation();
                ROS_INFO("Position: %f, %f, %f", intersection.x, intersection.y, intersection.z);
                ROS_INFO("Quaternion: %f, %f, %f, %f", tmp.x, tmp.y, tmp.z, tmp.w);
            }
        } else {
            moving_node_->setVisible( false );
        }

    }

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octopus_rviz_plugin::CarCreatorTool, rviz::Tool )