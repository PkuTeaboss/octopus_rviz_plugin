// Cheng Zhang


#ifndef CAR_CREATOR_TOOL_H
#define CAR_CREATOR_TOOL_H

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>
#include <rviz/tool.h>
#include <rviz/viewport_mouse_event.h>
#include <rviz/render_panel.h>

namespace octopus_rviz_plugin
{
	class CarCreatorTool: public rviz::Tool
	{
		Q_OBJECT
	public:
		CarCreatorTool();

        virtual void onInitialize();

        virtual void activate();
        virtual void deactivate();

        virtual int processKeyEvent (QKeyEvent *event, rviz::RenderPanel *panel);
        virtual int processMouseEvent( rviz::ViewportMouseEvent& event );

    private:
        Ogre::SceneNode* moving_node_;

    };
}


#endif