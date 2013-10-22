##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import time


#-------------------------------------------------------------------------------
#
# Create graphic agent with agent module
#
#-------------------------------------------------------------------------------

##### Create graphic agent: agent module
import agents.graphic.simple
import agents.graphic.proto

graph = agents.graphic.simple.createAgent("graphic", 0)


##### Open viewer
gInterface, scene_name, window_name, viewport_name = agents.graphic.simple.setupSingleGLView(graph)


###### Configure viewer
agents.graphic.proto.configureBasicLights(gInterface)
agents.graphic.proto.configureBasicCamera(gInterface)
gInterface.SceneryInterface.showGround(True)
graph.s.Viewer.enableNavigation(True)



##### Run agent
print "start graphic agents"
graph.s.start()


##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


