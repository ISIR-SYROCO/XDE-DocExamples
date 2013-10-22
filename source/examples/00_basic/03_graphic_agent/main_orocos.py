##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

import time


#-------------------------------------------------------------------------------
#
# Create graphic agent with orocos rtt.Task
#
#-------------------------------------------------------------------------------

##### Create graphic agent: orocos task
import dsimi.rtt
import deploy.deployer as ddeployer

graph = dsimi.rtt.Task(ddeployer.load("graphic", "dsimi::graphics::GraphicAgent", "dsimi-agt-graphics"),
                       binding_class=dsimi.rtt.ObjectStringBinding, 
                       static_classes=['Viewer', 'Dio', 'agent', 'Connectors'])


##### Configure graphic agent
view = graph.s.Viewer

view.init() # initialize ogre

# Configure plugins
plugins = view.getDefaultPluginsConfig() # load ogre plugins, prefer OpenGL renderer
plugins['plugin_files']['render'] = \
       filter(lambda r: r.find("_GL") != -1 ,plugins['plugin_files']['render'])     # only use OpenGL. Directx may not be present
view.configurePlugins(plugins)

# Configure Ogre
ogre_resources = [ # get the Ogre3D resources path (material definitions, etc.)
    loader.share_dir+'/resources/ogre/manikin',
    loader.share_dir+'/resources/ogre/meshes',
    loader.share_dir+'/resources/ogre/materials',
    loader.share_dir+'/resources/ogre/materials/textures',
    loader.share_dir+'/resources/ogre/materials/textures/ocean',
    loader.share_dir+'/resources/ogre/materials/programs',
    loader.share_dir+'/resources/ogre/materials/core'
]
view.setResourceFileList(ogre_resources) # load ogre resources
view.configureRenderer(view.getAvailableRenderers()[0])
view.configureDisplay(view.getDefaultDisplayConfig()) # configure default setup
view.setup() # load configuration specify above


##### Open viewer
view.createOgreWindowAndInput("Window") # create ogre scene
view.createScene("Scene")
view.bindSceneWindow("Scene", "Window", "Viewport", 0) # create a viewport a view of a scene in a window
                                                       # 0 is a viewport z-order (first one must be 0)


###### Configure viewer
view.enableNavigation( True )       # camera navigation throw mouse
view.initInterfaces()   # init interfaces use to setup the graphical scene
gInterface = graph.s.Interface( "Scene" ) # Graph Interface : mother of all interface
gInterface.LightInterface.enableFog(False) # no fog

# Set up brackground color
import color_pb2
bg_color = color_pb2.Color()
bg_color.R, bg_color.G, bg_color.B = 0.7, 0.7, 0.9
gInterface.LightInterface.setBackgroundColor("Viewport", bg_color)


##### Run agent
print "start graphic agents"
graph.s.start()


##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


