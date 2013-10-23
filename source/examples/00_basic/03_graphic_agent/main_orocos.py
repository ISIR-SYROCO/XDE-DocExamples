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
import xdefw.rtt
import deploy.deployer as ddeployer

graph = xdefw.rtt.Task(ddeployer.load("graphic", "xdefw::graphics::GraphicAgent", "xdefw-agt-graphics"),
                       binding_class=xdefw.rtt.ObjectStringBinding, 
                       static_classes=['Viewer', 'Dio', 'agent', 'Connectors'])

##### Configure graphic agent
# Configure Ogre
ogre_resource_path = loader.share_dir+'/resources/ogre'

ogre_other_resources = [ # get the Ogre3D resources path (material definitions, etc.)
    loader.share_dir+'/resources/ogre/meshes',
    loader.share_dir+'/resources/ogre/materials/textures',
    loader.share_dir+'/resources/ogre/materials/textures/ocean',
    loader.share_dir+'/resources/ogre/materials/programs',
    loader.share_dir+'/resources/ogre/materials/core'
]

view = graph.s.Viewer

view.init() # initialize ogre

# Configure plugins
plugins = view.getDefaultPluginsConfig() # load ogre plugins, prefer OpenGL renderer
plugins['plugin_files']['render'] = \
       filter(lambda r: r.find("_GL") != -1 ,plugins['plugin_files']['render'])     # only use OpenGL. Directx may not be present
view.configurePlugins(plugins)
view.configureRenderer(view.getAvailableRenderers()[0])

view.configureDisplay(view.getDefaultDisplayConfig()) # configure default setup

view.setCoreResourcesPath(ogre_resource_path) # load ogre resources
view.setAdditionalResourceFileList(ogre_other_resources) # Add the custom material
view.setup() # load configuration specify above

##### Open viewer
view.createOgreWindowAndInput("Window") # create ogre scene
view.createScene("Scene")
view.bindSceneWindow("Scene", "Window", "Viewport", 0) # create a viewport a view of a scene in a window
                                                       # 0 is a viewport z-order (first one must be 0)
view.initInterfaces()   # init interfaces use to setup the graphical scene

###### Configure viewer
view.enableNavigation( True )       # camera navigation throw mouse
gInterface = graph.s.Interface( "Scene" ) # Graph Interface : mother of all interface
gInterface.LightInterface.enableFog(False) # no fog

# Set up brackground color
import xde.desc.core.color_pb2 as color_pb2
bg_color = color_pb2.Color()
bg_color.R, bg_color.G, bg_color.B = 0.7, 0.7, 0.9
gInterface.LightInterface.setBackgroundColor("Viewport", bg_color)

##### Run agent
print "start graphic agents"
graph.s.start()

##### Interactive shell
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


