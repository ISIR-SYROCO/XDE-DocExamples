

##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()


##### Create physic and graphic agents
import agents.physic.core
import agents.graphic.simple
import agents.graphic.proto

time_step=.01

# Physic
def get_physic_agent():
    phy = agents.physic.core.createAgent("physic", 0)                        # physic agent
    ms = agents.physic.core.createGVMScene(phy, "main", time_step=time_step) # physic scene
    lmd = agents.physic.core.createXCDScene(phy, "xcd", "LMD", lmd_max=.01)  # collision scene

    ms.setGeometricalScene(lmd) # link physic scene and collision scene
    phy.s.setPeriod(0)
    
    return phy, ms, lmd


# Graphic
def get_graphic_agent():
    graph = agents.graphic.simple.createAgent("graphic", 0)                          # graphic scne
    gInterface, Nscene, Nwin, Nview = agents.graphic.simple.setupSingleGLView(graph) # graphic interface
    agents.graphic.proto.configureBasicLights(gInterface)
    agents.graphic.proto.configureBasicCamera(gInterface)
    graph.s.Viewer.enableNavigation(True)
    gInterface.SceneryInterface.showGround(True)
    
    return graph, gInterface




