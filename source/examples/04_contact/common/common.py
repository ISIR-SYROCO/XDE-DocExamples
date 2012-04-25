

##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()

##### import for clock agent
import deploy.deployer as ddeployer
import dsimi.rtt

##### Create physic and graphic agents
import agents.physic.core
import agents.graphic.simple
import agents.graphic.proto


# Physic
def get_physic_agent(time_step=.01, lmd_max=.01):
    phy = agents.physic.core.createAgent("physic", 0)                               # physic agent
    ms = agents.physic.core.createGVMScene(phy, "main", time_step=time_step)        # physic scene
    lmd = agents.physic.core.createXCDScene(phy, "xcd", "LMD", lmd_max=lmd_max)     # collision scene

    ms.setGeometricalScene(lmd) # link physic scene and collision scene
    phy.s.setPeriod(time_step)
    
    return phy, ms, lmd


# Graphic
def get_graphic_agent(enableNavigation=True, showGround=True):
    graph = agents.graphic.simple.createAgent("graphic", 0)                          # graphic scne
    gInterface, Nscene, Nwin, Nview = agents.graphic.simple.setupSingleGLView(graph) # graphic interface
    agents.graphic.proto.configureBasicLights(gInterface)
    agents.graphic.proto.configureBasicCamera(gInterface)
    graph.s.Viewer.enableNavigation(enableNavigation)
    gInterface.SceneryInterface.showGround(showGround)
    
    return graph, gInterface


def get_clock_agent(time_step=0.01):
    ##### Create clock
    clock = dsimi.rtt.Task(ddeployer.load("clock", "dio::Clock", "dio-cpn-clock", "dio/component/"))
    clock.s.setPeriod(time_step)
    return clock

    
    
# Connection between agents
def connnectBodyState(phy, graph, world):
    ocb = phy.s.Connectors.OConnectorBodyStateList.new("ocb", "body_state")
    graph.s.Connectors.IConnectorBody.new("icb", "body_state_H", "mainScene")
    graph.getPort("body_state_H").connectTo(phy.getPort("body_state_H"))

    # add markers on bodies
    for b in world.scene.rigid_body_bindings:
        if len(b.graph_node) and len(b.rigid_body):
            ocb.addBody(str(b.rigid_body))

    return ocb


def connnectContactVisualization(phy, graph, contactPairs, maxProximity=.05, glyphScale=2.):

    occ = phy.s.Connectors.OConnectorContactBody.new("occ", "contacts")
    icc = graph.s.Connectors.IConnectorContacts.new("icc", "contacts", "mainScene")
    graph.getPort("contacts").connectTo(phy.getPort("contacts"))
    
    icc.setMaxProximity(maxProximity)
    icc.setGlyphScale(glyphScale)

    
    body_names = phy.s.GVM.Scene('main').getBodyNames()
    for elem1, elem2 in contactPairs:
        
        if elem1 in body_names:
            L1 = [elem1]
        else:
            r1 = phy.s.GVM.Robot(elem1)
            L1 = [r1.getSegmentRigidBody2(n) for n in r1.getSegmentNames()]

        if elem2 in body_names:
            L2 = [elem2]
        else:
            r2 = phy.s.GVM.Robot(elem2)
            L2 = [r2.getSegmentRigidBody2(n) for n in r2.getSegmentNames()]

        for e1 in L1:
            for e2 in L2:
                occ.addInteraction(e1, e2)

    
    return occ, icc


