##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()


#-------------------------------------------------------------------------------
#
# Create physic agent with orocos rtt.Task
#
#-------------------------------------------------------------------------------

##### Create physic agent: orocos task
import dsimi.rtt
import deploy.deployer as ddeployer

phy = dsimi.rtt.Task(ddeployer.load("physic", "dsimi::physics::PhysicAgent", "dsimi-agt-physics"),
                     binding_class=dsimi.rtt.ObjectStringBinding,
                     static_classes=['agent', 'Connectors'])


main_scene = phy.s.GVM.Scene.new("main_scene")            # create a physical scene
lmd        = phy.s.XCD.Scene.new_LMD("lmd", 0.05, 3*3.14) # create a collision scene using
                                                          # the Local Minimum Distance algorithm

##### Configure main scene
period = 0.01
main_scene.setIntegratorFlags(17)     # 17 = 1 + 16 : dynamic + gauss seidel solver
main_scene.setUcRelaxationFactor(1.0) # relaxation factor
main_scene.setFdvrFactor(0.2)         #

main_scene.setGeometricalScene(lmd)   # link collision and physic scene
main_scene.setTimeStep(period)        # set the integrator step (must be set after the previous function call)
phy.s.setPeriod(period)

##### Run agent
print "start physic agents"
phy.s.start()


##### Interactive shell
import dsimi.interactive
shell = dsimi.interactive.shell()
shell()


