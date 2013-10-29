##### Preambule
import loader
import deploy
deploy.loadTypekitsAndPlugins()


#-------------------------------------------------------------------------------
#
# Create physic agent with agent module
#
#-------------------------------------------------------------------------------

##### Create physic agent: agent module
import agents.physic.core as core

period = 0.01
phy         = core.createAgent("physic", 0)
main_scene  = core.createGVMScene(phy, "main_scene", time_step=period)
lmd         = core.createXCDScene(phy, "xcd", "LMD", lmd_max=3*3.14)

main_scene.setGeometricalScene(lmd)
phy.s.setPeriod(period)

##### Run agent
print "start physic agents"
phy.s.start()

##### Interactive shell
import xdefw.interactive
shell = xdefw.interactive.shell_console()
shell()


