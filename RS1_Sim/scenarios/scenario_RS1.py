#
# RebelSat-1 Simulation
# 
# Description:
# 
#
# 
# 

# Import utilities
#from Basilisk.utilities import orbitalMotion, macros, vizSupport, simIncludeGravBody
from RS_BSK.dist3.Basilisk.utilities import orbitalMotion, macros, vizSupport, simIncludeGravBody

# Get current file path
import sys, os, inspect
filename = inspect.getframeinfo(inspect.currentframe()).filename
path = os.path.dirname(os.path.abspath(filename))

# Import master classes: simulation base class and scenario base class
sys.path.append(path + '/../')
sys.path.append(path + '/../models')
sys.path.append(path + '/../plotting')
from RS1_masters import RS1Sim, RS1Scenario
import RS1_Dynamics
import RS1_Fsw

# Import plotting files for your scenario
import RS1_Plotting as RS1_plt


# Create your own scenario child class
class scenario_RS1(RS1Sim, RS1Scenario):
    def __init__(self):
        super(scenario_RS1, self).__init__()
        self.name = 'scenario_RS1'

        # declare empty class variables
        self.sNavAttRec = None
        self.sNavTransRec = None

        self.set_DynModel(RS1_Dynamics)
        self.set_FswModel(RS1_Fsw)

        self.configure_initial_conditions()
        self.log_outputs()

        # if this scenario is to interface with the BSK Viz, uncomment the following line
        vizSupport.enableUnityVisualization(self, self.DynModels.taskName, self.DynModels.scObject
                                            # , saveFile=__file__
                                            , rwEffectorList=self.DynModels.rwStateEffector
                                            )

    def configure_initial_conditions(self):

        
        # setup Earth's gravitational field
        # TODO: not sure if I can use gravBodies from RS1DynamicModels::SetGravityBodies()
        earth = DynModels.gravFactory.createEarth()
        earth.isCentralBody = True                  # ensure this is the central gravitational body
        mu = earth.mu                               # gravitational constant of Earth
        
        # setup the orbit using classical orbit elements (https://en.wikipedia.org/wiki/Orbital_elements)
        periapsis = 413 * 1000                      # closest point of orbit to Earth (m)
        apoapsis = 422 * 1000                       # furthest point of orbit from Earth (m)
        oe = orbitalMotion.ClassicElements()
        oe.a = (periapsis + apoapsis) / 2.0         # semi-major axis (documentation says km, examples use m) (http://hanspeterschaub.info/basilisk/Documentation/utilities/orbitalMotion.html)
        oe.e = 0.0003492                            # eccentricity (no units)
        oe.i = 51.6439 * macros.D2R                 # inclination (rad)
        oe.Omega = 346.7648 * macros.D2R            # longitude of ascending node (aka where orbit passes reference plane) (rad)
        oe.omega = 165.4333 * macros.D2R            # argument of periapsis (aka oreintation of ellipse in orbital plane) (rad)
        oe.f = 298.6058 * macros.D2R                # true anomaly angle (aka position of orbiting body along ellipse at epoch time) (rad)
        rN, vN = orbitalMotion.elem2rv(mu, oe)      # position vector, velocity vector
        oe = orbitalMotion.rv2elem(mu, rN, vN)      # stores consistent initial orbit elements
        
        # define initial conditions of spacecraft states
        DynModels = self.get_DynModel()
        DynModels.scObject.hub.r_CN_NInit = rN      # inertial position of spacecraft (m)
        DynModels.scObject.hub.v_CN_NInit = vN      # inertial velocity of spacecraft (m/s)
        DynModels.scObject.hub.sigma_BNInit = [[0.1], [0.2], [-0.3]]        # initial attitude of B frame in Modified Rodrigues Parameters (MRP)
        DynModels.scObject.hub.omega_BN_BInit = [[0.001], [-0.01], [0.03]]  # initial angular velocity of B frame in B frame
        

    def log_outputs(self):
        # Dynamics process outputs
        DynModels = self.get_DynModel()
        self.sNavAttRec = DynModels.simpleNavObject.attOutMsg.recorder()
        self.sNavTransRec = DynModels.simpleNavObject.transOutMsg.recorder()
        self.AddModelToTask(DynModels.taskName, self.sNavAttRec)
        self.AddModelToTask(DynModels.taskName, self.sNavTransRec)

    def pull_outputs(self, showPlots):
        # Dynamics process outputs
        sigma_BN = self.sNavAttRec.sigma_BN
        r_BN_N = self.sNavTransRec.r_BN_N
        v_BN_N = self.sNavTransRec.v_BN_N

        # Plot results
        RS1_plt.clear_all_plots()
        timeLineSet = self.sNavAttRec.times() * macros.NANO2MIN
        RS1_plt.plot_orbit(r_BN_N)
        RS1_plt.plot_orientation(timeLineSet, r_BN_N, v_BN_N, sigma_BN)

        figureList = {}
        if showPlots:
            RS1_plt.show_all_plots()
        else:
            fileName = os.path.basename(os.path.splitext(__file__)[0])
            figureNames = ["orbit", "orientation"]
            figureList = RS1_plt.save_all_plots(fileName, figureNames)

        return figureList


def runScenario(scenario):

    # Initialize simulation
    scenario.InitializeSimulation()

    # Configure FSW mode
    scenario.modeRequest = 'standby'

    # Configure run time and execute simulation
    simulationTime = macros.min2nano(10.)
    scenario.ConfigureStopTime(simulationTime)

    scenario.ExecuteSimulation()


def run(showPlots):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        showPlots (bool): Determines if the script should display plots

    """

    # Configure a scenario in the base simulation
    TheScenario = scenario_RS1()
    runScenario(TheScenario)
    figureList = TheScenario.pull_outputs(showPlots)

    return figureList

if __name__ == "__main__":
    run(True)
