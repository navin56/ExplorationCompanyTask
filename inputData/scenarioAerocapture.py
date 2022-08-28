#
#  ISC License
#
#  Copyright (c) 2022, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Overview
--------

Demonstrates a spacecraft performing aerocapture.  :ref:`tabularAtmosphere` is used
to read in a table of atmospheric density value for the planet.  A cannonball
drag effector (:ref:`dragDynamicEffector`) is used to simulate the atmospheric drag force.

The script is found in the folder ``basilisk/examples`` and executed by using::

      python3 scenarioAerocapture.py


Illustration of Simulation Results
----------------------------------
For the Earth aerocapture the following figures illustrate the simulation results.
The spacecraft first dips into the atmosphere and looses orbital energy to the point
of being capture by the planet.  The spacecraft has enough velocity to escape the
planet atmosphere at the end of the simulation time.  This is also illustrated by
the velocity versus altitude plot.  The density plot illustrates the result of the
tabular atmosphere model.

::

    show_plots = True, planetCase = `Earth`

.. image:: /_images/Scenarios/scenarioAerocapture5Earth.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerocapture4Earth.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerocapture3Earth.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerocapture2Earth.svg
   :align: center

For the Mars aerocapture scenario the orbit is adjusted to be suitable for this planet scenario.
Here too the spacecraft enters the atmosphere to burn off orbital energy and become
captured by the planet.

::

    show_plots = True, planetCase = `Mars`

.. image:: /_images/Scenarios/scenarioAerocapture5Mars.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerocapture4Mars.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerocapture3Mars.svg
   :align: center

.. image:: /_images/Scenarios/scenarioAerocapture2Mars.svg
   :align: center


"""

from operator import imul
import os
import numpy as np

# import general simulation support files
from Basilisk.utilities import SimulationBaseClass
import matplotlib.pyplot as plt
from Basilisk.utilities import macros
from Basilisk.utilities import orbitalMotion

# import simulation related support
from Basilisk.simulation import spacecraft
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import tabularAtmosphere, simpleNav
from Basilisk.utilities import unitTestSupport
from Basilisk.simulation import dragDynamicEffector
from Basilisk.architecture import messaging
from Basilisk.utilities.readAtmTable import readAtmTable
from Basilisk.utilities import vizSupport

# import Sensor Models.
from Basilisk.simulation import imuSensor, starTracker

#
# Basilisk Scenario Script and Integrated Test
#
# Purpose:  Aerocapture with cannonball drag and tabular atmosphere modules
# Author:   Mikaela Felix and Hanspeter Schaub
# Creation Date:  May 17, 2022
#

# filename = inspect.getframeinfo(inspect.currentframe()).filename
# path = os.path.dirname(os.path.abspath(filename))

# The path to the location of Basilisk
# Used to get the location of supporting data.
from Basilisk import __path__
bskPath = __path__[0]
fileName = os.path.basename(os.path.splitext(__file__)[0])


def sph2rv(xxsph):
    """
    NOTE: this function assumes inertial and planet-fixed frames are aligned
    at this time
    """

    r = xxsph[0]
    lon = xxsph[1]
    lat = xxsph[2]
    u = xxsph[3]
    gam = xxsph[4]
    hda = xxsph[5]

    NI = np.eye(3)
    IE = np.array([[np.cos(lat) * np.cos(lon), -np.sin(lon), -np.sin(lat) * np.cos(lon)],
                   [np.cos(lat) * np.sin(lon), np.cos(lon), -np.sin(lat) * np.sin(lon)],
                   [np.sin(lat), 0, np.cos(lat)]])
    ES = np.array([[np.cos(gam), 0, np.sin(gam)],
                   [-np.sin(gam) * np.sin(hda), np.cos(hda), np.cos(gam) * np.sin(hda)],
                   [-np.sin(gam) * np.cos(hda), -np.sin(hda), np.cos(gam) * np.cos(hda)]])

    e1_E = np.array([1,0,0])
    rvec_N = (r * NI @ IE) @ e1_E

    s3_S = np.array([0,0,1])
    uvec_N = u * ( NI @ IE @ ES) @ s3_S

    return rvec_N, uvec_N


def run(show_plots, planetCase):
    """
    The scenarios can be run with the followings setups parameters:

    Args:
        show_plots (bool): Determines if the script should display plots
        planetCase (string): Specify if a `Mars` or `Earth` arrival is simulated

    """

    # Create simulation variable names
    simProcessName = "simProcess"
    simTaskDynamics     = "simDynamics"
    simTaskSensImu      = "simIMU"
    simTaskSensStr      = "simSTR"

    #  Create a sim module as an empty container
    scSim = SimulationBaseClass.SimBaseClass()

    #
    #  create the simulation process
    #
    dynProcess = scSim.CreateNewProcess(simProcessName)

    # create the dynamics task and specify the integration update time
    simulationTimeStep = macros.sec2nano(0.025)
    imuTimeStep        = macros.sec2nano(0.01)
    strTimeStep        = macros.sec2nano(1)

    dynProcess.addTask(scSim.CreateNewTask(simTaskDynamics, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(simTaskSensImu, imuTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(simTaskSensStr, strTimeStep))

    # Construct algorithm and associated C++ container
    # change module to tabAtmo
    tabAtmo = tabularAtmosphere.TabularAtmosphere()   # update with current values
    tabAtmo.ModelTag = "tabularAtmosphere"            # update python name of test module
    atmoTaskName = "atmosphere"

    # define constants & load data
    if planetCase == 'Earth':
        r_eq = 6378136.6
        dataFileName = bskPath + '/supportData/AtmosphereData/EarthGRAMNominal.txt'
        altList, rhoList, tempList = readAtmTable(dataFileName, 'EarthGRAM')
    else:
        r_eq = 3397.2 * 1000
        dataFileName = bskPath + '/supportData/AtmosphereData/MarsGRAMNominal.txt'
        altList, rhoList, tempList = readAtmTable(dataFileName, 'MarsGRAM')

    # assign constants & ref. data to module
    tabAtmo.planetRadius = r_eq
    tabAtmo.altList = tabularAtmosphere.DoubleVector(altList)
    tabAtmo.rhoList = tabularAtmosphere.DoubleVector(rhoList)
    tabAtmo.tempList = tabularAtmosphere.DoubleVector(tempList)

    # Drag Effector
    projArea = 5.0  # Set drag area in m^2
    dragCoeff = 2.0  # Set drag ceofficient
    m_sc = 50.0    # kg

    dragEffector = dragDynamicEffector.DragDynamicEffector()
    dragEffector.ModelTag = "DragEff"

    dragEffectorTaskName = "drag"
    dragEffector.coreParams.projectedArea = projArea
    dragEffector.coreParams.dragCoeff = dragCoeff
    dragEffector.coreParams.comOffset = [1., 0., 0.]

    dynProcess.addTask(scSim.CreateNewTask(atmoTaskName, simulationTimeStep))
    dynProcess.addTask(scSim.CreateNewTask(dragEffectorTaskName, simulationTimeStep))
    scSim.AddModelToTask(atmoTaskName, tabAtmo)

    # Add test module to runtime call list
    scSim.AddModelToTask(simTaskDynamics, tabAtmo)

    #
    #   setup the simulation tasks/objects
    #

    # initialize spacecraft object and set properties
    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "spacecraftBody"
    scObject.hub.mHub = m_sc
    scObject.hub.sigma_BNInit = [macros.D2R * -45, macros.D2R * -90, 0.0]
    tabAtmo.addSpacecraftToModel(scObject.scStateOutMsg)

    # Set up spacecraft sensors.
    scImu    = imuSensor.ImuSensor()
    scImu.sensorPos_B = [1., 0.0, 0.0]
    scImu.senRotMax   = 2.0
    scImu.senTransMax = 40.0
    scSim.AddModelToTask(simTaskSensImu, scImu)
    scImu.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    scImu1    = imuSensor.ImuSensor()
    scImu1.sensorPos_B = [1., 0.2, -0.1]
    scImu1.senRotMax   = 2.0
    scImu1.senTransMax = 40.0
    scSim.AddModelToTask(simTaskSensImu, scImu1)
    scImu1.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    scImu2    = imuSensor.ImuSensor()
    scImu2.sensorPos_B = [0.2, 0.5, 0.3]
    scImu2.senRotMax   = 2.0
    scImu2.senTransMax = 40.0
    scSim.AddModelToTask(simTaskSensImu, scImu2)
    scImu2.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    scStr    = starTracker.StarTracker()
    scSim.AddModelToTask(simTaskSensStr, scStr)
    scStr.scStateInMsg.subscribeTo(scObject.scStateOutMsg)


    simpleNavObj = simpleNav.SimpleNav()
    scSim.AddModelToTask(simTaskSensImu, simpleNavObj)
    simpleNavObj.scStateInMsg.subscribeTo(scObject.scStateOutMsg)

    scObject.addDynamicEffector(dragEffector)

    # add spacecraft object to the simulation process
    scSim.AddModelToTask(simTaskDynamics, scObject)
    scSim.AddModelToTask(dragEffectorTaskName, dragEffector)
    # clear prior gravitational body and SPICE setup definitions

    dragEffector.atmoDensInMsg.subscribeTo(tabAtmo.envOutMsgs[0])

    # setup Gravity Body
    gravFactory = simIncludeGravBody.gravBodyFactory()
    if planetCase == 'Earth':
        planet = gravFactory.createEarth()
    else:
        planet = gravFactory.createMars()
    planet.isCentralBody = True  # ensure this is the central gravitational body
    mu = planet.mu

    # attach gravity model to spacecraft
    scObject.gravField.gravBodies = spacecraft.GravBodyVector(list(gravFactory.gravBodies.values()))

    if planetCase == 'Earth':
        r = 6503 * 1000
        u = 11.2 * 1000
        gam = -5.15 * macros.D2R
    else:
        r = (3397.2 + 125.) * 1000
        u = 6 * 1000
        gam = -10 * macros.D2R
    lon = 0
    lat = 0
    hda = np.pi/2
    xxsph = [r,lon,lat,u,gam,hda]
    rN, vN = sph2rv(xxsph)

    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # set the simulation time
    if planetCase == 'Earth':
        simulationTime = macros.sec2nano(60)
    else:
        simulationTime = macros.sec2nano(400)

    #
    #   Setup data logging before the simulation is initialized
    #

    dataLog = scObject.scStateOutMsg.recorder()
    imuLog         = scImu.sensorOutMsg.recorder()
    imuLog1        = scImu1.sensorOutMsg.recorder()
    imuLog2        = scImu2.sensorOutMsg.recorder()
    strLog         = scStr.sensorOutMsg.recorder()


    scSim.AddModelToTask(simTaskDynamics, dataLog)
    scSim.AddModelToTask(simTaskSensImu, imuLog)
    scSim.AddModelToTask(simTaskSensImu, imuLog1)
    scSim.AddModelToTask(simTaskSensImu, imuLog2)
    scSim.AddModelToTask(simTaskSensStr, strLog)

    #
    #   initialize Spacecraft States with initialization variables
    #
    scObject.hub.r_CN_NInit = rN  # m - r_CN_N
    scObject.hub.v_CN_NInit = vN  # m - v_CN_N

    # if this scenario is to interface with the BSK Viz, uncomment the following line
    vizSupport.enableUnityVisualization(scSim, simTaskDynamics, scObject
                                        , saveFile=fileName
                                        )
    #
    #   initialize Simulation
    #
    scSim.InitializeSimulation()

    #
    #   configure a simulation stop time time and execute the simulation run
    #
    scSim.ConfigureStopTime(simulationTime)
    scSim.ExecuteSimulation()

    #
    #   retrieve the logged data
    #
    posData = dataLog.r_BN_N
    velData = dataLog.v_BN_N
    
    imuData    = np.concatenate((imuLog.AccelPlatform, imuLog.AngVelPlatform), axis=1)
    imuData1   = np.concatenate((imuLog1.AccelPlatform, imuLog1.AngVelPlatform), axis=1)
    imuData2   = np.concatenate((imuLog2.AccelPlatform, imuLog2.AngVelPlatform), axis=1)

    # Log Sensor data to csv
    fmtSpec  = '%4.6f'
    np.save("imuSens.npy", imuData)
    np.save("imuSens1.npy", imuData1)
    np.save("imuSens2", imuData2)

    # GNSS
    gnssData   = np.concatenate((posData, velData), axis=1)
    np.save("gnssSens.npy", gnssData)

    # STR Values to csv
    strTime    = strLog.timeTag * macros.NANO2SEC
    strAtt     = strLog.qInrtl2Case
    strData    = np.column_stack((strTime, strAtt))
    np.save("strSens.npy", strData)

    np.set_printoptions(precision=16)

    figureList = {}
    plt.close("all")  # clears out plots from earlier test runs

    r = np.linalg.norm(posData, axis=1)
    v = np.linalg.norm(velData, axis=1)

    plt.figure(1)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(v/1e3, (r-r_eq)/1e3)
    plt.xlabel('velocity [km/s]')
    plt.ylabel('altitude [km]')
    plt.grid()
    pltName = fileName + "1" + planetCase
    plt.savefig(fname=pltName)
    figureList[pltName] = plt.figure(1)

    plt.figure(2)
    fig = plt.gcf()
    ax = fig.gca()
    plt.plot(dataLog.times()*macros.NANO2MIN, (r-r_eq)/1e3)
    plt.xlabel('time [min]')
    plt.ylabel('altitude [km]')
    plt.grid()
    pltName = fileName + "2" + planetCase
    plt.savefig(fname=pltName)
    figureList[pltName] = plt.figure(2)

    if show_plots:
        plt.show()
        plt.close("all")

    return figureList

    # close the plots being saved off to avoid over-writing old and new figures
if __name__ == '__main__':
    run(False, 'Earth')      # planet arrival case, can be Earth or Mars
