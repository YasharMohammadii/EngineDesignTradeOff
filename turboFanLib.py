# ---------------------------------------------------- #
# IMPORTING LIBRARIES
# ---------------------------------------------------- #
# import concurrent.futures as cf
# import ctypes
# import multiprocessing as mp
import os
# import warnings
# import time
# from concurrent.futures.process import BrokenProcessPool
import numpy as np
import openmdao.api as om
import pycycle.api as pyc
# from openmdao.utils.om_warnings import SolverWarning


# ---------------------------------------------------- #
# TWO-SPOOL SEPERATE FLOW TURBOFAN ON-DESIGN ANALYSIS
# ---------------------------------------------------- #
class CFM56_3C1(pyc.Cycle):
    """
    This class defines a two-spool separate flow turbofan engine model for on-design analysis.
    The model is built using OpenMDAO and PyCycle libraries.

    Attributes
    ----------
    None

    Methods
    -------
    """

    def initialize(self):
        """
        Initializes the CFM56_3C1 class with default options.
        """
        super().initialize()
        self.options.declare('target_opr', types=float, default=30.0,
                             desc='Target operating pressure ratio for the engine')
        self.options.declare('target_T4', types=float, default=1300.0,
                             desc='Target turbine inlet temperature for the engine')
        self.options.declare('thermo_method', default='TABULAR',
                             desc='Thermodynamic property calculation method')
        self.options.declare('thermo_data', default=pyc.AIR_JETA_TAB_SPEC,
                             desc='Thermodynamic data specification')

    def setup(self):
        """
        Sets up the OpenMDAO group with the necessary components and connections for the turbofan engine model.
        """
        target_opr = self.options['target_opr']
        target_T4 = self.options['target_T4']

        # Define the components of the turbofan engine
        self.add_subsystem('fc', pyc.FlightConditions())
        self.add_subsystem('inlet', pyc.Inlet())
        self.add_subsystem('fan', pyc.Compressor(
            map_data=pyc.FanMap, map_extrap=True), promotes_inputs=[('Nmech', 'N1')])
        self.add_subsystem('splitter', pyc.Splitter())
        self.add_subsystem('lpc', pyc.Compressor(
            map_data=pyc.LPCMap, map_extrap=True), promotes_inputs=[('Nmech', 'N1')])
        self.add_subsystem('hpc', pyc.Compressor(
            map_data=pyc.HPCMap, map_extrap=True), promotes_inputs=[('Nmech', 'N2')])
        self.add_subsystem('burner', pyc.Combustor(fuel_type='FAR'))
        self.add_subsystem('hpt', pyc.Turbine(
            map_data=pyc.HPTMap, map_extrap=True), promotes_inputs=[('Nmech', 'N2')])
        self.add_subsystem('lpt', pyc.Turbine(
            map_data=pyc.LPTMap, map_extrap=True), promotes_inputs=[('Nmech', 'N1')])
        self.add_subsystem('core_nozz', pyc.Nozzle(
            nozzType='CV', lossCoef='Cv'))
        self.add_subsystem('byp_nozz', pyc.Nozzle(
            nozzType='CV', lossCoef='Cv'))
        self.add_subsystem('lp_shaft', pyc.Shaft(num_ports=3),
                           promotes_inputs=[('Nmech', 'N1')])
        self.add_subsystem('hp_shaft', pyc.Shaft(num_ports=2),
                           promotes_inputs=[('Nmech', 'N2')])
        self.add_subsystem('performance', pyc.Performance(
            num_nozzles=2, num_burners=1))

        # Connect the components (use PyCycle flow port names)
        self.pyc_connect_flow('fc.Fl_O', 'inlet.Fl_I')
        self.pyc_connect_flow('inlet.Fl_O', 'fan.Fl_I')
        self.pyc_connect_flow('fan.Fl_O', 'splitter.Fl_I')
        # Splitter outputs: Fl_O1 -> core (LPC), Fl_O2 -> bypass nozzle
        self.pyc_connect_flow('splitter.Fl_O2', 'byp_nozz.Fl_I')
        self.pyc_connect_flow('splitter.Fl_O1', 'lpc.Fl_I')
        self.pyc_connect_flow('lpc.Fl_O', 'hpc.Fl_I')
        self.pyc_connect_flow('hpc.Fl_O', 'burner.Fl_I')
        self.pyc_connect_flow('burner.Fl_O', 'hpt.Fl_I')
        self.pyc_connect_flow('hpt.Fl_O', 'lpt.Fl_I')
        self.pyc_connect_flow('lpt.Fl_O', 'core_nozz.Fl_I')

        self.connect('fan.trq', 'lp_shaft.trq_0')
        self.connect('lpc.trq', 'lp_shaft.trq_1')
        self.connect('lpt.trq', 'lp_shaft.trq_2')

        self.connect('hpc.trq', 'hp_shaft.trq_0')
        self.connect('hpt.trq', 'hp_shaft.trq_1')

        self.connect('core_nozz.Fg', 'performance.Fg_0')
        self.connect('byp_nozz.Fg', 'performance.Fg_1')
        self.connect('inlet.F_ram', 'performance.ram_drag')
        self.connect('burner.Wfuel', 'performance.Wfuel_0')

        # ------------------------------------------------------------------------------ #
        # COMPONENTS FOR LPC AND HPC WORK EQUALITY FOR A TARGET OPERATING PRESSURE RATIO
        # ------------------------------------------------------------------------------ #
        self.add_subsystem('comps_work_eq', om.ExecComp('W_diff = W_lpc - W_hpc', W_diff={
                           'val': 0, 'units': 'hp'}, W_lpc={'units': 'hp'}, W_hpc={'units': 'hp'}))
        self.connect('lpc.power', 'comps_work_eq.W_lpc')
        self.connect('hpc.power', 'comps_work_eq.W_hpc')
        self.add_subsystem('opr_eq', om.ExecComp('hpc_PR = target_opr / (fan_PR * lpc_PR)',
                                                 target_opr={
                                                     'val': target_opr, 'units': None},
                                                 fan_PR={'units': None},
                                                 lpc_PR={'units': None},
                                                 hpc_PR={'units': None}))
        self.connect('fan.PR', 'opr_eq.fan_PR')
        self.connect('lpc.PR', 'opr_eq.lpc_PR')

        # ----------------------------------------------------------------------------- #
        # BALANCE COMPONENTS
        # ----------------------------------------------------------------------------- #
        bal = om.BalanceComp()
        # BALANCE 1: LPC Pressure Ratio - for work equality
        # HPC PR is DERIVED from OPR (not balanced)
        self.connect("opr_eq.hpc_PR", "hpc.PR")
        bal.add_balance('PR_lpc', val=2.2, lower=1.1,
                        upper=10.0, eq_units='hp')
        self.connect('balance.PR_lpc', 'lpc.PR')
        self.connect('comps_work_eq.W_diff', 'balance.lhs:PR_lpc')
        # BALANCE 2: HPT Pressure Ratio - for HP shaft power balance
        bal.add_balance('PR_hpt', val=4.0, lower=1.1,
                        upper=20.0, eq_units='hp')
        self.connect('balance.PR_hpt', 'hpt.PR')
        self.connect('hp_shaft.pwr_net', 'balance.lhs:PR_hpt')

        # BALANCE 2: LPT Pressure Ratio - for LP shaft power balance
        bal.add_balance('PR_lpt', val=4.5, lower=1.1,
                        upper=20.0, eq_units='hp')
        self.connect('balance.PR_lpt', 'lpt.PR')
        self.connect('lp_shaft.pwr_net', 'balance.lhs:PR_lpt')

        # BALANCE 3: Inlet Mass Flow - for sizing the engine to a target thrust
        bal.add_balance('W', val=350.0, units='kg/s',
                        eq_units='lbf', lower=10.0, upper=1000.0)
        self.connect('balance.W', 'fc.W')
        self.connect('performance.Fn', 'balance.lhs:W')

        # BALANCE 4: Fuel-to-Air Ratio - for sizing the engine to a target turbine inlet temperature
        bal.add_balance('FAR', val=0.023, units=None,
                        eq_units='degC', lower=0.005, upper=0.05)
        self.connect('balance.FAR', 'burner.Fl_I:FAR')
        self.connect('burner.Fl_O:tot:T', 'balance.lhs:FAR')

        self.add_subsystem('balance', bal)
        self.set_input_defaults('balance.rhs:FAR', target_T4, units='degC')

        newton = self.nonlinear_solver = om.NewtonSolver()
        newton.options['atol'] = 1e-6
        newton.options['rtol'] = 1e-99
        newton.options['iprint'] = -1
        newton.options['maxiter'] = 50
        newton.options['solve_subsystems'] = True
        newton.options['max_sub_solves'] = 5
        ls = newton.linesearch = om.ArmijoGoldsteinLS()
        ls.options['maxiter'] = 3
        ls.options['rho'] = 0.75
        ls.options['iprint'] = -1
        self.linear_solver = om.DirectSolver()

        self.set_input_defaults('fc.W', 350.0, units='kg/s')
        super().setup()
