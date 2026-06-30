from contextlib import contextmanager
import sys
from turboFanLib import *
import os
import matplotlib.pyplot as plt
import json


""" SETTING UP TEST CASE FOR CFM56_3C1 TURBOFAN ENGINE MODEL """


def runCase(target_opr, target_T4, Print=False, dir=None):
    prob = om.Problem(reports=False)
    prob.model = CFM56_3C1(target_opr=float(target_opr),
                           target_T4=float(target_T4))

    prob.setup()

    # Flight Conditions
    prob.set_val("fc.alt", 0.0, units="ft")
    prob.set_val("fc.MN", 0.001)
    prob.set_val("inlet.ram_recovery", 0.98)

    # Component Parameters
    prob.set_val("core_nozz.Cv", 0.94)
    prob.set_val("byp_nozz.Cv", 0.94)
    prob.set_val("burner.dPqP", 0.08)

    # Shaft Parameters
    prob.set_val("hp_shaft.HPX", 202.0, units="hp")
    prob.set_val("hp_shaft.fracLoss", 0.02)
    prob.set_val("lp_shaft.fracLoss", 0.02)

    # Component Efficiencies
    prob.set_val("fan.eff", 0.92)
    prob.set_val("lpc.eff", 0.90)
    prob.set_val("hpc.eff", 0.89)
    prob.set_val("hpt.eff", 0.90)
    prob.set_val("lpt.eff", 0.91)

    # Fan Pressure Ratio and Bypass Ratio
    prob.set_val("fan.PR", 1.6)
    prob.set_val("splitter.BPR", 5.1)

    # Rotor Speeds
    prob.set_val("N1", 5490.0, units="rpm")
    prob.set_val("N2", 15183.0, units="rpm")
    prob.set_val("balance.PR_hpt", 4.2)
    prob.set_val("balance.PR_lpt", 4.5)

    # Setting Target Values for Balances
    prob.set_val("balance.rhs:W", 100, units="kN")
    prob.set_val("balance.rhs:FAR", target_T4, units="degC")

    """ RUN THE MODEL """
    @contextmanager
    def silence_all_stdout():
        """Context manager to completely redirect stdout and stderr to devnull."""
        new_target = open(os.devnull, 'w')
        old_stdout = sys.stdout
        old_stderr = sys.stderr
        sys.stdout = new_target
        sys.stderr = new_target
        try:
            yield
        finally:
            sys.stdout = old_stdout
            sys.stderr = old_stderr
            new_target.close()

    # Use the context manager to wrap your execution call
    with silence_all_stdout():
        prob.run_model()

    """ CALCULATING THERMAL EFFICIENCY """
    v_in = np.sum(prob.get_val('inlet.Fl_I:stat:V', units='m/s'))

    core_W_in = np.sum(prob.get_val('lpc.Fl_I:stat:W', units='kg/s'))
    core_v_out = np.sum(prob.get_val('core_nozz.Fl_O:stat:V', units='m/s'))
    core_W_out = np.sum(prob.get_val('core_nozz.Fl_O:stat:W', units='kg/s'))
    core_kinetic_power = 0.5 * \
        (core_W_out * core_v_out**2 - core_W_in * v_in**2)

    byp_v_out = np.sum(prob.get_val('byp_nozz.Fl_O:stat:V', units='m/s'))
    byp_W_out = np.sum(prob.get_val('byp_nozz.Fl_O:stat:W', units='kg/s'))
    byp_kinetic_power = 0.5 * byp_W_out * (byp_v_out**2 - v_in**2)

    fuel_power = np.sum(prob.get_val('burner.Wfuel', units='kg/s')) * 43.0e6

    eta_th = (core_kinetic_power + byp_kinetic_power) / fuel_power

    """ CREATING RETURN OUTPUTS"""
    stages = [
        '0. Fan Inlet',
        '1. LPC Inlet',
        '2. HPC Inlet',
        '3. Burner Inlet',
        '4. HPT Inlet',
        '5. LPT Inlet',
        '6. Core Nozz Inlet',
        '7. Core Nozz Outlet',
        '8. Byp Nozz Outlet'
    ]

    totalTemps = [
        np.sum(prob.get_val('fan.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('lpc.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('hpc.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('burner.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('hpt.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('lpt.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('core_nozz.Fl_I:tot:T', units='degC')),
        np.sum(prob.get_val('core_nozz.Fl_O:tot:T', units='degC')),
        np.sum(prob.get_val('byp_nozz.Fl_O:tot:T', units='degC'))
    ]

    statTemps = [
        np.sum(prob.get_val('fan.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('lpc.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('hpc.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('burner.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('hpt.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('lpt.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('core_nozz.Fl_I:stat:T', units='degC')),
        np.sum(prob.get_val('core_nozz.Fl_O:stat:T', units='degC')),
        np.sum(prob.get_val('byp_nozz.Fl_O:stat:T', units='degC'))
    ]

    totalPress = [
        np.sum(prob.get_val('inlet.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('fan.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('lpc.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('hpc.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('burner.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('hpt.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('lpt.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('core_nozz.Fl_O:tot:P', units='kPa')),
        np.sum(prob.get_val('byp_nozz.Fl_O:tot:P', units='kPa'))
    ]

    statPress = [
        np.sum(prob.get_val('inlet.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('fan.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('lpc.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('hpc.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('burner.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('hpt.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('lpt.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('core_nozz.Fl_O:stat:P', units='kPa')),
        np.sum(prob.get_val('byp_nozz.Fl_O:stat:P', units='kPa'))
    ]

    entropies = [
        np.sum(prob.get_val('fan.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('lpc.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('hpc.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('burner.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('hpt.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('lpt.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('core_nozz.Fl_I:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('core_nozz.Fl_O:stat:S', units='J/(kg*K)')),
        np.sum(prob.get_val('byp_nozz.Fl_O:stat:S', units='J/(kg*K)'))
    ]

    specvol = [
        1/np.sum(prob.get_val('fan.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('lpc.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('hpc.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('burner.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('hpt.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('lpt.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('core_nozz.Fl_I:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('core_nozz.Fl_O:stat:rho', units='kg/m**3')),
        1/np.sum(prob.get_val('byp_nozz.Fl_O:stat:rho', units='kg/m**3'))
    ]

    performance = {
        'Thrust': np.sum(prob.get_val('performance.Fn', units='N')),
        'OPR': np.sum(prob.get_val('hpc.Fl_O:tot:P'))/np.sum(prob.get_val('inlet.Fl_I:tot:P')),
        'T4': np.sum(prob.get_val('burner.Fl_O:tot:T', units='degC')),
        'Thermal Efficiency': eta_th,
        'SFC': np.sum(prob.get_val("performance.Wfuel", units="g/s")*1000/(core_kinetic_power + byp_kinetic_power)),
        'TSFC': np.sum(prob.get_val("performance.Wfuel", units="g/s")/prob.get_val("performance.Fn", units="kN")),
        'Air Mass Flow Rate': np.sum(prob.get_val("fc.W", units="kg/s")),
        'Fuel Mass Flow Rate': np.sum(prob.get_val('performance.Wfuel', units='kg/s')),
        'Core EGT': np.sum(prob.get_val('core_nozz.Fl_O:tot:T', units='degC')),
        'Bypass EGT': np.sum(prob.get_val('byp_nozz.Fl_O:tot:T', units='degC')),
        'FAR': np.sum(prob.get_val('burner.Fl_I:FAR'))
    }

    PRs = {
        'Fan': np.sum(prob.get_val('fan.PR')),
        'LPC': np.sum(prob.get_val('lpc.PR')),
        'HPC': np.sum(prob.get_val('hpc.PR')),
        'HPT': np.sum(prob.get_val('hpt.PR')),
        'LPT': np.sum(prob.get_val('lpt.PR'))
    }

    Results = {
        'Stages': stages,
        'Total Temperatures': totalTemps,
        'Static Temperatures': statTemps,
        'Total Pressures': totalPress,
        'Static Pressures': statPress,
        'Entropies': entropies,
        'Specifice Volumes': specvol,
        'Performance': performance,
        'Pressure Ratios': PRs
    }

    """ SAVING RESULTS """
    if dir == None:
        with open(f'OPR-{target_opr}_T4-{target_T4}.json', 'w') as f:
            json.dump(Results, f, indent=4)
    else:
        with open(f'{dir}/OPR-{target_opr}_T4-{target_T4}.json', 'w') as f:
            json.dump(Results, f, indent=4)

    if Print:
        """ PRINT RESULTS """
        print("--------------------------------------------------")
        print("     CFM56-3C1 TURBOFAN ENGINE MODEL RESULTS")
        print("--------------------------------------------------")
        print('TARGET AND CALCULATED OPERATING CONDITIONS:')
        print('-------------------------------------------')
        print(f"Target Thrust: {100:.2f} kN")
        print(
            f"Calculated Thrust: {np.sum(prob.get_val('performance.Fn', units='N'))/1000:.2f} kN")
        print(f"Target OPR: {target_opr:.2f}")
        print(
            f"Calculated OPR: {np.sum(prob.get_val('hpc.Fl_O:tot:P'))/np.sum(prob.get_val('inlet.Fl_I:tot:P')):.2f}")
        print(f"Target Turbine Inlet Temperature: {target_T4:.2f} degC")
        print(
            f"Calculated Turbine Inlet Temperature: {np.sum(prob.get_val('burner.Fl_O:tot:T', units='degC')):.2f} degC")

        print('\n\n')
        print('ENGINE PERFORMANCE PARAMETERS:')
        print('------------------------------')
        print(f'Thermal Efficiency: {eta_th*100:.2f} %')
        print(f'SFC: {Results['Performance']['SFC']:.6f} g/kJ')
        print(f'TSFC: {Results['Performance']['TSFC']:.6f} g/kN/s')
        print(
            f'Air Mass Flow Rate: {np.sum(prob.get_val("fc.W", units="kg/s")):.2f} kg/s')
        print(
            f"Fuel Flow Rate: {np.sum(prob.get_val('performance.Wfuel', units='kg/s')):.4f} kg/s")
        print(
            f"Core Exhaust Gas Temperature: {np.sum(prob.get_val('core_nozz.Fl_O:tot:T', units='degC')):.2f} degC")
        print(
            f"Bypass Exhaust Gas Temperature: {np.sum(prob.get_val('byp_nozz.Fl_O:tot:T', units='degC')):.2f} degC")
        print(f"Bypass Ratio: {np.sum(prob.get_val('splitter.BPR')):.2f}")
        print(
            f"Fuel-to-Air Ratio (FAR): {np.sum(prob.get_val('burner.Fl_I:FAR')):.4f}")

        print('\n\n')
        print("PRESSURE RATIOS:")
        print('----------------')
        print(f"Fan Pressure Ratio: {np.sum(prob.get_val('fan.PR')):.2f}")
        print(
            f"Low Pressure Compressor Pressure Ratio: {np.sum(prob.get_val('lpc.PR')):.2f}")
        print(
            f"High Pressure Compressor Pressure Ratio: {np.sum(prob.get_val('hpc.PR')):.2f}")
        print(
            f"High Pressure Turbine Pressure Ratio: {np.sum(prob.get_val('hpt.PR')):.2f}")
        print(
            f"Low Pressure Turbine Pressure Ratio: {np.sum(prob.get_val('lpt.PR')):.2f}")

    return Results


def plotStage(Result, stageParameter, color='black'):
    plt.plot([Result['Stages'][0], Result['Stages'][1], Result['Stages'][8]], [
        Result[stageParameter][0], Result[stageParameter][1], Result[stageParameter][8]],
        Result['Stages'][0:8], Result[stageParameter][0:8],
        marker='o', linewidth=2, color=color)
    plt.title(stageParameter)
    plt.grid(True, alpha=0.3)
    return


def TSdiagram(Result, Label=True, color='black'):
    T = []
    S = []
    L = []
    for i, t in enumerate(Result['Static Temperatures']):
        T.append(Result['Static Temperatures'][i])
    for i, s in enumerate(Result['Entropies']):
        S.append(Result['Entropies'][i])
    for i, l in enumerate(Result['Stages']):
        L.append(Result['Stages'][i])
    plt.plot(S[:-1], T[:-1],
             [S[1], S[-1]], [T[1], T[-1]],
             marker='o', linewidth=2, color=color)
    if Label:
        for i in np.linspace(0, len(L) - 1, len(L), dtype=int):
            if L[i][0] >= '4':
                plt.text(S[i]+20, T[i], L[i], verticalalignment='center',
                         weight='bold', color=color)
            else:
                plt.text(S[i]-20, T[i], L[i], verticalalignment='center',
                         horizontalalignment='right', weight='bold', color=color)
    plt.grid(True, alpha=0.3)
    plt.xlabel('Entropy S (J/(kg*K))')
    plt.ylabel('Static Temperature T (degC)')
    plt.title('T-S Diagram')


def TestCaseDoc(opr_range, T4_range, num, dir='TestCasesResults', overwrite=False, Print=False):
    """ SOLVING TEST CASES """
    target_opr = np.linspace(opr_range[0], opr_range[-1], num)
    target_T4 = np.linspace(T4_range[0], T4_range[-1], num)

    for opr in target_opr:
        for T4 in target_T4:
            try:
                Result = json.load(open(f'{dir}/OPR-{opr}_T4-{T4}.json', 'r'))
                if overwrite:
                    runCase(opr, T4, Print=Print, dir=dir)
                    print(
                        f'[OPR = {opr}, T4 = {T4}] Case available. Results overwritten to {dir}/OPR-{opr}_T4-{T4}.json')
                else:
                    print(
                        f'[OPR = {opr}, T4 = {T4}] Case available. Run skipped.')
                    continue
            except:
                runCase(opr, T4, Print=Print, dir=dir)
                print(
                    f'[OPR = {opr}, T4 = {T4}] New case. Results added to {dir}/OPR-{opr}_T4-{T4}.json')


def ParameterHeatMap(Label, param, opr_range, T4_range, num, plotname, dir='TestCasesResults'):
    par = []
    for opr in np.linspace(opr_range[0], opr_range[-1], num):
        opr_par = []
        for T4 in np.linspace(T4_range[0], T4_range[-1], num):
            Result = json.load(open(f'{dir}/OPR-{opr}_T4-{T4}.json', 'r'))
            opr_par.append(Result[Label][param])
        par.append(opr_par)

    plt.imshow(par,
               extent=[
                   T4_range[0] - (T4_range[-1] - T4_range[0])/2/(num - 1),
                   T4_range[-1] + (T4_range[-1] - T4_range[0])/2/(num - 1),
                   opr_range[0] - (opr_range[-1] - opr_range[0])/2/(num - 1),
                   opr_range[-1] + (opr_range[-1] - opr_range[0])/2/(num - 1)])

    plotname.set_xticks(np.linspace(T4_range[0], T4_range[-1], num))
    plotname.set_yticks(np.linspace(opr_range[0], opr_range[-1], num))
    plotname.set_aspect((T4_range[-1] - T4_range[0]) /
                        (opr_range[-1] - opr_range[0]))
    plt.xlabel('Turbine Inlet Temperature (degC)')
    plt.ylabel('Operating Pressure Ratio')
    if type(param) == int:
        plt.title(f'{Label}: {Result['Stages'][param]}')
    else:
        plt.title(f'{Label}: {param}')


def PVdiagram(Result, Label=True, color='black'):
    P = []
    V = []
    L = []
    for i, p in enumerate(Result['Static Pressures']):
        P.append(Result['Static Pressures'][i])
    for i, v in enumerate(Result['Specifice Volumes']):
        V.append(Result['Entropies'][i])
    for i, l in enumerate(Result['Stages']):
        L.append(Result['Stages'][i])
    plt.plot(V[:-1], P[:-1],
             [V[1], V[-1]], [P[1], P[-1]],
             marker='o', linewidth=2, color=color)
    if Label:
        for i in np.linspace(0, len(L) - 1, len(L), dtype=int):
            if L[i][0] >= '4':
                plt.text(V[i]+20, P[i], L[i], verticalalignment='center',
                         weight='bold', color=color)
            else:
                plt.text(V[i]-20, P[i], L[i], verticalalignment='center',
                         horizontalalignment='right', weight='bold', color=color)
    plt.grid(True, alpha=0.3)
    plt.xlabel('Specifice Volume v (m**3/kg)')
    plt.ylabel('Static Pressure P (kPa)')
    plt.title('P-v Diagram')


def readRes(opr, T4, dir=None):
    for func in [int, float]:
        try:
            if dir == None:
                Result = json.load(
                    open(f'OPR-{func(opr)}_T4-{func(T4)}.json', 'r'))
            else:
                Result = json.load(
                    open(f'{dir}/OPR-{func(opr)}_T4-{func(T4)}.json', 'r'))
            return Result
        except:
            pass
    return print(f'[Failed to Load Results]')
