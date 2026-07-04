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
    prob.model.add_design_var('lpc.PR', lower=1.8, upper=5.0)

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
        'FAR': np.sum(prob.get_val('burner.Fl_I:FAR')),
        'Specific Thrust': np.sum(prob.get_val('performance.Fn', units='kN'))/np.sum(prob.get_val("fc.W", units="kg/s"))
    }

    PRs = {
        'Fan': np.sum(prob.get_val('fan.PR')),
        'LPC': np.sum(prob.get_val('lpc.PR')),
        'HPC': np.sum(prob.get_val('hpc.PR')),
        'HPT': np.sum(prob.get_val('hpt.PR')),
        'LPT': np.sum(prob.get_val('lpt.PR'))
    }

    burner_in_h = np.sum(prob.get_val('burner.Fl_I:tot:T', units='K')) * \
        np.sum(prob.get_val('burner.Fl_I:tot:Cp', units='MJ/(kg*K)'))
    burner_in_W = np.sum(prob.get_val('hpc.out_stat.W', units='kg/s'))
    burner_out_h = np.sum(prob.get_val('burner.Fl_I:tot:T', units='K')) * \
        np.sum(prob.get_val('burner.Fl_O:tot:Cp', units='MJ/(kg*K)'))
    burner_out_W = np.sum(prob.get_val('burner.Wout', units='kg/s'))
    burner_power = burner_out_h*burner_out_W - burner_in_h*burner_in_W
    burner_eff = burner_power * 1e6 / fuel_power
    mech = {
        'LP Shaft Speed': np.sum(prob.get_val('lp_shaft.Nmech', units='rpm')),
        'Fan Power': - np.sum(prob.get_val('fan.power', units='MW')),
        'Fan Efficiency': np.sum(prob.get_val('fan.eff')),
        'LPC Power': - np.sum(prob.get_val('lpc.power', units='MW')),
        'LPC Efficiency': np.sum(prob.get_val('lpc.eff')),
        'LPT Power': np.sum(prob.get_val('lpt.power', units='MW')),
        'LPT Efficiency': np.sum(prob.get_val('lpt.eff')),
        'HP Shaft Speed': np.sum(prob.get_val('hp_shaft.Nmech', units='rpm')),
        'HPC Power': - np.sum(prob.get_val('hpc.power', units='MW')),
        'HPC Efficiency': np.sum(prob.get_val('hpc.eff')),
        'HPT Power': np.sum(prob.get_val('hpt.power', units='MW')),
        'HPT Efficiency': np.sum(prob.get_val('hpt.eff')),
        'Burner Power': burner_power,
        'Burner Efficiency': burner_eff
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
        'Pressure Ratios': PRs,
        'Mechanical Parameters': mech
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
        print(f'TSFC: {Results['Performance']['TSFC']:.6f} g/(kN*s)')
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


def plotStage(ax, Result, stageParameter, color='black'):
    ax.plot([Result['Stages'][0], Result['Stages'][1], Result['Stages'][8]], [
        Result[stageParameter][0], Result[stageParameter][1], Result[stageParameter][8]],
        Result['Stages'][0:8], Result[stageParameter][0:8],
        marker='o', linewidth=2, color=color)
    ax.set_title(stageParameter)
    ax.grid(True, alpha=0.3)
    return


def TSdiagram(ax, Result, Label=True, color='black', textColor=None, alpha=1):
    if textColor == None:
        textColor = color
    T = []
    S = []
    L = []
    for i, t in enumerate(Result['Total Temperatures']):
        T.append(Result['Total Temperatures'][i])
    for i, s in enumerate(Result['Entropies']):
        S.append(Result['Entropies'][i])
    for i, l in enumerate(Result['Stages']):
        L.append(Result['Stages'][i][3:])
    ax.plot(S[:-1], T[:-1],
            [S[1], S[-1]], [T[1], T[-1]],
            marker='o', linewidth=2, color=color, alpha=alpha)
    if Label is not False:
        if type(Label) == bool:
            l = np.linspace(0, len(L) - 1, len(L), dtype=int)
        else:
            l = Label
        for i in l:
            if i >= 4:
                if i == 6 or i == 7:
                    ax.text(S[i]+30, T[i], L[i][0:9], verticalalignment='center',
                            weight='bold', color=textColor)
                else:
                    ax.text(S[i]+30, T[i], L[i], verticalalignment='center',
                            weight='bold', color=textColor)
            else:
                ax.text(S[i]-30, T[i], L[i], verticalalignment='center',
                        horizontalalignment='right', weight='bold', color=textColor)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(6475, 8525)
    ax.set_xlabel('Entropy S (J/(kg*K))', size=12)
    ax.set_ylabel('Total Temperature T (degC)', size=12)
    ax.tick_params(axis='both', labelsize=12)
    ax.set_title('T-S Diagram', weight='bold', size=14)


def PVdiagram(ax, Result, Label=True, color='black', textColor=None, alpha=1):
    if textColor == None:
        textColor = color
    P = []
    V = []
    L = []
    for i, p in enumerate(Result['Total Pressures']):
        P.append(Result['Total Pressures'][i])
    for i, v in enumerate(Result['Specifice Volumes']):
        V.append(Result['Specifice Volumes'][i])
    for i, l in enumerate(Result['Stages']):
        L.append(Result['Stages'][i][3:])
    ax.plot(V[:-1], P[:-1],
            [V[1], V[-1]], [P[1], P[-1]],
            marker='o', linewidth=2, color=color, alpha=alpha)
    if Label is not False:
        if type(Label) == bool:
            l = np.linspace(0, len(L) - 1, len(L), dtype=int)
        else:
            l = Label
        for i in l:
            if i >= 6:
                ax.text(V[i], P[i]+100, L[i], horizontalalignment='center', verticalalignment='bottom',
                        weight='bold', color=textColor)
            elif i <= 2:
                ax.text(V[i], P[i]-50, L[i], horizontalalignment='right', verticalalignment='top',
                        weight='bold', color=textColor)
            elif i == 3:
                ax.text(V[i], P[i]+50, L[i], horizontalalignment='right', verticalalignment='bottom',
                        weight='bold', color=textColor)
            elif i == 4:
                ax.text(V[i], P[i]+50, L[i], horizontalalignment='left', verticalalignment='bottom',
                        weight='bold', color=textColor)
            else:
                ax.text(V[i]+0.05, P[i]+50, L[i], verticalalignment='bottom',
                        horizontalalignment='left', weight='bold', color=textColor)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(-0.2, 2)
    ax.set_ylim(-100, 3300)
    ax.set_xlabel('Specifice Volume v (m**3/kg)', size=12)
    ax.set_ylabel('Total Pressure P (kPa)', size=12)
    ax.tick_params(axis='both', labelsize=12)
    ax.set_title('P-v Diagram', weight='bold', size=14)


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


def ParameterHeatMap(ax, Label, param, opr_range, T4_range, num, unit='', dir='TestCasesResults', aspectRatio=1):
    par = []
    for T4 in np.linspace(T4_range[0], T4_range[-1], num):
        T4_par = []
        for opr in np.linspace(opr_range[0], opr_range[-1], num):
            Result = json.load(open(f'{dir}/OPR-{opr}_T4-{T4}.json', 'r'))
            # Shape: [par in opr0, par in opr1, par in opr2, ...]
            T4_par.append(Result[Label][param])
        par.append(T4_par)  # Shape: [T4][opr]
    x0 = opr_range[0] - (opr_range[-1] - opr_range[0])/2/(num - 1)
    x1 = opr_range[-1] + (opr_range[-1] - opr_range[0])/2/(num - 1)
    y0 = T4_range[0] - (T4_range[-1] - T4_range[0])/2/(num - 1)
    y1 = T4_range[-1] + (T4_range[-1] - T4_range[0])/2/(num - 1)
    p = ax.imshow(par,
                  extent=[x0, x1, y0, y1], origin='lower')

    X, Y = np.meshgrid(np.linspace(
        opr_range[0], opr_range[-1], num), np.linspace(T4_range[0], T4_range[-1], num))
    ax.contour(X, Y, par, colors='w')

    ax.set_xticks([opr_range[0], opr_range[-1]])
    ax.set_yticks([T4_range[0], T4_range[-1]])

    asp = (opr_range[-1] - opr_range[0]) / \
        (T4_range[-1] - T4_range[0])*aspectRatio
    ax.set_aspect(asp)
    plt.colorbar(p).set_label(unit, size=12)
    ax.set_xlabel('OPR', size=12)
    ax.set_ylabel('T4 (°C)', size=12)
    ax.tick_params(axis='both', labelsize=12)
    if type(param) == int:
        ax.set_title(
            f'{Label}: {Result['Stages'][param]}', weight='bold', size=14)
    else:
        ax.set_title(f'{param}', weight='bold', size=14)


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
            continue
    return print(f'[Failed to Load Results]')


def OPRinfluencePlot(ax, label, param, unit, opr_range, T4_range, n, dir='TestCasesResults', cmap=plt.cm.viridis):
    for i, T4 in enumerate(np.linspace(T4_range[0], T4_range[-1], n)):
        OPR = []
        PAR = []
        for opr in np.linspace(opr_range[0], opr_range[-1], n):
            Result = readRes(opr, T4, dir=dir)
            PAR.append(Result[label][param])
            OPR.append(opr)

        norm = plt.Normalize(vmin=T4_range[0], vmax=T4_range[-1])
        colors = cmap(np.linspace(0, 1, n))
        p = ax.plot(OPR, PAR, linewidth=2,
                    color=colors[i], label=f'T4 = {T4}°C')

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = ax.figure.colorbar(
        sm, ax=ax).set_label('Turbine Inlet Temperature (°C)', size=12)

    ax.grid(alpha=0.3)
    ax.set_xlabel('OPR', size=12)
    ax.set_ylabel(unit, size=12)
    ax.tick_params(axis='both', labelsize=12)
    if type(param) == int:
        ax.set_title(
            f'{label}: {Result['Stages'][param]}', weight='bold', size=14)
    else:
        ax.set_title(f'{label}: {param}', weight='bold', size=14)


def T4influencePlot(ax, label, param, unit, T4_range, opr_range, n, dir='TestCasesResults', cmap=plt.cm.viridis):
    for i, opr in enumerate(np.linspace(opr_range[0], opr_range[-1], n)):
        T4 = []
        PAR = []
        for t4 in np.linspace(T4_range[0], T4_range[-1], n):
            Result = readRes(opr, t4, dir=dir)
            PAR.append(Result[label][param])
            T4.append(t4)

        norm = plt.Normalize(vmin=opr_range[0], vmax=opr_range[-1])
        colors = cmap(np.linspace(0, 1, n))
        ax.plot(T4, PAR, linewidth=2, color=colors[i])

    sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
    sm.set_array([])
    cbar = ax.figure.colorbar(
        sm, ax=ax).set_label('Operating Pressure Ratio', size=12)

    ax.grid(alpha=0.3)
    ax.set_xlabel('T4 (°C)', size=12)
    ax.set_ylabel(unit, size=12)
    ax.tick_params(axis='both', labelsize=12)
    if type(param) == int:
        ax.set_title(
            f'{label}: {Result['Stages'][param]}', weight='bold', size=14)
    else:
        ax.set_title(f'{label}: {param}', weight='bold', size=14)


def runIdealCase(target_opr, target_T4, Print=False, dir=None):
    prob = om.Problem(reports=False)
    prob.model = CFM56_3C1(target_opr=float(target_opr),
                           target_T4=float(target_T4))

    prob.setup()

    # Flight Conditions
    prob.set_val("fc.alt", 0.0, units="ft")
    prob.set_val("fc.MN", 0.001)
    prob.set_val("inlet.ram_recovery", 1)

    # Component Parameters
    prob.set_val("core_nozz.Cv", 1)
    prob.set_val("byp_nozz.Cv", 1)
    prob.set_val("burner.dPqP", 0.0)

    # Shaft Parameters
    # prob.set_val("hp_shaft.HPX", 202.0, units="hp")
    prob.set_val("hp_shaft.fracLoss", 0.0)
    prob.set_val("lp_shaft.fracLoss", 0.0)

    # Component Efficiencies
    prob.set_val("fan.eff", 1)
    prob.set_val("lpc.eff", 1)
    prob.set_val("hpc.eff", 1)
    prob.set_val("hpt.eff", 1)
    prob.set_val("lpt.eff", 1)

    # Fan Pressure Ratio and Bypass Ratio
    prob.set_val("fan.PR", 1.6)
    prob.set_val("splitter.BPR", 5.1)
    prob.model.add_design_var('lpc.PR', lower=1.8, upper=5.0)

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
        'FAR': np.sum(prob.get_val('burner.Fl_I:FAR')),
        'Specific Thrust': np.sum(prob.get_val('performance.Fn', units='kN'))/np.sum(prob.get_val("fc.W", units="kg/s"))
    }

    PRs = {
        'Fan': np.sum(prob.get_val('fan.PR')),
        'LPC': np.sum(prob.get_val('lpc.PR')),
        'HPC': np.sum(prob.get_val('hpc.PR')),
        'HPT': np.sum(prob.get_val('hpt.PR')),
        'LPT': np.sum(prob.get_val('lpt.PR'))
    }

    burner_in_h = np.sum(prob.get_val('burner.Fl_I:tot:T', units='K')) * \
        np.sum(prob.get_val('burner.Fl_I:tot:Cp', units='MJ/(kg*K)'))
    burner_in_W = np.sum(prob.get_val('hpc.out_stat.W', units='kg/s'))
    burner_out_h = np.sum(prob.get_val('burner.Fl_I:tot:T', units='K')) * \
        np.sum(prob.get_val('burner.Fl_O:tot:Cp', units='MJ/(kg*K)'))
    burner_out_W = np.sum(prob.get_val('burner.Wout', units='kg/s'))
    burner_power = burner_out_h*burner_out_W - burner_in_h*burner_in_W
    burner_eff = burner_power * 1e6 / fuel_power
    mech = {
        'LP Shaft Speed': np.sum(prob.get_val('lp_shaft.Nmech', units='rpm')),
        'Fan Power': - np.sum(prob.get_val('fan.power', units='MW')),
        'Fan Efficiency': np.sum(prob.get_val('fan.eff')),
        'LPC Power': - np.sum(prob.get_val('lpc.power', units='MW')),
        'LPC Efficiency': np.sum(prob.get_val('lpc.eff')),
        'LPT Power': np.sum(prob.get_val('lpt.power', units='MW')),
        'LPT Efficiency': np.sum(prob.get_val('lpt.eff')),
        'HP Shaft Speed': np.sum(prob.get_val('hp_shaft.Nmech', units='rpm')),
        'HPC Power': - np.sum(prob.get_val('hpc.power', units='MW')),
        'HPC Efficiency': np.sum(prob.get_val('hpc.eff')),
        'HPT Power': np.sum(prob.get_val('hpt.power', units='MW')),
        'HPT Efficiency': np.sum(prob.get_val('hpt.eff')),
        'Burner Power': burner_power,
        'Burner Efficiency': burner_eff
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
        'Pressure Ratios': PRs,
        'Mechanical Parameters': mech
    }

    """ SAVING RESULTS """
    if dir == None:
        with open(f'OPR-{target_opr}_T4-{target_T4}-Ideal.json', 'w') as f:
            json.dump(Results, f, indent=4)
    else:
        with open(f'{dir}/OPR-{target_opr}_T4-{target_T4}-Ideal.json', 'w') as f:
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
        print(f'TSFC: {Results['Performance']['TSFC']:.6f} g/(kN*s)')
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


def stageEstimate(Result):
    # N_total = log(lpc_PR) / log(PR_L_max) + log(hpc_PR) / log(PR_H_max)
    PR_L_MAX = 1.25
    PR_H_MAX = 1.38
    lpc_PR = Result['Pressure Ratios']['LPC']
    hpc_PR = Result['Pressure Ratios']['HPC']
    N_LPC = np.log(lpc_PR) / np.log(PR_L_MAX)
    N_HPC = np.log(hpc_PR) / np.log(PR_H_MAX)
    N_total = N_LPC + N_HPC
    return N_LPC, N_HPC, N_total
